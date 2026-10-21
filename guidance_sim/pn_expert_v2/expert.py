"""PN expert v2: ballistic predictive ZEM guidance, separate from legacy control.

This is a gravity/drag-aware modification of proportional navigation, not the
classical ``N * closing_speed * LOS_rate`` law. An unforced prediction gives
zero-effort miss (ZEM) at the *nose/target-plane* intersection. The numerical
terminal sensitivity J includes the finite overload response. The requested
overload is ``-(N/2) pinv(J) ZEM``: for a double integrator with no lag this
reduces to ``N * ZEM / time_to_go**2`` in acceleration units. N=3 is the usual
ZEM form of PN. The resulting command is limited to a shared overload disk.

Only measured/estimated state and an estimated target are accepted here. There
is no scene, target-truth, image renderer, actuator, or simulator dependency.
Pitch/yaw overloads are positive down/right in the velocity frame, respectively.
"""
from __future__ import annotations

import copy
import math

import numpy as np


EXPERT_DEFAULTS = dict(max_control_g=.25, response_tau_s=.10,
    max_slew_g_s=2.5, update_period_s=.02, prediction_dt_s=.025,
    png_N=3., vision_timeout_s=.15, drag_enabled=True)


def _vector(value, size, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain {size} finite numbers")
    return result.copy()


def _limit(vector, maximum):
    length = math.hypot(*vector)
    return vector.copy() if length <= maximum else vector*(maximum/length)


class BallisticPNExpert:
    """Hold a bounded predictive PN request between observation updates.

    ``wind_W_mps`` is an optional *estimate*, fixed at zero by the default
    vision experiment. Unknown true wind must not be passed as privileged input.
    """

    name = "ballistic_ZEM_PN"

    def __init__(self, config=None, *, mass_kg, rho_kg_m3, area_m2,
                 drag_coefficient, gravity_W, plane_normal_W, nose_offset_L,
                 wind_W_mps=(0., 0., 0.)):
        config = {} if config is None else config
        if not isinstance(config, dict) or set(config)-set(EXPERT_DEFAULTS):
            raise ValueError("Unknown ballistic PN expert configuration field")
        self.config = dict(EXPERT_DEFAULTS, **copy.deepcopy(config))
        for key, value in self.config.items():
            if key == "drag_enabled":
                if not isinstance(value, bool):
                    raise ValueError("drag_enabled must be boolean")
            elif isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0.:
                raise ValueError(f"{key} must be positive and finite")
            else:
                self.config[key] = float(value)
        for name, value in (("mass_kg", mass_kg), ("rho_kg_m3", rho_kg_m3),
                            ("area_m2", area_m2), ("drag_coefficient", drag_coefficient)):
            if isinstance(value, bool) or not math.isfinite(value) or value < 0. or (name == "mass_kg" and value == 0.):
                raise ValueError(f"{name} is outside the physical range")
        self.gravity_W = _vector(gravity_W, 3, "gravity_W")
        self.gravity = float(np.linalg.norm(self.gravity_W))
        if self.gravity <= 0.:
            raise ValueError("gravity_W must be nonzero")
        self.normal = _vector(plane_normal_W, 3, "plane_normal_W")
        length = float(np.linalg.norm(self.normal))
        if length < 1e-12:
            raise ValueError("plane_normal_W must be nonzero")
        self.normal /= length
        self.nose_L = _vector(nose_offset_L, 3, "nose_offset_L")
        self.wind_W = _vector(wind_W_mps, 3, "wind_W_mps")
        self.drag_factor = (.5*float(rho_kg_m3)*float(area_m2)*float(drag_coefficient)/float(mass_kg)
                            if self.config["drag_enabled"] else 0.)
        axis = np.array([0., 0., 1.]) if abs(self.normal[2]) < .9 else np.array([0., 1., 0.])
        tangent = np.cross(axis, self.normal)
        tangent /= np.linalg.norm(tangent)
        self.plane_basis = np.column_stack((tangent, np.cross(self.normal, tangent)))
        self._last_update_s = None
        self._command_g = np.zeros(2)
        self._diagnostics = dict(mode=self.name, status="waiting_for_observation")

    @staticmethod
    def _frame(velocity):
        """Batched zero-roll velocity frames, including a vertical fallback."""
        vx, vy, vz = velocity.T
        horizontal = np.hypot(vx, vy)
        speed = np.linalg.norm(velocity, axis=1)
        hs = np.maximum(horizontal, 1e-12)
        ss = np.maximum(speed, 1e-12)
        forward = velocity/ss[:, None]
        left = np.column_stack((-vy/hs, vx/hs, np.zeros_like(vx)))
        up = np.column_stack((-vz*vx/(ss*hs), -vz*vy/(ss*hs), horizontal/ss))
        vertical = horizontal < 1e-12
        left[vertical] = [0., 1., 0.]
        up[vertical] = np.column_stack((-np.sign(vz[vertical]), np.zeros(np.sum(vertical)), np.zeros(np.sum(vertical))))
        return forward, left, up

    def _nose(self, position, velocity):
        forward, left, up = self._frame(velocity)
        return position+self.nose_L[0]*forward+self.nose_L[1]*left+self.nose_L[2]*up

    def _response(self, actual, target, dt):
        error = actual-target
        distance = np.linalg.norm(error, axis=1)
        slew, tau = self.config["max_slew_g_s"], self.config["response_tau_s"]
        switch = np.maximum(0., distance/slew-tau)
        remaining = np.where(dt <= switch, distance-slew*dt,
            np.minimum(distance, slew*tau)*np.exp(-np.maximum(0., dt-switch)/tau))
        fraction = np.divide(remaining, distance, out=np.zeros_like(distance), where=distance > 0.)
        return target+error*fraction[:, None]

    def _acceleration(self, velocity, actual):
        _, left, up = self._frame(velocity)
        acceleration = self.gravity_W-self.gravity*(actual[:, 0, None]*up+actual[:, 1, None]*left)
        if self.drag_factor:
            relative = velocity-self.wind_W
            acceleration -= self.drag_factor*np.linalg.norm(relative, axis=1)[:, None]*relative
        return acceleration

    def _predict(self, position, velocity, actual, commands, target):
        """RK4 predictions with cubic-state nose intersections in the last step.

        Linear nose interpolation would introduce a millimetre-scale ZEM bias
        that is strongly amplified when only milliseconds of authority remain.
        Cubic Hermite interpolation retains the integrator's local accuracy.
        """
        count = len(commands)
        p = np.tile(position, (count, 1))
        v = np.tile(velocity, (count, 1))
        n = np.tile(actual, (count, 1))
        nose = self._nose(p, v)
        distance = (nose-target) @ self.normal
        initial_sign = np.sign(distance)
        hit = np.abs(distance) < 1e-10
        impact = np.full((count, 3), np.nan)
        impact[hit] = nose[hit]
        times = np.full(count, np.nan)
        times[hit] = 0.
        normal_speed = abs(float(velocity @ self.normal))
        estimate = abs(float(distance[0]))/max(normal_speed, 1.)
        horizon = min(8., max(1., 1.8*estimate+1.))
        dt = self.config["prediction_dt_s"]
        for step in range(int(math.ceil(horizon/dt))):
            n_half = self._response(n, commands, .5*dt)
            n_end = self._response(n, commands, dt)
            a1 = self._acceleration(v, n)
            v2 = v+.5*dt*a1
            a2 = self._acceleration(v2, n_half)
            v3 = v+.5*dt*a2
            a3 = self._acceleration(v3, n_half)
            v4 = v+dt*a3
            a4 = self._acceleration(v4, n_end)
            p_new = p+dt*(v+2*v2+2*v3+v4)/6.
            v_new = v+dt*(a1+2*a2+2*a3+a4)/6.
            nose_new = self._nose(p_new, v_new)
            distance_new = (nose_new-target) @ self.normal
            crossed = (~hit) & (initial_sign*distance_new <= 0.)
            if np.any(crossed):
                fraction = distance[crossed]/(distance[crossed]-distance_new[crossed])
                p0, p1, v0, v1 = p[crossed], p_new[crossed], v[crossed], v_new[crossed]
                for _ in range(4):
                    s = fraction[:, None]
                    p_cross = ((2*s**3-3*s**2+1)*p0+(s**3-2*s**2+s)*dt*v0+
                               (-2*s**3+3*s**2)*p1+(s**3-s**2)*dt*v1)
                    v_cross = ((6*s**2-6*s)*p0/dt+(3*s**2-4*s+1)*v0+
                               (-6*s**2+6*s)*p1/dt+(3*s**2-2*s)*v1)
                    nose_cross = self._nose(p_cross, v_cross)
                    error = (nose_cross-target) @ self.normal
                    derivative = dt*(v_cross @ self.normal)
                    increment = np.divide(error, derivative, out=np.zeros_like(error), where=np.abs(derivative) > 1e-12)
                    fraction = np.clip(fraction-increment, 0., 1.)
                s = fraction[:, None]
                p_cross = ((2*s**3-3*s**2+1)*p0+(s**3-2*s**2+s)*dt*v0+
                           (-2*s**3+3*s**2)*p1+(s**3-s**2)*dt*v1)
                v_cross = ((6*s**2-6*s)*p0/dt+(3*s**2-4*s+1)*v0+
                           (-6*s**2+6*s)*p1/dt+(3*s**2-2*s)*v1)
                impact[crossed] = self._nose(p_cross, v_cross)
                times[crossed] = (step+fraction)*dt
                hit[crossed] = True
            if np.all(hit):
                break
            p, v, n, nose, distance = p_new, v_new, n_end, nose_new, distance_new
        return impact, times

    def update(self, t_s, position_W, velocity_W, actual_g, target_est_W,
               valid_age_s=0.):
        """Return ``(pitch/yaw target g, JSON-compatible diagnostics)``.

        Loss of target validity removes the request immediately, without
        resetting actual overload or any simulator state. Dynamics retain their
        normal lag while that request decays.
        """
        if isinstance(t_s, bool) or not math.isfinite(t_s):
            raise ValueError("t_s must be finite")
        position = _vector(position_W, 3, "position_W")
        velocity = _vector(velocity_W, 3, "velocity_W")
        actual = _vector(actual_g, 2, "actual_g")
        age = float(valid_age_s)
        if math.isnan(age) or age < 0.:
            raise ValueError("valid_age_s must be nonnegative")
        if target_est_W is None or age > self.config["vision_timeout_s"]:
            self._command_g[:] = 0.
            self._last_update_s = None
            self._diagnostics = dict(mode=self.name, status="target_unavailable",
                                     target_age_s=age if math.isfinite(age) else None)
            return self._command_g.copy(), copy.deepcopy(self._diagnostics)
        target = _vector(target_est_W, 3, "target_est_W")
        if self._last_update_s is not None and 0. <= t_s-self._last_update_s < self.config["update_period_s"]-1e-10:
            return self._command_g.copy(), copy.deepcopy(self._diagnostics)
        maximum = self.config["max_control_g"]
        epsilon = min(.02, maximum*.2)
        commands = np.array([[0., 0.], [epsilon, 0.], [-epsilon, 0.],
                             [0., epsilon], [0., -epsilon]])
        impacts, times = self._predict(position, velocity, actual, commands, target)
        diagnostic = dict(mode=self.name, target_age_s=age, max_control_g=maximum,
                          prediction_dt_s=self.config["prediction_dt_s"])
        self._last_update_s = float(t_s)
        if not np.all(np.isfinite(impacts)) or not np.all(np.isfinite(times)):
            self._command_g[:] = 0.
            diagnostic["status"] = "no_predicted_plane_crossing"
        else:
            errors = (impacts-target) @ self.plane_basis
            jacobian = np.column_stack(((errors[1]-errors[2])/(2*epsilon),
                                        (errors[3]-errors[4])/(2*epsilon)))
            singular = np.linalg.svd(jacobian, compute_uv=False)
            diagnostic.update(time_to_go_s=float(times[0]),
                predicted_unforced_impact_W=impacts[0].tolist(),
                predicted_unforced_miss_m=float(np.linalg.norm(errors[0])),
                zem_plane_m=errors[0].tolist(), sensitivity_m_per_g=jacobian.tolist(),
                minimum_sensitivity_m_per_g=float(singular[-1]))
            if times[0] < .015 or singular[-1] < 1e-5:
                # The plane is already reached: avoid a singular terminal spike.
                self._command_g[:] = 0.
                diagnostic["status"] = "terminal_no_remaining_authority"
            else:
                request = -(self.config["png_N"]/2.)*np.linalg.solve(jacobian, errors[0])
                self._command_g = _limit(request, maximum)
                diagnostic.update(status="active", unconstrained_command_g=request.tolist(),
                    command_g=self._command_g.tolist(), saturated=math.hypot(*request) > maximum,
                    predicted_linear_residual_m=float(np.linalg.norm(errors[0]+jacobian@self._command_g)))
        self._diagnostics = diagnostic
        return self._command_g.copy(), copy.deepcopy(diagnostic)

    def capture_state(self):
        return dict(version=1, config=copy.deepcopy(self.config),
            last_update_s=self._last_update_s, command_g=self._command_g.tolist(),
            diagnostics=copy.deepcopy(self._diagnostics))

    def restore_state(self, state):
        if not isinstance(state, dict) or state.get("version") != 1 or state.get("config") != self.config:
            raise ValueError("Incompatible ballistic PN expert snapshot")
        command = _vector(state.get("command_g"), 2, "snapshot command_g")
        if math.hypot(*command) > self.config["max_control_g"]+1e-10:
            raise ValueError("Snapshot command exceeds the overload limit")
        update = state.get("last_update_s")
        if update is not None and (isinstance(update, bool) or not isinstance(update, (int, float)) or not math.isfinite(update)):
            raise ValueError("Invalid expert update time")
        diagnostic = state.get("diagnostics")
        if not isinstance(diagnostic, dict):
            raise ValueError("Invalid expert diagnostics")
        self._command_g, self._last_update_s = command, update
        self._diagnostics = copy.deepcopy(diagnostic)
