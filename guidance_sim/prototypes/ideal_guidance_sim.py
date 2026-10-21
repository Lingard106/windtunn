"""Independent guidance experiment with bounded, finite-response control force.

The default input is the existing rendered-camera detector. ``truth`` is an
explicit, labelled upper-target LOS comparison. Motion is a gravity-aware RK4
point mass with gravity and optional quadratic drag; attitude follows velocity
with zero Euler roll. MuJoCo renders and
updates geometry only: no rigid-body stepping, AP, servo, PWM or aerodynamic
control loads are executed. Zero control gives an unpowered ballistic trajectory;
with drag disabled this is the analytic parabola, not gravity compensation.
"""
from __future__ import annotations

import argparse
import copy
from dataclasses import replace
from functools import partial
import json
import math
from pathlib import Path
import sys

import mujoco
import numpy as np

if __package__:
    from .dart_demo import DemoSimulation, load_config, validate_config
    from .dart_viewer import DartViewer
    from .flight_sensors import GRAVITY, sample_flight_vision
    from .guidedemo_control import FlightController, load_flight_config
    from .imu_noise import ImuNoise
    from .openmv_tracker import OpenMVTracker
    from .playback import SampleClock
    from .simulation_setup import resolve_startup_paths
    from .vision_settings import VisionSettings
else:
    from dart_demo import DemoSimulation, load_config, validate_config
    from dart_viewer import DartViewer
    from flight_sensors import GRAVITY, sample_flight_vision
    from guidedemo_control import FlightController, load_flight_config
    from imu_noise import ImuNoise
    from openmv_tracker import OpenMVTracker
    from playback import SampleClock
    from simulation_setup import resolve_startup_paths
    from vision_settings import VisionSettings

PROJECT = Path(__file__).resolve().parents[1]
IDEAL_PROFILE = PROJECT / "profiles/ideal_guidance.json"
GRAVITY_W = np.array([0., 0., -GRAVITY])
DYNAMICS_DEFAULT = dict(max_control_g=.25, response_tau_s=.10,
                        max_slew_g_s=2.5, drag_enabled=True)
EXPERT_DEFAULT = dict(law="classical_pn", update_period_s=.02, prediction_dt_s=.025,
                      vision_filter_tau_s=.08, png_N=3.,
                      nominal_speed_mps=16.93559338, nominal_yaw_deg=-15.23843314,
                      use_nominal_launch=False)


def validate_expert(value):
    if not isinstance(value, dict) or set(value)-set(EXPERT_DEFAULT):
        raise ValueError("expert 包含未知字段。")
    result = dict(EXPERT_DEFAULT, **value)
    if result["law"] not in ("ballistic_pn", "classical_pn"):
        raise ValueError("expert.law 必须是ballistic_pn或classical_pn。")
    if not isinstance(result["use_nominal_launch"], bool):
        raise ValueError("expert.use_nominal_launch 必须是布尔值。")
    for key in set(EXPERT_DEFAULT)-{"law", "use_nominal_launch"}:
        number = result[key]
        if isinstance(number, bool) or not isinstance(number, (int, float)) or not math.isfinite(number):
            raise ValueError(f"expert.{key} 必须是有限数。")
        if key != "nominal_yaw_deg" and number <= 0:
            raise ValueError(f"expert.{key} 必须为正。")
        result[key] = float(number)
    return result


def validate_dynamics(value):
    if not isinstance(value, dict) or set(value)-set(DYNAMICS_DEFAULT):
        raise ValueError("reduced_order 包含未知字段。")
    result = dict(DYNAMICS_DEFAULT, **value)
    for key in ("max_control_g", "response_tau_s", "max_slew_g_s"):
        number = result[key]
        if isinstance(number, bool) or not isinstance(number, (int, float)) or not math.isfinite(number) or number <= 0:
            raise ValueError(f"reduced_order.{key} 必须是正有限数。")
        result[key] = float(number)
    if not isinstance(result["drag_enabled"], bool):
        raise ValueError("reduced_order.drag_enabled 必须是布尔值。")
    return result


def load_ideal_config(value=None):
    if value is None:
        value = IDEAL_PROFILE
    if isinstance(value, (str, Path)):
        value = json.loads(Path(value).read_text(encoding="utf-8-sig"))
    if not isinstance(value, dict):
        raise ValueError("简化导引配置必须是JSON对象。")
    value = copy.deepcopy(value)
    dynamics = validate_dynamics(value.pop("reduced_order", {}))
    expert = validate_expert(value.pop("expert", {}))
    return ideal_flight_config(value), dynamics, expert


def radial_limit(value, limit):
    value = np.asarray(value, dtype=float)
    length = math.hypot(*value)
    return value.copy() if length <= limit else value*(limit/length)


def response_after(actual, target, dt, tau, slew):
    """Exact vector first-order response with a norm-bounded slew rate.

    For a held target, error direction is constant. Its length first decreases
    linearly at `slew`, then exponentially once it reaches `slew*tau`.
    Both endpoints inside the overload disk imply the whole response stays in it.
    """
    error = np.asarray(actual)-np.asarray(target)
    distance = math.hypot(*error)
    if distance == 0. or dt == 0.:
        return np.array(actual, copy=True)
    switch_t = max(0., distance/slew-tau)
    if dt <= switch_t:
        remaining = distance-slew*dt
    else:
        remaining = min(distance, slew*tau)*math.exp(-(dt-switch_t)/tau)
    return np.asarray(target)+error*(remaining/distance)


def ideal_flight_config(value=None):
    """Use the normal config schema, with unused realization and noise disabled."""
    if value is None:
        return load_ideal_config()[0]
    config = load_flight_config(value)
    config["controller"]["autopilot_mode"] = "guidedemo_v1"
    config["controller"]["roll_gain"] = 0.
    config["controller"]["roll_mode"] = "legacy_p"
    config["controller"]["roll_enabled"] = False
    config["imu_noise"]["enabled"] = False
    return config


def velocity_frame(velocity, fallback_yaw=0., fallback_pitch=0.):
    """Local X follows velocity; Y points left; Euler roll is exactly zero."""
    vx, vy, vz = np.asarray(velocity, dtype=float)
    horizontal = math.hypot(vx, vy)
    speed = math.hypot(horizontal, vz)
    yaw = math.atan2(vy, vx) if horizontal > 1e-10 else fallback_yaw
    pitch = math.atan2(vz, horizontal) if speed > 1e-10 else fallback_pitch
    cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    rotation = np.array([[cp*cy, -sy, -sp*cy],
                         [cp*sy, cy, -sp*sy], [sp, 0., cp]])
    return rotation, yaw, pitch


def velocity_body_rate(velocity, acceleration, yaw, pitch):
    """Analytic body angular velocity for Rz(yaw) Ry(-pitch), rad/s."""
    vx, vy, vz = np.asarray(velocity, dtype=float)
    ax, ay, az = np.asarray(acceleration, dtype=float)
    horizontal2 = vx*vx + vy*vy
    speed2 = horizontal2 + vz*vz
    if horizontal2 <= 1e-18 or speed2 <= 1e-18:
        return np.zeros(3)
    horizontal = math.sqrt(horizontal2)
    yaw_dot = (vx*ay-vy*ax)/horizontal2
    horizontal_dot = (vx*ax+vy*ay)/horizontal
    pitch_dot = (horizontal*az-vz*horizontal_dot)/speed2
    return np.array([yaw_dot*math.sin(pitch), -pitch_dot, yaw_dot*math.cos(pitch)])


class IdealController(FlightController):
    """Execute the shared PNG/LOS clock, bypassing the entire AP realization."""

    def _autopilot_command(self, axis, imu):
        return 0.

    def observation(self):
        result = super().observation()
        result["autopilot_mode"] = "ideal"
        return result


class IdealSimulation(DemoSimulation):
    engine_label = "Bounded-response RK4 point mass; MuJoCo rendering only"

    def __init__(self, config, scene_manifest=None, dart_manifest=None, launch_profile=None,
                 *, guidance_source="vision", target_mode="fixed", target_seed=2027,
                 aero_model=None, flight_control_enabled=True, flight_control_config=None,
                 dynamics_config=None, expert_config=None, position_offset_W=None,
                 aim_reference_config=None):
        if guidance_source not in ("vision", "truth"):
            raise ValueError("guidance_source must be vision or truth")
        self.guidance_source = guidance_source
        self._ideal_yaw = math.radians(config.get("yaw_deg", 0.))
        self._ideal_pitch = math.radians(config.get("pitch_deg", 0.))
        self._ideal_acceleration_W = GRAVITY_W.copy()
        flight, dynamics, expert = load_ideal_config(flight_control_config)
        self.dynamics_config = validate_dynamics(dynamics if dynamics_config is None else dynamics_config)
        self.expert_config = validate_expert(expert if expert_config is None else expert_config)
        if isinstance(aim_reference_config, (str, Path)):
            aim_reference_config = json.loads(Path(aim_reference_config).read_text(encoding="utf-8-sig"))
        self._aim_reference_config = copy.deepcopy(aim_reference_config)
        self._aim_reference = None
        self._launch_offset_W = np.array([0., 0., 0.] if position_offset_W is None else position_offset_W, dtype=float)
        if self._launch_offset_W.shape != (3,) or not np.isfinite(self._launch_offset_W).all():
            raise ValueError("position_offset_W必须是三个有限数。")
        self._initializing_ideal = True
        self._pn_command_g = np.zeros(2)
        self._pn_expert = None
        self._target_estimate_W = None
        self._target_estimate_t = None
        self._expert_diagnostics = dict(status="awaiting_visual")
        self._ideal_actual_g = np.zeros(2)
        super().__init__(config, scene_manifest, dart_manifest, launch_profile,
            target_mode=target_mode, target_seed=target_seed, aero_model="none",
            flight_control_enabled=flight_control_enabled,
            flight_control_config=flight)
        self._initializing_ideal = False

    @property
    def simulation_factory(self):
        return partial(type(self), guidance_source=self.guidance_source,
                       dynamics_config=copy.deepcopy(self.dynamics_config),
                       expert_config=copy.deepcopy(self.expert_config),
                       position_offset_W=self._launch_offset_W.copy(),
                       aim_reference_config=copy.deepcopy(self._aim_reference_config))

    def reset(self, config=None, **kwargs):
        self.flight_config = ideal_flight_config(self.flight_config)
        self._ideal_actual_g = np.zeros(2)
        self._pn_command_g = np.zeros(2)
        self._target_estimate_W = None
        self._target_estimate_t = None
        self._expert_diagnostics = dict(status="awaiting_visual")
        if self._initializing_ideal and config is not None and self.expert_config["use_nominal_launch"]:
            config = dict(config, speed_mps=self.expert_config["nominal_speed_mps"],
                          yaw_deg=self.expert_config["nominal_yaw_deg"])
        if self.launch_profile is not None:
            self.launch_profile = replace(self.launch_profile,
                target_collision=replace(self.launch_profile.target_collision, enabled=False))
        super().reset(config, **kwargs)
        self.data.qpos[:3] += self._launch_offset_W
        self.flight_controller = IdealController(self.flight_config)
        # Kept for the shared viewer's config/capture protocol; never advanced.
        self.imu_noise = ImuNoise(self.flight_config)
        self.model.geom_contype[:] = 0
        self.model.geom_conaffinity[:] = 0
        self._ideal_yaw = math.radians(self.cfg["yaw_deg"])
        self._ideal_pitch = math.radians(self.cfg["pitch_deg"])
        self._last_truth_t = None
        self._ideal_max_tracking_error_g = 0.
        self._ideal_max_alignment_error = 0.
        self._setup_expert()
        self._refresh_kinematics()
        self.trajectory = [self.data.qpos[:3].copy()]
        return self.observation()

    def _setup_expert(self):
        self._pn_expert = None
        self._aim_reference = None
        self._observer_calibration = None
        if self.launch_profile is None:
            return
        profile = self.launch_profile
        if profile.guide_light is not None:
            lamp = np.array(profile.guide_light.center_W_m)
            self._observer_calibration = dict(lamp_plane_x_m=float(lamp[0]),
                lamp_to_target_W_m=(np.array(profile.target_position_W_m)-lamp).tolist(),
                source="static launch-profile CAD calibration; no dynamic target state")
        if self.expert_config["law"] == "ballistic_pn":
            # Lazy loading keeps the traditional validation independent of v2.
            if str(PROJECT) not in sys.path:
                sys.path.insert(0, str(PROJECT))
            from pn_expert_v2.expert import BallisticPNExpert
            config = dict(self.dynamics_config, **{key: self.expert_config[key]
                for key in ("update_period_s", "prediction_dt_s", "png_N")},
                vision_timeout_s=self.flight_config["controller"]["vision_timeout_s"])
            nose = self.dart_model.nose_tip if self.dart_model is not None else [.215, 0., 0.]
            self._pn_expert = BallisticPNExpert(config, mass_kg=self.cfg["mass_kg"],
                rho_kg_m3=self.cfg["rho_kg_m3"], area_m2=self.cfg["reference_area_m2"],
                drag_coefficient=self.cfg["drag_coefficient"], gravity_W=GRAVITY_W,
                plane_normal_W=profile.target_normal_W, nose_offset_L=nose)
            if self._aim_reference_config is not None:
                from learning.aim_reference import AimReference
                self._aim_reference = AimReference(self._aim_reference_config,
                    plane_normal_W=profile.target_normal_W,
                    fovy_deg=self.cfg["camera_fov_deg"])
                if not np.array_equal(self._aim_reference.image_size_px, [320, 240]):
                    raise ValueError("当前渲染和视觉链路固定320×240，参考标定尺寸必须一致。")

    @property
    def guidance_goal_offset_W_m(self):
        return np.zeros(3) if self._aim_reference is None else self._aim_reference.offset_W_m.copy()

    @property
    def guidance_target_estimate_W_m(self):
        return None if self._target_estimate_W is None else self._target_estimate_W+self.guidance_goal_offset_W_m

    @property
    def aim_reference_observation(self):
        if self._aim_reference is None:
            return dict(mode="display_only", active=False, offset_W_m=[0., 0., 0.],
                explanation="本版本红十字仅作显示；PN专家按CAD上方靶心制导。")
        radius = self.launch_profile.target_radius_m
        result = self._aim_reference.observation(target_radius_m=radius)
        result["within_original_target_radius"] = bool(np.linalg.norm(self.guidance_goal_offset_W_m) <= radius)
        return result

    def set_aim_reference_mode(self, mode):
        if self._aim_reference is None:
            raise ValueError("当前入口未启用学习分区的参考接口。")
        from learning.aim_reference import AimReference
        config = dict(self._aim_reference_config, mode=mode)
        candidate = AimReference(config, plane_normal_W=self.launch_profile.target_normal_W,
                                 fovy_deg=self.cfg["camera_fov_deg"])
        self._aim_reference_config = copy.deepcopy(candidate.config)

    def restart_episode(self, baseline, numeric, *, position_offset_W=(0., 0., 0.)):
        """Reuse the compiled scene for new launch/wind episodes only.

        The caller must supply a zero-time snapshot and cannot change geometry,
        camera or physical constants without a full reset. No renderer or target
        evaluation is replaced, and no random seed is passed to the controller.
        """
        offset = np.asarray(position_offset_W, dtype=float)
        if offset.shape != (3,) or not np.isfinite(offset).all():
            raise ValueError("position_offset_W必须是三个有限数。")
        candidate = validate_config(numeric)
        original = baseline.get("numeric_config")
        allowed = {"speed_mps", "yaw_deg", "pitch_deg", "launch_height_m", "wind_cross_mps"}
        if (original is None or baseline.get("time_s") != 0. or
                any(candidate[key] != original[key] for key in candidate if key not in allowed)):
            raise ValueError("快速回合重置只接受零时刻快照及发射/风参数变更。")
        self.restore_state(baseline)
        self.cfg = candidate
        self._launch_offset_W = offset.copy()
        self.data.qpos[:3] += offset-np.asarray(baseline["launch_offset_W"])
        self.data.qpos[2] += candidate["launch_height_m"]-original["launch_height_m"]
        yaw, pitch = np.radians([candidate["yaw_deg"], candidate["pitch_deg"]])
        self.data.qvel[:3] = candidate["speed_mps"]*np.array(
            [math.cos(yaw)*math.cos(pitch), math.sin(yaw)*math.cos(pitch), math.sin(pitch)])
        self._ideal_yaw, self._ideal_pitch = yaw, pitch
        self._flight_yaw_reference_deg = candidate["yaw_deg"]
        self._setup_expert()
        self._refresh_kinematics()
        self.trajectory = [self.data.qpos[:3].copy()]
        return self.observation()

    def apply_flight_config(self, config_or_path):
        if isinstance(config_or_path, dict):
            patched = copy.deepcopy(self.flight_config)
            patched["reduced_order"] = copy.deepcopy(self.dynamics_config)
            patched["expert"] = copy.deepcopy(self.expert_config)
            for section, values in config_or_path.items():
                if isinstance(values, dict) and isinstance(patched.get(section), dict):
                    patched[section].update(copy.deepcopy(values))
                else:
                    patched[section] = copy.deepcopy(values)
        else:
            patched = config_or_path
        flight, dynamics, expert = load_ideal_config(patched)
        candidate = copy.copy(self)
        candidate.flight_config, candidate.dynamics_config, candidate.expert_config = flight, dynamics, expert
        candidate._external_renderer = candidate._onboard_renderer = candidate._graphics_renderer = None
        try:
            observation = candidate.reset()
        except Exception:
            candidate.close()
            raise
        self.close()
        self.__dict__ = candidate.__dict__
        return observation

    def set_imu_noise(self, enabled, scale):
        if enabled:
            raise ValueError("理想导引实验使用完美自状态，IMU噪声固定关闭；视觉仍为实际图像识别。")
        return super().set_imu_noise(False, scale)

    def _command_g(self):
        if not self.flight_control_enabled:
            return np.zeros(2)
        if self.expert_config["law"] == "ballistic_pn":
            return self._pn_command_g.copy()
        control = self.flight_controller.observation()
        return np.array([control["a_cmd_pitch_g"], control["a_cmd_yaw_g"]])

    def _limited_command_g(self):
        return radial_limit(self._command_g(), self.dynamics_config["max_control_g"])

    def _drag_acceleration(self, velocity):
        if not self.dynamics_config["drag_enabled"]:
            return np.zeros(3)
        relative = np.asarray(velocity)-np.array([0., self.cfg["wind_cross_mps"], 0.])
        speed = np.linalg.norm(relative)
        factor = (.5*self.cfg["rho_kg_m3"]*self.cfg["reference_area_m2"]*
                  self.cfg["drag_coefficient"]/self.cfg["mass_kg"])
        return -factor*speed*relative

    def _acceleration(self, velocity, actual):
        rotation, _, _ = velocity_frame(velocity, self._ideal_yaw, self._ideal_pitch)
        specific = rotation @ np.array([0., -GRAVITY*actual[1], -GRAVITY*actual[0]])
        return specific + self._drag_acceleration(velocity) + GRAVITY_W

    def _refresh_kinematics(self):
        velocity = self.data.qvel[:3].copy()
        rotation, yaw, pitch = velocity_frame(velocity, self._ideal_yaw, self._ideal_pitch)
        self._ideal_yaw, self._ideal_pitch = yaw, pitch
        cy, sy = math.cos(yaw/2), math.sin(yaw/2)
        cp, sp = math.cos(pitch/2), math.sin(pitch/2)
        self.data.qpos[3:7] = [cy*cp, sy*sp, -cy*sp, sy*cp]
        actual = self._ideal_actual_g
        self._ideal_control_force_W = rotation @ np.array(
            [0., -GRAVITY*actual[1], -GRAVITY*actual[0]])
        self._flight_specific_force_W = self._ideal_control_force_W + self._drag_acceleration(velocity)
        self._ideal_acceleration_W = self._flight_specific_force_W + GRAVITY_W
        self.data.qvel[3:6] = velocity_body_rate(
            velocity, self._ideal_acceleration_W, yaw, pitch)
        self._last_control_force_W = self.cfg["mass_kg"] * self._ideal_control_force_W
        self._last_control_torque_W = np.zeros(3)
        self.data.xfrc_applied[:] = 0.
        self._sync_target_motion(forward=False)
        mujoco.mj_forward(self.model, self.data)
        achieved = rotation.T @ self._flight_specific_force_W / GRAVITY
        error = float(np.max(np.abs(np.array([-achieved[2], -achieved[1]])-self._command_g())))
        self._ideal_max_tracking_error_g = max(getattr(self, "_ideal_max_tracking_error_g", 0.), error)
        speed = np.linalg.norm(velocity)
        if speed > 1e-10:
            alignment = float(np.linalg.norm(rotation[:, 0]-velocity/speed))
            self._ideal_max_alignment_error = max(getattr(self, "_ideal_max_alignment_error", 0.), alignment)

    def submit_visual_measurement(self, details):
        if self._aim_reference is not None and details.get("aim_point_px") is not None:
            self._aim_reference.update(details["aim_point_px"])
        if self.guidance_source == "truth":
            return None  # Detector output is display-only in this explicit comparison.
        measurement = super().submit_visual_measurement(details)
        bbox = details.get("bbox")
        unclipped = (bbox is not None and bbox[0] > 0 and bbox[1] > 0 and
                     bbox[0]+bbox[2] < 320 and bbox[1]+bbox[3] < 240)
        reliable = bool(details.get("locked", False) and unclipped)
        if measurement is not None:
            measurement["upper_target_measurement_accepted"] = bool(measurement["valid"] and reliable)
        if measurement and measurement["valid"] and reliable and self._observer_calibration is not None:
            camera_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "onboard")
            origin = self.data.cam_xpos[camera_id].copy()
            rotation = self.data.xmat[self.body_id].reshape(3, 3)
            ray = rotation @ np.array(measurement["los_ray_L"])
            if ray[0] > 1e-8:
                depth = (self._observer_calibration["lamp_plane_x_m"]-origin[0])/ray[0]
                if depth > 0:
                    measured = origin+depth*ray+np.array(self._observer_calibration["lamp_to_target_W_m"])
                    elapsed = (0. if self._target_estimate_t is None else self.t-self._target_estimate_t)
                    alpha = (1. if self._target_estimate_W is None else
                        1.-math.exp(-elapsed/self.expert_config["vision_filter_tau_s"]))
                    self._target_estimate_W = (measured if self._target_estimate_W is None else
                        self._target_estimate_W+alpha*(measured-self._target_estimate_W))
                    self._target_estimate_t = self.t
                    measurement["upper_target_estimate_W_m"] = self._target_estimate_W.tolist()
                    self._last_visual_measurement = copy.deepcopy(measurement)
        return measurement

    def _advance_expert(self):
        if self.expert_config["law"] != "ballistic_pn" or self._pn_expert is None:
            return
        age = math.inf if self._target_estimate_t is None else self.t-self._target_estimate_t
        if self._target_estimate_W is None:
            self._pn_command_g = np.zeros(2)
            self._expert_diagnostics = dict(status="awaiting_visual")
            return
        self._pn_command_g, self._expert_diagnostics = self._pn_expert.update(self.t,
            self.data.qpos[:3].copy(), self.data.qvel[:3].copy(), self._ideal_actual_g.copy(),
            self.guidance_target_estimate_W_m.copy(), valid_age_s=age)

    def _submit_truth_los(self):
        if self._last_truth_t is not None and self.t <= self._last_truth_t + 1e-12:
            return
        if self.launch_profile is None:
            raise ValueError("真值LOS对照需要显式目标发射方案。")
        rotation = self.data.xmat[self.body_id].reshape(3, 3).copy()
        ray = rotation.T @ (self.target_position_W()-self.data.xpos[self.body_id])
        valid = bool(ray[0] > 1e-9 and np.linalg.norm(ray) > 1e-9)
        imu = self._flight_imu()
        pitch = math.degrees(math.atan2(-ray[2], ray[0])) if valid else None
        yaw = math.degrees(math.atan2(-ray[1], ray[0])) if valid else None
        self.flight_controller.submit_visual(self.t, pitch, yaw,
            imu["pitch_rate_deg_s"], imu["yaw_rate_deg_s"],
            rotation_WL=rotation.tolist() if valid else None, los_ray_L=ray.tolist() if valid else None)
        self._last_visual_measurement = dict(t_s=self.t, valid=valid, source="truth_upper_target",
                                             los_pitch_deg=pitch, los_yaw_deg=yaw)
        if valid:
            self._target_estimate_W = self.target_position_W().copy()
            self._target_estimate_t = self.t
        self._last_truth_t = self.t

    def step(self, steps=1):
        for _ in range(steps):
            if self.done:
                break
            end = min(self.t+self.dt, self.cfg["t_max_s"])
            while not self.done and self.t < end-1e-12:
                if self.flight_control_enabled:
                    if self.flight_controller.next_update_time_s <= self.t+1e-12:
                        if self.guidance_source == "truth":
                            self._submit_truth_los()
                        self.flight_controller.advance_to(self.t, self._flight_imu())
                        self._advance_expert()
                    self._refresh_kinematics()
                dt = min(self.dt, end-self.t)
                if self.flight_control_enabled:
                    boundary = self.flight_controller.next_update_time_s-self.t
                    if boundary > 1e-12:
                        dt = min(dt, boundary)
                previous_tip, previous_time = self.nose_position_W(), self.t
                target = self._limited_command_g()
                actual_start = self._ideal_actual_g.copy()
                def actual_at(elapsed):
                    return response_after(actual_start, target, elapsed,
                        self.dynamics_config["response_tau_s"], self.dynamics_config["max_slew_g_s"])
                state = np.r_[self.data.qpos[:3].copy(), self.data.qvel[:3].copy()]
                def derivative(value, elapsed):
                    return np.r_[value[3:], self._acceleration(value[3:], actual_at(elapsed))]
                k1 = derivative(state, 0.)
                k2 = derivative(state+dt*k1/2, dt/2)
                k3 = derivative(state+dt*k2/2, dt/2)
                k4 = derivative(state+dt*k3, dt)
                state += dt*(k1+2*k2+2*k3+k4)/6
                self._ideal_actual_g = actual_at(dt)
                self.data.qpos[:3], self.data.qvel[:3] = state[:3], state[3:]
                self.data.time = previous_time+dt
                self.effective_dt = dt
                self._refresh_kinematics()
                self.internal_step_count += 1
                self.status = "running"
                self._check_target_crossing(previous_tip, self.nose_position_W(), previous_time, self.t)
                if not np.isfinite(state).all():
                    self.status, self.done = "invalid", True
                elif not self.done and state[2] <= .03:
                    self.status, self.done = "landed", True
                elif not self.done and self.t >= self.cfg["t_max_s"]-1e-12:
                    self.status, self.done = "finished", True
                if self.t >= len(self.trajectory)*.02-1e-9:
                    self.trajectory.append(state[:3].copy())
            self.macro_step_count += 1
            self.effective_dt = self.dt
        return self.observation()

    def _flight_observation(self):
        control = self.flight_controller.observation()
        if self.expert_config["law"] == "ballistic_pn":
            control = dict(control, a_cmd_pitch_g=float(self._pn_command_g[0]),
                           a_cmd_yaw_g=float(self._pn_command_g[1]), autopilot_mode="ballistic_pn")
        rotation = self.data.xmat[self.body_id].reshape(3, 3)
        specific = rotation.T @ self._flight_specific_force_W / GRAVITY
        actual = dict(pitch=float(-specific[2]), yaw=float(-specific[1]))
        target = self._limited_command_g()
        heading = math.degrees(self._ideal_yaw)
        yaw_relative = (heading-self._flight_yaw_reference_deg+180.) % 360.-180.
        motion = np.zeros(6)
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY,
                                self.body_id, motion, 0)
        omega = rotation.T @ motion[:3]
        relative_velocity = self.data.qvel[:3]-np.array([0., self.cfg["wind_cross_mps"], 0.])
        return dict(enabled=self.flight_control_enabled, realization="ideal",
            attitude_deg=dict(pitch=math.degrees(self._ideal_pitch), yaw=yaw_relative, roll=0.),
            angular_rate_deg_s=dict(pitch=float(np.degrees(-omega[1])),
                                   yaw=float(np.degrees(omega[2])), roll=float(np.degrees(omega[0]))),
            attitude_reference="yaw relative to launch; roll fixed to zero, no roll autopilot",
            actual_specific_force_L_g=dict(zip(("x", "y", "z"), specific.tolist())),
            actual_time_s=self.t, axis_command_deg={}, axis_actual_deg={},
            actuator_diagnostics=dict(available=False),
            dynamic_pressure_pa=float(.5*self.cfg["rho_kg_m3"]*
                                      np.dot(relative_velocity, relative_velocity)),
            command_limit_kind="combined_norm", command_limit_g=float(self.dynamics_config["max_control_g"]),
            guidance_source=self.guidance_source, control=control, actual_overload_g=actual,
            limited_command_g=dict(pitch=float(target[0]), yaw=float(target[1])),
            realized_control_g=dict(pitch=float(self._ideal_actual_g[0]), yaw=float(self._ideal_actual_g[1])),
            realized_control_norm_g=float(np.linalg.norm(self._ideal_actual_g)),
            response_model=copy.deepcopy(self.dynamics_config),
            expert=copy.deepcopy(self._expert_diagnostics), guidance_law=self.expert_config["law"],
            nominal_target_estimate_W_m=None if self._target_estimate_W is None else self._target_estimate_W.tolist(),
            target_estimate_W_m=None if self.guidance_target_estimate_W_m is None else self.guidance_target_estimate_W_m.tolist(),
            aim_reference=self.aim_reference_observation,
            guidance_goal_offset_W_m=self.guidance_goal_offset_W_m.tolist(),
            yaw_attitude_relative_deg=yaw_relative, yaw_trajectory_relative_deg=yaw_relative,
            status=control["tracking_status"] if self.flight_control_enabled else "disabled",
            overload_time_s=self.t, imu_noise=self.imu_noise.observation(),
            visual_measurement=copy.deepcopy(self._last_visual_measurement),
            guidance_goal_result=self.evaluate_guidance_goal(),
            specific_force_W_mps2=self._flight_specific_force_W.tolist(),
            inertial_acceleration_W_mps2=self._ideal_acceleration_W.tolist(),
            gravity_W_mps2=GRAVITY_W.tolist(),
            max_tracking_error_g=getattr(self, "_ideal_max_tracking_error_g", 0.),
            max_velocity_alignment_error=getattr(self, "_ideal_max_alignment_error", 0.),
            overload_convention="specific force, not inertial acceleration; pitch down/yaw right positive",
            attitude_realization="forward follows velocity; zero Euler roll; no independent attitude lag",
            servo_model_used=False, autopilot_used=False,
            aerodynamic_loads_used=self.dynamics_config["drag_enabled"])

    def evaluate_guidance_goal(self):
        """Ground-truth scoring only; never consumed by the expert or features."""
        if self.target_result is None:
            return None
        result = self.target_result
        goal = np.asarray(result["target_position_W_m"])+self.guidance_goal_offset_W_m
        delta = np.asarray(result["position_W_m"])-goal
        normal = np.asarray(result["normal_W"])
        error = delta-np.dot(delta, normal)*normal
        return dict(selected_goal_W_m=goal.tolist(), selected_goal_miss_m=float(np.linalg.norm(error)),
            physical_target_miss_m=result["miss_distance_m"], physical_target_hit=result["hit"],
            goal_offset_W_m=self.guidance_goal_offset_W_m.tolist(),
            selected_goal_within_original_radius=bool(np.linalg.norm(self.guidance_goal_offset_W_m) <= result["radius_m"]))

    @property
    def aero_metadata(self):
        return dict(model="quadratic_drag" if self.dynamics_config["drag_enabled"] else "none",
                    parameters={key: self.cfg[key] for key in ("drag_coefficient", "rho_kg_m3",
                        "reference_area_m2", "mass_kg", "wind_cross_mps")},
                    role="passive drag only; no control-surface mapping or passive normal-force model",
                    gravity_included=True, integrator="RK4", rendering="MuJoCo mj_forward only")

    @property
    def flight_control_metadata(self):
        return dict(model=self.expert_config["law"]+" with bounded finite-response specific force", realization="ideal",
            guidance_source=self.guidance_source, config=copy.deepcopy(self.flight_config),
            reduced_order=copy.deepcopy(self.dynamics_config),
            expert_config=copy.deepcopy(self.expert_config),
            aim_reference_config=copy.deepcopy(self._aim_reference_config),
            visual_calibration=copy.deepcopy(getattr(self, "_observer_calibration", None)),
            parameter_validation="uncalibrated engineering assumptions, not measured flight capabilities",
            gravity_W_mps2=GRAVITY_W.tolist(),
            command_convention="non-gravity specific force in g; positive down/right",
            dynamics="p_dot=v; v_dot=R(v)[0,-g*n_yaw,-g*n_pitch]+drag+gravity; n_dot=radial_clip((radial_clip(n_PNG,cap)-n)/tau,slew)",
            gravity_compensation="none; zero control is ballistic; gravity added exactly once",
            target_truth_enters_guidance=self.guidance_source == "truth",
            target_truth_policy="vision: detector ray plus own camera pose and declared static plane/rigid offset; no dynamic target truth. truth: explicit target position comparison",
            autopilot="bypassed; replaced by finite overload response", servo="bypassed",
            aerodynamics="quadratic passive drag optional; control-surface derivatives bypassed",
            collision="disabled; first geometric target-plane crossing only",
            unused_parameters=["k_rate", "k_alpha", "k_int", "servo", "actuation",
                               "normal_coefficient", "stability_coefficient", "angular_damping_coefficient"])

    @property
    def launch_metadata(self):
        result = super().launch_metadata
        result["integration"] = dict(method="RK4 point mass; exact held-command force response; MuJoCo rendering only",
                                     macro_dt_s=self.dt, control_tick_split=True)
        result["evaluation"] = "geometric nose crossing only; no impact, bounce or official IR judging"
        result["point_mass_launch_offset_W_m"] = self._launch_offset_W.tolist()
        result["expert_launch_calibration"] = {key: self.expert_config[key] for key in
            ("nominal_speed_mps", "nominal_yaw_deg")}
        return result

    def capture_state(self):
        spec = mujoco.mjtState.mjSTATE_INTEGRATION
        integration = np.empty(mujoco.mj_stateSize(self.model, spec))
        mujoco.mj_getState(self.model, self.data, integration, spec)
        return dict(ideal_snapshot_version=4, guidance_source=self.guidance_source,
            numeric_config=copy.deepcopy(self.cfg), time_s=self.t,
            launch_offset_W=self._launch_offset_W.copy(),
            aim_reference_config=copy.deepcopy(self._aim_reference_config),
            aim_reference_state=None if self._aim_reference is None else self._aim_reference.capture_state(),
            expert_config=copy.deepcopy(self.expert_config),
            expert_state=None if self._pn_expert is None else self._pn_expert.capture_state(),
            pn_command_g=self._pn_command_g.copy(), expert_diagnostics=copy.deepcopy(self._expert_diagnostics),
            target_estimate_W=None if self._target_estimate_W is None else self._target_estimate_W.copy(),
            target_estimate_t=self._target_estimate_t,
            dynamics_config=copy.deepcopy(self.dynamics_config), actual_control_g=self._ideal_actual_g.copy(),
            integration=integration, done=self.done, status=self.status,
            target_result=copy.deepcopy(self.target_result), target_contact=None,
            target_motion=self.target_motion.capture_state(), trajectory=np.array(self.trajectory).copy(),
            enabled=self.flight_control_enabled, controller=self.flight_controller.capture_state(),
            noise=self.imu_noise.capture_state(), visual=copy.deepcopy(self._last_visual_measurement),
            specific_force_W=self._flight_specific_force_W.copy(),
            acceleration_W=self._ideal_acceleration_W.copy(), yaw=self._ideal_yaw, pitch=self._ideal_pitch,
            last_truth_t=self._last_truth_t, yaw_reference_deg=self._flight_yaw_reference_deg,
            max_tracking_error_g=self._ideal_max_tracking_error_g,
            max_alignment_error=self._ideal_max_alignment_error,
            internal_step_count=self.internal_step_count, macro_step_count=self.macro_step_count)

    def restore_state(self, state):
        if state.get("ideal_snapshot_version") != 4 or state.get("guidance_source") != self.guidance_source:
            raise ValueError("回放状态不属于当前理想导引实验。")
        if state.get("aim_reference_config") != self._aim_reference_config:
            raise ValueError("回放目标参考配置与当前实验不一致。")
        if state.get("expert_config") != self.expert_config:
            raise ValueError("回放专家参数与当前实验不一致。")
        if state.get("dynamics_config") != self.dynamics_config:
            raise ValueError("回放过载响应参数与当前实验不一致。")
        actual_control = np.asarray(state.get("actual_control_g"), dtype=float)
        if actual_control.shape != (2,) or not np.isfinite(actual_control).all() or np.linalg.norm(actual_control) > self.dynamics_config["max_control_g"]+1e-12:
            raise ValueError("回放实际控制过载无效。")
        if state["enabled"] != self.flight_control_enabled:
            raise ValueError("回放导引开关与当前实验不一致。")
        spec = mujoco.mjtState.mjSTATE_INTEGRATION
        integration = np.asarray(state["integration"], dtype=float)
        if integration.shape != (mujoco.mj_stateSize(self.model, spec),):
            raise ValueError("回放模型不匹配。")
        controller = IdealController(self.flight_config)
        controller.restore_state(state["controller"])
        noise = ImuNoise(self.flight_config)
        noise.restore_state(state["noise"])
        self.target_motion.restore_state(state["target_motion"])
        mujoco.mj_setState(self.model, self.data, integration, spec)
        self.cfg = copy.deepcopy(state["numeric_config"])
        self._launch_offset_W = np.array(state["launch_offset_W"], copy=True)
        if self._aim_reference is not None:
            self._aim_reference.restore_state(state["aim_reference_state"])
        self.flight_controller, self.imu_noise = controller, noise
        self.done, self.status = bool(state["done"]), state["status"]
        self.target_result, self.target_contact = copy.deepcopy(state["target_result"]), None
        self.trajectory = [row.copy() for row in state["trajectory"]]
        self._last_visual_measurement = copy.deepcopy(state["visual"])
        self._flight_specific_force_W = np.array(state["specific_force_W"], copy=True)
        self._ideal_acceleration_W = np.array(state["acceleration_W"], copy=True)
        self._last_control_force_W = self.cfg["mass_kg"]*self._flight_specific_force_W
        self._last_control_torque_W = np.zeros(3)
        self._ideal_yaw, self._ideal_pitch = state["yaw"], state["pitch"]
        self._ideal_actual_g = actual_control.copy()
        self._pn_command_g = np.array(state["pn_command_g"], dtype=float, copy=True)
        self._expert_diagnostics = copy.deepcopy(state["expert_diagnostics"])
        self._target_estimate_W = None if state["target_estimate_W"] is None else np.array(state["target_estimate_W"], copy=True)
        self._target_estimate_t = state["target_estimate_t"]
        if self._pn_expert is not None:
            self._pn_expert.restore_state(state["expert_state"])
        self._ideal_control_force_W = velocity_frame(self.data.qvel[:3], self._ideal_yaw,
            self._ideal_pitch)[0] @ np.array([0., -GRAVITY*actual_control[1], -GRAVITY*actual_control[0]])
        self._last_control_force_W = self.cfg["mass_kg"]*self._ideal_control_force_W
        self._last_truth_t = state["last_truth_t"]
        self._flight_yaw_reference_deg = state["yaw_reference_deg"]
        self._ideal_max_tracking_error_g = state["max_tracking_error_g"]
        self._ideal_max_alignment_error = state["max_alignment_error"]
        self.internal_step_count, self.macro_step_count = state["internal_step_count"], state["macro_step_count"]
        self.effective_dt = self.dt
        self._sync_target_motion(forward=False)
        mujoco.mj_forward(self.model, self.data)


class IdealViewer(DartViewer):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        predictive = self.sim.expert_config["law"] == "ballistic_pn"
        version = "PN参考微调v3" if self.sim._aim_reference is not None else "PN专家v2"
        self.root.title((version+" · 弹道预测" if predictive else "传统PN对照 · 有限响应") +
            " · " + ("真实视觉" if self.sim.guidance_source == "vision" else "真值LOS对照"))
        self.message.set("弹道预测PN：真实绿灯识别＋测绘平面/刚性偏置→上方靶面估计；响应参数待标定。"
            if predictive else "传统PN对照：保留原N10和原发射预设；过载限幅/滞后，重力与阻力开启。")
        if self.sim._aim_reference is not None:
            reference = self.sim.aim_reference_observation
            scale = reference["reference_distance_m"]/reference["focal_px"]*1000.
            self.message.set(f"v3红十字：标定零点{reference['calibrated_aim_point_px']}；下移→目标向上，右移→目标向左。"
                f"当前换算{scale:.3f}mm/像素，改完点击应用；物理靶面判定保留。")

    def _open_flight_tuning(self):
        import tkinter as tk
        from tkinter import messagebox, ttk
        self._show_settings("flight")
        if self._flight_tuning_panel is not None:
            return
        for child in self.flight_tuning_host.winfo_children():
            child.destroy()
        window = ttk.Frame(self.flight_tuning_host)
        window.pack(fill="both", expand=True)
        panel = ttk.Frame(window, padding=14)
        panel.pack(fill="both", expand=True)
        ttk.Label(panel, text="弹道预测PN（ZEM修正），不是传统纯视线差分PN。\n"
                  "导引来源：" + ("真实相机识别" if self.sim.guidance_source == "vision" else "显式上方靶心真值LOS对照")).grid(row=0, column=0, columnspan=2, pady=(0, 12))
        fields = [("png_N", "PNG比例N"), ("closing_speed_mps", "接近速度 m/s"),
                  ("max_accel_g", "PNG单轴请求限幅 g"), ("vision_timeout_s", "视觉超时 s"),
                  ("max_control_g", "合成控制过载上限 g"), ("response_tau_s", "响应时间常数 s"),
                  ("max_slew_g_s", "合成过载变化率 g/s")]
        predictive = self.sim.expert_config["law"] == "ballistic_pn"
        if predictive:
            fields = [("png_N", "弹道PN比例N"), ("vision_filter_tau_s", "目标测量滤波 s")]+fields[3:]
        values = {}
        for i, (key, label) in enumerate(fields, 1):
            ttk.Label(panel, text=label).grid(row=i, column=0, sticky="w", padx=5, pady=5)
            source = (self.sim.dynamics_config if key in DYNAMICS_DEFAULT else
                self.sim.expert_config if predictive and key in ("png_N", "vision_filter_tau_s") else
                self.sim.flight_config["controller"])
            values[key] = tk.StringVar(value=str(source[key]))
            ttk.Entry(panel, textvariable=values[key], width=14).grid(row=i, column=1, padx=5, pady=5)
        drag = tk.BooleanVar(value=self.sim.dynamics_config["drag_enabled"])
        ttk.Checkbutton(panel, text="保留当前Cd的被动阻力（重力始终开启）", variable=drag).grid(
            row=len(fields)+1, column=0, columnspan=2, pady=5)
        def close():
            if window.winfo_exists():
                window.destroy()
        def apply():
            try:
                changed = {key: float(value.get()) for key, value in values.items()}
                expert_keys = ("png_N", "vision_filter_tau_s") if predictive else ()
                self.sim.apply_flight_config({
                    "expert": {key: value for key, value in changed.items() if key in expert_keys},
                    "controller": {key: value for key, value in changed.items() if key not in DYNAMICS_DEFAULT and key not in expert_keys},
                    "reduced_order": dict({key: value for key, value in changed.items() if key in DYNAMICS_DEFAULT},
                                          drag_enabled=drag.get())})
            except (ValueError, OSError) as exc:
                messagebox.showerror("参数未应用", str(exc), parent=window)
                return
            self._new_recording()
            self.message.set("PNG与有限过载响应参数已应用；重力开启，舵偏映射跳过。")
        ttk.Button(panel, text="应用并重置", command=apply).grid(row=len(fields)+2, column=0, columnspan=2, pady=12)
        self._flight_tuning_panel = type("IdealPanel", (), {"close": staticmethod(close)})()


def run_headless(sim, duration, output, settings):
    output.mkdir(parents=True, exist_ok=True)
    tracker = OpenMVTracker(settings) if sim.guidance_source == "vision" else None
    clock = SampleClock(90.)
    clock.reset(sim.t)
    vision_count = detected = 0
    trace = []
    max_requested_g = max_realized_g = 0.
    if tracker is not None:
        details = sample_flight_vision(sim, tracker)
        vision_count, detected = 1, int(details["detected"])
    while sim.t < duration-1e-12 and not sim.done:
        sim.step()
        if tracker is not None and clock.consume(sim.t):
            details = sample_flight_vision(sim, tracker)
            vision_count += 1
            detected += int(details["detected"])
        obs = sim.observation()
        control = obs["flight_control"]["control"]
        max_requested_g = max(max_requested_g, math.hypot(control["a_cmd_pitch_g"], control["a_cmd_yaw_g"]))
        max_realized_g = max(max_realized_g, obs["flight_control"]["realized_control_norm_g"])
        trace.append(dict(t=sim.t, p_W=obs["p_W"].tolist(), v_W=obs["v_W"].tolist(),
                          flight_control=obs["flight_control"]))
    obs = sim.observation()
    result = dict(engine=sim.engine_label, model=sim.flight_control_metadata,
        guidance_source=sim.guidance_source, duration_s=sim.t, status=sim.status,
        target_result=sim.target_result, target_contact=None,
        vision_config=settings.as_dict(), vision_samples=vision_count, detected_frames=detected,
        numeric_config=sim.cfg, flight_config=sim.flight_config,
        final_position_W_m=obs["p_W"].tolist(), final_velocity_W_mps=obs["v_W"].tolist(),
        final_flight_control=obs["flight_control"])
    result["max_requested_norm_g"] = max_requested_g
    result["max_realized_control_norm_g"] = max_realized_g
    (output/"result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    (output/"trace.json").write_text(json.dumps(trace, ensure_ascii=False)+"\n", encoding="utf-8")
    print(json.dumps(dict(source=sim.guidance_source, t_s=sim.t, status=sim.status,
                         target=sim.target_result,
                         max_requested_norm_g=max_requested_g,
                         max_realized_control_norm_g=max_realized_g), ensure_ascii=False))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--duration", type=float, default=2.6)
    parser.add_argument("--output", type=Path, default=PROJECT/"outputs/ideal_guidance")
    parser.add_argument("--guidance-source", choices=("vision", "truth"), default="vision")
    parser.add_argument("--guidance-law", choices=("ballistic_pn", "classical_pn"))
    parser.add_argument("--aim-reference-config", type=Path)
    parser.add_argument("--flight-control", choices=("on", "off"), default="on")
    parser.add_argument("--no-drag", action="store_true", help="Gravity-only ballistic reference when guidance is off")
    parser.add_argument("--yaw-offset-deg", type=float, default=0.)
    parser.add_argument("--pitch-offset-deg", type=float, default=0.)
    parser.add_argument("--speed-offset-mps", type=float, default=0.)
    parser.add_argument("--position-offset-m", type=float, nargs=3, default=[0., 0., 0.])
    parser.add_argument("--wind-cross-mps", type=float)
    parser.add_argument("--flight-config", type=Path, default=IDEAL_PROFILE)
    parser.add_argument("--vision-config", type=Path, default=PROJECT/"profiles/vision_desktop.json")
    parser.add_argument("--scene", type=Path)
    parser.add_argument("--launch", type=Path)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--dart", type=Path)
    parser.add_argument("--auto-run", action="store_true")
    parser.add_argument("--auto-close", type=float)
    parser.add_argument("--target-mode", choices=("fixed", "random_fixed", "random_moving", "end_moving"), default="fixed")
    parser.add_argument("--target-seed", type=int, default=2027)
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error("--duration must be positive")
    if not math.isfinite(args.yaw_offset_deg):
        parser.error("--yaw-offset-deg must be finite")
    if not all(math.isfinite(v) for v in [args.pitch_offset_deg, args.speed_offset_mps, *args.position_offset_m]):
        parser.error("launch offsets must be finite")
    if args.wind_cross_mps is not None and not math.isfinite(args.wind_cross_mps):
        parser.error("--wind-cross-mps must be finite")
    if args.auto_close is not None and (not math.isfinite(args.auto_close) or args.auto_close <= 0):
        parser.error("--auto-close must be positive")
    inputs = resolve_startup_paths(args.config, args.scene, args.dart, args.launch)
    settings = VisionSettings(**json.loads(args.vision_config.read_text(encoding="utf-8-sig")))
    expert_config = load_ideal_config(args.flight_config)[2]
    if args.guidance_law is not None:
        expert_config["law"] = args.guidance_law
    sim = IdealSimulation(load_config(inputs.config), inputs.scene, inputs.dart, inputs.launch,
        guidance_source=args.guidance_source, target_mode=args.target_mode, target_seed=args.target_seed,
        flight_control_enabled=args.flight_control == "on",
        flight_control_config=args.flight_config, expert_config=expert_config,
        position_offset_W=args.position_offset_m, aim_reference_config=args.aim_reference_config)
    try:
        if args.no_drag:
            sim.dynamics_config["drag_enabled"] = False
        numeric = dict(sim.cfg, **inputs.explicit_values)
        numeric["yaw_deg"] += args.yaw_offset_deg
        numeric["pitch_deg"] += args.pitch_offset_deg
        numeric["speed_mps"] += args.speed_offset_mps
        if args.wind_cross_mps is not None:
            numeric["wind_cross_mps"] = args.wind_cross_mps
        numeric["t_max_s"] = max(numeric["t_max_s"], args.duration)
        sim.reset(numeric)
        if args.headless:
            run_headless(sim, args.duration, args.output, settings)
        else:
            import tkinter as tk
            root = tk.Tk()
            viewer = IdealViewer(root, sim, auto_run=args.auto_run, vision_settings=settings)
            if args.auto_close is not None:
                root.after(round(args.auto_close*1000), viewer.close)
            try:
                root.mainloop()
            finally:
                viewer.close()
    finally:
        sim.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
