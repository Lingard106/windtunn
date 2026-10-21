"""Guidedemo active PNG/v1 formulas with a deterministic simulation clock.

Inputs are timestamped visual angles and seven legacy IMU channels and optional body +X roll rate.
No target position or other target truth is accepted. In guidedemo_v1 mode the
firmware's pseudo-angle formula is retained; conditional integration corrects
the original reversed anti-windup test.
Red-reference aiming, configurable signal signs, timeout/neutral behavior and
the optional acceleration PID are separately identified simulation adapters.
"""

from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path
from collections.abc import Mapping

import numpy as np


PROJECT = Path(__file__).resolve().parents[1]
DEFAULT_FLIGHT_CONFIG = PROJECT / "profiles/flight_control_default.json"
SOURCE_MANIFEST = PROJECT / "reference/guidedemo/source_manifest.json"
IMU_KEYS = frozenset(("pitch_deg", "yaw_deg", "roll_deg", "pitch_rate_deg_s",
                      "yaw_rate_deg_s", "a_pitch_g", "a_yaw_g"))
_G = 9.80665
_DEG2RAD = 0.017453292519943295
_LOS_MIN_DT_S = .001  # Original guidanceTask.cpp guard, not a filter.
SUPPORTED_TAIL_LAYOUT = {
    "model": "x_tail_four_orthogonal",
    "body_axes": ["+X_forward", "+Y_left", "+Z_up"],
    "azimuth_reference": "+Y_toward_+Z",
    "azimuth_deg": [135., 45., 225., 315.],
    "channel_positions": ["upper_right", "upper_left", "lower_right", "lower_left"],
    "numbering": "simulation_channels_0_to_3",
    "pwm_wiring": "unverified",
    "axial_interpretation": "azimuth_only_no_axial_rows_inferred",
}
SOURCE_MIXING_MATRIX = ((1, 1, 1), (1, -1, -1), (-1, 1, -1), (-1, -1, 1))

LIMITATIONS = (
    "guidedemo_v1 mode ports the selected PNG and Autopilot v1; acceleration_pid is a separate simulation adapter, not original firmware.",
    "The v1 pseudo-angle formula is preserved; v2 claims a derivative-order correction, not validated here.",
    "The original reversed anti-windup condition is corrected for the negative integral action.",
    "Visual rays with attitude use full 3-D frame transport; legacy angle-only inputs retain the two-axis gyro approximation.",
    "Fixed closing speed, controller gains, sensor signs and mixer require physical calibration.",
    "X-tail geometry is user-confirmed; channel numbering and positive deflections are simulation conventions, with actual PWM wiring unverified.",
    "Firmware front/rear labels are not interpreted as two axial fin rows; only the specified X-tail azimuths are modeled.",
    "No LOS low-pass filter is added. Invalid frames hold the previous rate only until vision_timeout_s.",
    "Timeout neutralization and resetting visual/AP history are simulation adapters; firmware holds indefinitely.",
    "IMU inputs are explicit ideal semantic channels, not the firmware's uncalibrated BMI088 array indices.",
    "Servo response and aggregate control aerodynamics are estimates, not measured CAD control derivatives.",
    "Python double-precision arithmetic preserves formula/update order, not bit-exact MCU float32 arithmetic.",
    "Red-reference aiming correction, axis switches/signs and optional acceleration PID are uncalibrated simulation adapters.",
)

AXES = ("pitch", "yaw")
AXIS_OVERRIDE_FIELDS = ("png_N", "k_rate", "k_alpha", "k_int", "filter_k_alpha",
                        "filter_k_theta", "filter_b1_s", "filter_tau_s")
AXIS_OVERRIDE_KEYS = frozenset(f"{axis}_{field}" for axis in AXES for field in AXIS_OVERRIDE_FIELDS)
ROLL_FIELDS = ("roll_enabled", "roll_mode", "roll_target_deg", "roll_angle_gain_per_s",
               "roll_rate_gain_s", "roll_rate_limit_deg_s", "roll_saturation_deg")
ROLL_IMU_KEY = "roll_rate_deg_s"
ALL_IMU_KEYS = IMU_KEYS | {ROLL_IMU_KEY}


def _axis_config(controller, axis):
    """Resolve each optional axis value without changing the shared config."""
    result = dict(controller)
    for field in AXIS_OVERRIDE_FIELDS:
        value = controller[f"{axis}_{field}"]
        result[field] = controller[field] if value is None else value
    return result


def _axis_diagnostic():
    return {"unlimited_deg": 0., "command_deg": 0., "limit_deg": None,
            "saturated": False, "rate_term_deg": 0., "alpha_term_deg": 0.,
            "integral_term_deg": 0.}

SIGN_FIELDS = ("los_sign", "gyro_compensation_sign", "guidance_sign",
               "accel_feedback_sign", "rate_feedback_sign", "output_sign")


def _pid_state():
    return {"integral_g_s": 0., "previous_measured_g": None,
            "derivative_g_s": 0., "error_g": 0., "p_deg": 0., "i_deg": 0.,
            "d_deg": 0., "rate_deg": 0., "unsaturated_deg": 0., "saturated": False}


def _number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{label} must be a finite number")
    return float(value)


def _time(value, label="time_s"):
    value = _number(value, label)
    if value < 0:
        raise ValueError(f"{label} must be nonnegative")
    return value


def _read_json(path):
    try:
        return json.loads(Path(path).expanduser().read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"Cannot read flight control configuration: {exc}") from exc


def load_flight_config(path_or_mapping=None):
    """Load defaults and merge explicit section overrides; return an owned copy."""
    defaults = _read_json(DEFAULT_FLIGHT_CONFIG)
    if path_or_mapping is None:
        supplied = {}
    elif isinstance(path_or_mapping, Mapping):
        supplied = copy.deepcopy(dict(path_or_mapping))
    elif isinstance(path_or_mapping, (str, Path)):
        supplied = _read_json(path_or_mapping)
    else:
        raise ValueError("Flight configuration must be a path or mapping")
    if not isinstance(supplied, dict) or set(supplied) - set(defaults):
        raise ValueError("Flight configuration has unknown top-level fields")
    result = copy.deepcopy(defaults)
    for key, value in supplied.items():
        if key in ("controller", "servo", "actuation", "imu_noise", "tail_layout"):
            if not isinstance(value, dict) or set(value) - set(defaults[key]):
                raise ValueError(f"Invalid or unknown {key} fields")
            result[key].update(value)
        else:
            result[key] = value
    if isinstance(result["schema_version"], bool) or result["schema_version"] != 1:
        raise ValueError("Flight configuration schema_version must be 1")
    if result["model"] != "guidedemo_png_autopilot_v1":
        raise ValueError("Only guidedemo_png_autopilot_v1 is implemented")
    if not isinstance(result["provenance"], dict):
        raise ValueError("Flight configuration provenance must be an object")
    if result["tail_layout"] != SUPPORTED_TAIL_LAYOUT:
        raise ValueError("Only the fixed X-tail channel layout [135,45,225,315] deg is supported; changing metadata cannot change the mixer")
    for section in ("controller", "servo", "actuation"):
        for key, value in result[section].items():
            if section == "controller" and key in AXIS_OVERRIDE_KEYS | {"roll_saturation_deg"} and value is None:
                continue
            if section == "controller" and key == "roll_mode":
                if value not in ("legacy_p", "angle_rate"):
                    raise ValueError("controller.roll_mode must be legacy_p or angle_rate")
                continue
            if section == "controller" and key == "autopilot_mode":
                if value not in ("guidedemo_v1", "acceleration_pid"):
                    raise ValueError("controller.autopilot_mode must be guidedemo_v1 or acceleration_pid")
                continue
            if section == "controller" and key in ("pitch_enabled", "yaw_enabled", "roll_enabled"):
                if not isinstance(value, bool):
                    raise ValueError(f"controller.{key} must be boolean")
                continue
            result[section][key] = _number(value, f"{section}.{key}")
    positive = {
        "controller": ("period_s", "closing_speed_mps", "max_accel_g", "saturation_deg",
                       "filter_tau_s", "vision_timeout_s"),
        "servo": ("limit_deg", "rate_deg_s", "tau_s", "pwm_period_s", "us_per_deg"),
    }
    for section, keys in positive.items():
        for key in keys:
            if result[section][key] <= 0:
                raise ValueError(f"{section}.{key} must be positive")
    if result["controller"]["filter_k_theta"] == 0:
        raise ValueError("controller.filter_k_theta must be nonzero")
    if result["controller"]["png_N"] < 0 or result["actuation"]["drag_per_rad2"] < 0:
        raise ValueError("png_N and drag_per_rad2 must be nonnegative")
    for key, value in result["actuation"].items():
        if value < 0:
            raise ValueError(f"actuation.{key} must be nonnegative; use controller axis signs for direction")
    controller = result["controller"]
    for axis in AXES:
        for suffix in SIGN_FIELDS:
            if controller[f"{axis}_{suffix}"] not in (-1., 1.):
                raise ValueError(f"controller.{axis}_{suffix} must be +1 or -1")
        for suffix in ("kp_deg_per_g", "ki_deg_per_g_s", "kd_deg_s_per_g", "rate_gain_s"):
            if controller[f"pid_{axis}_{suffix}"] < 0:
                raise ValueError(f"controller.pid_{axis}_{suffix} must be nonnegative; use axis sign controls for direction")
        if controller[f"pid_{axis}_integral_limit_g_s"] <= 0:
            raise ValueError(f"controller.pid_{axis}_integral_limit_g_s must be positive")
        if controller[f"aim_{axis}_gain_g_per_deg"] < 0:
            raise ValueError(f"controller.aim_{axis}_gain_g_per_deg must be nonnegative")
    for axis in AXES:
        resolved = _axis_config(controller, axis)
        if resolved["png_N"] < 0:
            raise ValueError(f"controller.{axis}_png_N must be nonnegative")
        if resolved["filter_tau_s"] <= 0 or resolved["filter_k_theta"] == 0:
            raise ValueError(f"controller.{axis} filter_tau_s must be positive and filter_k_theta nonzero")
    for key in ("roll_angle_gain_per_s", "roll_rate_gain_s"):
        if controller[key] < 0:
            raise ValueError(f"controller.{key} must be nonnegative")
    if controller["roll_rate_limit_deg_s"] <= 0:
        raise ValueError("controller.roll_rate_limit_deg_s must be positive")
    if controller["roll_saturation_deg"] is not None and controller["roll_saturation_deg"] <= 0:
        raise ValueError("controller.roll_saturation_deg must be positive or null")
    if controller["aim_accel_limit_g"] < 0:
        raise ValueError("controller.aim_accel_limit_g must be nonnegative")
    servo = result["servo"]
    if not 0 < servo["pwm_min_us"] <= servo["center_us"] <= servo["pwm_max_us"]:
        raise ValueError("PWM center must lie between positive PWM pulse limits")
    if servo["pwm_min_us"] >= servo["pwm_max_us"]:
        raise ValueError("servo.pwm_min_us must be smaller than servo.pwm_max_us")
    if servo["pwm_max_us"] >= servo["pwm_period_s"] * 1e6:
        raise ValueError("PWM pulse must be shorter than its period")
    noise = result["imu_noise"]
    if not isinstance(noise["enabled"], bool):
        raise ValueError("imu_noise.enabled must be boolean")
    if isinstance(noise["seed"], bool) or not isinstance(noise["seed"], int) or noise["seed"] < 0:
        raise ValueError("imu_noise.seed must be a nonnegative integer")
    for key in ("scale", "attitude_std_deg", "rate_std_deg_s", "accel_std_g", "period_s"):
        noise[key] = _number(noise[key], f"imu_noise.{key}")
        if noise[key] < 0 or (key == "period_s" and noise[key] == 0):
            raise ValueError(f"imu_noise.{key} must be {'positive' if key == 'period_s' else 'nonnegative'}")
    for key in ("attitude_std_deg", "rate_std_deg_s", "accel_std_g"):
        if not math.isfinite(noise["scale"] * noise[key]):
            raise ValueError(f"imu_noise.scale * {key} must be finite")
    try:
        json.dumps(result, allow_nan=False)
    except (TypeError, ValueError) as exc:
        raise ValueError("Flight configuration must be JSON-compatible") from exc
    return result


def _autopilot_update(state, command_g, omega_deg_s, measured_g, theta_deg, dt, config):
    """v1 pseudo-angle update with saturation-aware negative integral action."""
    state["alpha_hat_deg"] += dt * (
        config["filter_k_alpha"] / config["filter_k_theta"] *
        (config["filter_b1_s"] * omega_deg_s + theta_deg) - state["alpha_hat_deg"]
    ) / config["filter_tau_s"]
    error = command_g - measured_g
    next_integral = state["integral_g_s"] + error * dt
    delta = -(config["k_rate"] * omega_deg_s +
              config["k_alpha"] * state["alpha_hat_deg"] + config["k_int"] * next_integral)
    limit = config["saturation_deg"]
    integral_action = -config["k_int"] * error * dt
    if abs(delta) >= limit and delta * integral_action > 0:
        feedback = config["k_rate"] * omega_deg_s + config["k_alpha"] * state["alpha_hat_deg"]
        old_delta = -(feedback + config["k_int"] * state["integral_g_s"])
        # Retain only the fraction that reaches the boundary on its first
        # crossing; once saturated, reject increments that drive farther out.
        # A pure integrator must still be able to reach the actuator limit.
        same_side_saturated = abs(old_delta) >= limit and old_delta * delta > 0
        next_integral = (state["integral_g_s"] if same_side_saturated else
                         (-math.copysign(limit, delta) - feedback) / config["k_int"])
        delta = -(config["k_rate"] * omega_deg_s +
                  config["k_alpha"] * state["alpha_hat_deg"] + config["k_int"] * next_integral)
    state["integral_g_s"] = next_integral
    return min(limit, max(-limit, delta))


def _pid_update(state, command_g, omega_deg_s, measured_g, dt, config, axis):
    """Overload PID with measurement derivative and conditional integration.

    Specific force is positive down/right; positive mixed pitch/yaw deflection
    produces up/left force, hence the explicit minus sign before the PID sum.
    output_sign is applied by the caller and is included in anti-windup through
    the equivalent symmetric axis saturation.
    """
    prefix = f"pid_{axis}_"
    kp, ki, kd = (config[prefix + name] for name in
                  ("kp_deg_per_g", "ki_deg_per_g_s", "kd_deg_s_per_g"))
    error = command_g - measured_g
    previous = state["previous_measured_g"]
    derivative = 0. if previous is None else (measured_g - previous) / dt
    integral_limit = config[prefix + "integral_limit_g_s"]
    old_integral = state["integral_g_s"]
    integral = min(integral_limit, max(-integral_limit, old_integral + error * dt))
    p, d = kp * error, -kd * derivative
    rate = config[prefix + "rate_gain_s"] * omega_deg_s
    raw = -(p + ki * integral + d + rate)
    limit = config["saturation_deg"]
    # Freeze only when this integral increment drives farther into saturation.
    if abs(raw) > limit and raw * (-ki * (integral-old_integral)) > 0:
        integral = old_integral
        raw = -(p + ki * integral + d + rate)
    state.update(integral_g_s=integral, previous_measured_g=measured_g,
                 derivative_g_s=derivative, error_g=error, p_deg=p,
                 i_deg=ki*integral, d_deg=d, rate_deg=rate,
                 unsaturated_deg=raw, saturated=abs(raw) > limit)
    return min(limit, max(-limit, raw))


class FlightController:
    """200 Hz absolute-time controller; advance_to applies the supplied IMU as ZOH.

    Visual frames can be queued ahead of advance_to, but are consumed only at a
    control tick at/after their timestamp. The simulator calls at exact control
    boundaries; a standalone large advance uses its single IMU sample for every
    due tick. Rendering/observation/snapshot calls never advance this clock.
    """

    def __init__(self, config=None):
        self.config = load_flight_config(config)
        self._c = self.config["controller"]
        self._axis_c = {axis: _axis_config(self._c, axis) for axis in AXES}
        self._config_hash = hashlib.sha256(json.dumps(
            self.config, sort_keys=True, allow_nan=False).encode("utf-8")).hexdigest()
        # Legacy version-1 snapshots predate the optional fields. Migration is
        # permitted only when every new option retains its legacy default.
        defaults = _read_json(DEFAULT_FLIGHT_CONFIG)["controller"]
        self._legacy_config_hash = None
        added = set(AXIS_OVERRIDE_KEYS) | set(ROLL_FIELDS)
        if all(self._c[key] == defaults[key] for key in added):
            legacy_config = copy.deepcopy(self.config)
            for key in added:
                legacy_config["controller"].pop(key)
            self._legacy_config_hash = hashlib.sha256(json.dumps(
                legacy_config, sort_keys=True, allow_nan=False).encode("utf-8")).hexdigest()
        self._state = {
            "advanced_to_s": 0., "tick_count": 0, "control_time_s": None,
            "last_submitted_visual_t_s": None, "last_visual_t_s": None,
            "last_visual_valid": False, "last_valid_visual_t_s": None,
            "previous_valid_visual": None, "visual_queue": [],
            "q_dot_pitch_deg_s": 0., "q_dot_yaw_deg_s": 0.,
            "a_cmd_pitch_g": 0., "a_cmd_yaw_g": 0.,
            "pitch_cmd_deg": 0., "yaw_cmd_deg": 0., "roll_cmd_deg": 0.,
            "command_deg": [0., 0., 0., 0.], "tracking_status": "initial",
            "autopilot_pitch": {"alpha_hat_deg": 0., "integral_g_s": 0.},
            "autopilot_yaw": {"alpha_hat_deg": 0., "integral_g_s": 0.},
            "latest_imu": None, "roll_active": False, "roll_rate_command_deg_s": None,
            "roll_angle_error_deg": None,
            "axis_diagnostics": {axis: _axis_diagnostic() for axis in ("pitch", "yaw", "roll")},
        }
        for axis in AXES:
            self._state.update({f"aim_{axis}_deg": None,
                                f"aim_error_{axis}_deg": 0.,
                                f"a_png_{axis}_g": 0., f"a_aim_{axis}_g": 0.,
                                f"a_measured_raw_{axis}_g": 0.,
                                f"a_measured_{axis}_g": 0.,
                                f"pid_{axis}": _pid_state()})

    @property
    def next_update_time_s(self):
        return self._state["tick_count"] * self._c["period_s"]

    def submit_visual(self, t_s, los_pitch_deg, los_yaw_deg, pitch_rate_deg_s, yaw_rate_deg_s,
                      *, aim_pitch_deg=None, aim_yaw_deg=None,
                      rotation_WL=None, los_ray_L=None):
        """Queue an image measurement and optional red-reference angular ray.

        Target/reference LOS are positive down/right. No reference preserves the
        original PNG-only behavior; red-reference offsets are never differentiated.
        Both target LOS values None represent an invalid frame.
        A measured body ray plus its own attitude allows full 3-D rotation
        compensation, including roll; neither value contains target truth.
        """
        t_s = _time(t_s, "visual.t_s")
        last = self._state["last_submitted_visual_t_s"]
        if last is not None and t_s <= last:
            raise ValueError("Visual timestamps must be strictly increasing")
        if (los_pitch_deg is None) != (los_yaw_deg is None):
            raise ValueError("Both LOS angles must be provided or both None")
        if (aim_pitch_deg is None) != (aim_yaw_deg is None):
            raise ValueError("Both aim angles must be provided or both None")
        if (rotation_WL is None) != (los_ray_L is None):
            raise ValueError("Attitude and measured LOS ray must be provided together")
        if rotation_WL is not None:
            rotation = np.asarray(rotation_WL, dtype=float)
            ray = np.asarray(los_ray_L, dtype=float)
            ray_scale = float(np.max(np.abs(ray))) if ray.shape == (3,) else 0.
            if (rotation.shape != (3, 3) or ray.shape != (3,) or
                    not np.isfinite(rotation).all() or not np.isfinite(ray).all() or
                    np.max(np.abs(rotation)) > 1. + 1e-7 or ray_scale == 0. or
                    not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-7) or
                    not np.isclose(np.linalg.det(rotation), 1., atol=1e-7) or
                    los_pitch_deg is None):
                raise ValueError("Visual attitude must be a rotation and LOS a valid nonzero ray")
            rotation_WL = rotation.tolist()
            scaled_ray = ray / ray_scale
            los_ray_L = (scaled_ray / np.linalg.norm(scaled_ray)).tolist()
        frame = {"t_s": t_s,
                 "los_pitch_deg": None if los_pitch_deg is None else _number(los_pitch_deg, "los_pitch_deg"),
                 "los_yaw_deg": None if los_yaw_deg is None else _number(los_yaw_deg, "los_yaw_deg"),
                 "pitch_rate_deg_s": _number(pitch_rate_deg_s, "pitch_rate_deg_s"),
                 "yaw_rate_deg_s": _number(yaw_rate_deg_s, "yaw_rate_deg_s"),
                 "aim_pitch_deg": None if aim_pitch_deg is None else _number(aim_pitch_deg, "aim_pitch_deg"),
                 "aim_yaw_deg": None if aim_yaw_deg is None else _number(aim_yaw_deg, "aim_yaw_deg"),
                 "rotation_WL": rotation_WL, "los_ray_L": los_ray_L}
        self._state["visual_queue"].append(frame)
        self._state["last_submitted_visual_t_s"] = t_s

    def _neutral(self):
        state = self._state
        state["previous_valid_visual"] = None
        for key in ("q_dot_pitch_deg_s", "q_dot_yaw_deg_s", "a_cmd_pitch_g", "a_cmd_yaw_g",
                    "pitch_cmd_deg", "yaw_cmd_deg", "roll_cmd_deg"):
            state[key] = 0.
        state["command_deg"] = [0., 0., 0., 0.]
        state["roll_active"] = False
        state["roll_rate_command_deg_s"] = None
        state["roll_angle_error_deg"] = None
        state["axis_diagnostics"] = {axis: _axis_diagnostic() for axis in ("pitch", "yaw", "roll")}
        for key in ("autopilot_pitch", "autopilot_yaw"):
            state[key] = {"alpha_hat_deg": 0., "integral_g_s": 0.}
        for axis in AXES:
            state[f"aim_{axis}_deg"] = None
            for field in ("aim_error", "a_png", "a_aim"):
                suffix = "deg" if field == "aim_error" else "g"
                state[f"{field}_{axis}_{suffix}"] = 0.
            state[f"pid_{axis}"] = _pid_state()

    def _consume_visual(self, tick_t):
        state = self._state
        consumed_valid = False
        epsilon = 1e-12
        while state["visual_queue"] and state["visual_queue"][0]["t_s"] <= tick_t + epsilon:
            frame = state["visual_queue"].pop(0)
            state["last_visual_t_s"] = frame["t_s"]
            state["last_visual_valid"] = frame["los_pitch_deg"] is not None
            if not state["last_visual_valid"]:
                continue
            previous = state["previous_valid_visual"]
            if previous is not None and frame["t_s"] - previous["t_s"] > self._c["vision_timeout_s"] + epsilon:
                self._neutral()
                previous = None
            if previous is None:
                state["q_dot_pitch_deg_s"] = state["q_dot_yaw_deg_s"] = 0.
            else:
                dt = frame["t_s"] - previous["t_s"]
                if dt > _LOS_MIN_DT_S:
                    transported_angles = None
                    if frame.get("rotation_WL") is not None and previous.get("rotation_WL") is not None:
                        # Express the old measured inertial ray in the NEW body
                        # axes. A fixed world ray now has exactly zero LOS rate,
                        # regardless of pitch/yaw/roll and off-axis geometry.
                        old_ray_here = (np.asarray(frame["rotation_WL"]).T @
                            np.asarray(previous["rotation_WL"]) @ np.asarray(previous["los_ray_L"]))
                        transported_angles = {
                            "pitch": math.degrees(math.atan2(-old_ray_here[2], old_ray_here[0])),
                            "yaw": math.degrees(math.atan2(-old_ray_here[1], old_ray_here[0]))}
                    for axis in AXES:
                        camera_rate = ((frame[f"los_{axis}_deg"] - previous[f"los_{axis}_deg"] + 180.) % 360. - 180.) / dt
                        rotation_rate = frame[f"{axis}_rate_deg_s"]
                        if transported_angles is not None:
                            rotation_rate = ((transported_angles[axis] - previous[f"los_{axis}_deg"] + 180.) % 360. - 180.) / dt
                        state[f"q_dot_{axis}_deg_s"] = (
                            self._c[f"{axis}_los_sign"] * camera_rate -
                            self._c[f"{axis}_gyro_compensation_sign"] * rotation_rate)
            for axis in AXES:
                reference = frame[f"aim_{axis}_deg"]
                state[f"aim_{axis}_deg"] = reference
                state[f"aim_error_{axis}_deg"] = (0. if reference is None else
                    self._c[f"{axis}_los_sign"] * (frame[f"los_{axis}_deg"]-reference))
            state["previous_valid_visual"] = frame
            state["last_valid_visual_t_s"] = frame["t_s"]
            consumed_valid = True
        return consumed_valid

    def advance_to(self, t_s, imu_dict):
        """Run due control ticks, returning a copy of the four mixed commands."""
        t_s = _time(t_s)
        if t_s < self._state["advanced_to_s"] - 1e-12:
            raise ValueError("Controller time cannot move backwards")
        if not isinstance(imu_dict, Mapping) or set(imu_dict) not in (IMU_KEYS, ALL_IMU_KEYS):
            raise ValueError("IMU must contain seven legacy channels and optional roll_rate_deg_s")
        if self._c["roll_mode"] == "angle_rate" and ROLL_IMU_KEY not in imu_dict:
            raise ValueError("angle_rate roll control requires body +X roll_rate_deg_s")
        imu = {key: _number(value, f"imu.{key}") for key, value in imu_dict.items()}
        state = self._state
        while self.next_update_time_s <= t_s + 1e-12:
            tick_t = self.next_update_time_s
            consumed_valid = self._consume_visual(tick_t)
            state["latest_imu"] = dict(imu)
            for axis in AXES:
                state[f"a_measured_raw_{axis}_g"] = imu[f"a_{axis}_g"]
                state[f"a_measured_{axis}_g"] = self._c[f"{axis}_accel_feedback_sign"] * imu[f"a_{axis}_g"]
            last_valid = state["last_valid_visual_t_s"]
            if last_valid is None or tick_t - last_valid > self._c["vision_timeout_s"] + 1e-12:
                self._neutral()
                state["tracking_status"] = "initial" if last_valid is None else "stale"
            else:
                state["tracking_status"] = "valid" if consumed_valid and state["last_visual_valid"] else "holding"
                for axis in AXES:
                    if not self._c[f"{axis}_enabled"]:
                        for field in ("a_png", "a_aim", "a_cmd"):
                            state[f"{field}_{axis}_g"] = 0.
                        state[f"{axis}_cmd_deg"] = 0.
                        state[f"autopilot_{axis}"] = {"alpha_hat_deg": 0., "integral_g_s": 0.}
                        state[f"pid_{axis}"] = _pid_state()
                        state["axis_diagnostics"][axis] = _axis_diagnostic()
                        continue
                    acceleration = (self._axis_c[axis]["png_N"] * self._c["closing_speed_mps"] *
                                    state[f"q_dot_{axis}_deg_s"] * _DEG2RAD / _G)
                    sign = self._c[f"{axis}_guidance_sign"]
                    state[f"a_png_{axis}_g"] = sign * acceleration
                    aim = self._c[f"aim_{axis}_gain_g_per_deg"] * state[f"aim_error_{axis}_deg"]
                    aim_limit = self._c["aim_accel_limit_g"]
                    state[f"a_aim_{axis}_g"] = sign * min(aim_limit, max(-aim_limit, aim))
                    limit = self._c["max_accel_g"]
                    state[f"a_cmd_{axis}_g"] = min(limit, max(-limit,
                        state[f"a_png_{axis}_g"] + state[f"a_aim_{axis}_g"]))
                    state[f"{axis}_cmd_deg"] = self._autopilot_command(axis, imu)
            # Enhanced roll stabilization is independent of target acquisition;
            # legacy_p intentionally retains its original visual validity gate.
            active_visual = state["tracking_status"] in ("valid", "holding")
            roll = self._roll_command(imu, active_visual)
            state["roll_cmd_deg"] = roll
            pitch, yaw = state["pitch_cmd_deg"], state["yaw_cmd_deg"]
            state["command_deg"] = [pitch+yaw+roll, pitch-yaw-roll,
                                    -pitch+yaw-roll, -pitch-yaw+roll]
            state["control_time_s"] = tick_t
            state["tick_count"] += 1
        state["advanced_to_s"] = t_s
        return np.array(state["command_deg"], dtype=float)

    def _autopilot_command(self, axis, imu):
        """Realize a guidance command; ideal backends override this boundary."""
        state = self._state
        omega = self._c[f"{axis}_rate_feedback_sign"] * imu[f"{axis}_rate_deg_s"]
        config = self._axis_c[axis]
        if self._c["autopilot_mode"] == "guidedemo_v1":
            command = _autopilot_update(state[f"autopilot_{axis}"], state[f"a_cmd_{axis}_g"],
                omega, state[f"a_measured_{axis}_g"], imu[f"{axis}_deg"], self._c["period_s"], config)
        else:
            command = _pid_update(state[f"pid_{axis}"], state[f"a_cmd_{axis}_g"],
                omega, state[f"a_measured_{axis}_g"], self._c["period_s"], self._c, axis)
        sign = self._c[f"{axis}_output_sign"]
        if self._c["autopilot_mode"] == "guidedemo_v1":
            ap = state[f"autopilot_{axis}"]
            rate = config["k_rate"] * omega
            alpha = config["k_alpha"] * ap["alpha_hat_deg"]
            integral = config["k_int"] * ap["integral_g_s"]
            raw = -(rate + alpha + integral)
        else:
            pid = state[f"pid_{axis}"]
            raw = pid["unsaturated_deg"]
            rate, alpha, integral = pid["rate_deg"], pid["p_deg"]+pid["d_deg"], pid["i_deg"]
        state["axis_diagnostics"][axis] = {
            "unlimited_deg": sign*raw, "command_deg": sign*command,
            "limit_deg": config["saturation_deg"], "saturated": abs(raw) >= config["saturation_deg"],
            "rate_term_deg": -sign*rate, "alpha_term_deg": -sign*alpha,
            "integral_term_deg": -sign*integral}
        return sign * command

    def _roll_command(self, imu, active_visual):
        state, config = self._state, self._c
        active = config["roll_enabled"] and (config["roll_mode"] == "angle_rate" or active_visual)
        state["roll_active"] = bool(active)
        state["roll_rate_command_deg_s"] = None
        state["roll_angle_error_deg"] = None
        diagnostic = _axis_diagnostic()
        if not active:
            state["axis_diagnostics"]["roll"] = diagnostic
            return 0.
        if config["roll_mode"] == "legacy_p":
            # Preserve the original unwrapped -gain*phi expression at zero target.
            error = config["roll_target_deg"] - imu["roll_deg"]
            raw = config["roll_gain"] * error
            limit = config["roll_saturation_deg"]
        else:
            error = (config["roll_target_deg"] - imu["roll_deg"] + 180.) % 360. - 180.
            rate_limit = config["roll_rate_limit_deg_s"]
            rate_command = min(rate_limit, max(-rate_limit, config["roll_angle_gain_per_s"]*error))
            state["roll_rate_command_deg_s"] = rate_command
            raw = config["roll_rate_gain_s"] * (rate_command-imu[ROLL_IMU_KEY])
            limit = config["roll_saturation_deg"]
            if limit is None:
                limit = config["saturation_deg"]
        command = raw if limit is None else min(limit, max(-limit, raw))
        state["roll_angle_error_deg"] = error
        diagnostic.update(unlimited_deg=raw, command_deg=command, limit_deg=limit,
                          saturated=limit is not None and abs(raw) >= limit)
        state["axis_diagnostics"]["roll"] = diagnostic
        return command

    def capture_state(self):
        """Return an independent JSON-compatible state, including pending frames."""
        return {"version": 2, "config_sha256": self._config_hash, "state": copy.deepcopy(self._state)}

    def restore_state(self, snapshot):
        """Restore only compatible snapshots without advancing control or consuming frames."""
        if (not isinstance(snapshot, dict) or isinstance(snapshot.get("version"), bool) or
                snapshot.get("version") not in (1, 2) or not isinstance(snapshot.get("state"), dict)):
            raise ValueError("Flight controller snapshot is incompatible")
        old_snapshot = snapshot["version"] == 1
        valid_hashes = {self._config_hash}
        if old_snapshot and self._legacy_config_hash is not None:
            valid_hashes.add(self._legacy_config_hash)
        if snapshot.get("config_sha256") not in valid_hashes:
            raise ValueError("Flight controller snapshot configuration is incompatible")
        try:
            # Upgrade a historical snapshot only at legacy-equivalent settings.
            candidate = json.loads(json.dumps(snapshot["state"], allow_nan=False))
            if old_snapshot:
                additions = {"latest_imu": None, "roll_active": False,
                    "roll_rate_command_deg_s": None, "roll_angle_error_deg": None,
                    "axis_diagnostics": {axis: _axis_diagnostic() for axis in ("pitch", "yaw", "roll")}}
                for key, value in additions.items():
                    candidate.setdefault(key, value)
            if set(candidate) != set(self._state):
                raise ValueError("Flight controller snapshot state fields are incompatible")
            count = candidate["tick_count"]
            if isinstance(count, bool) or not isinstance(count, int) or count < 0:
                raise ValueError("Invalid controller tick count")
            advanced = _time(candidate["advanced_to_s"])
            control_time = candidate["control_time_s"]
            if count == 0:
                if control_time is not None:
                    raise ValueError("Unstarted controller has a control timestamp")
            elif (abs(_time(control_time) - (count-1)*self._c["period_s"]) > 1e-9 or
                  control_time > advanced + 1e-12):
                raise ValueError("Invalid controller clock in snapshot")
            if candidate["tracking_status"] not in ("initial", "valid", "holding", "stale"):
                raise ValueError("Invalid tracking status")
            if not isinstance(candidate["last_visual_valid"], bool):
                raise ValueError("Invalid visual validity")
            for key in ("last_submitted_visual_t_s", "last_visual_t_s", "last_valid_visual_t_s"):
                if candidate[key] is not None:
                    _time(candidate[key], key)
            for key in ("q_dot_pitch_deg_s", "q_dot_yaw_deg_s", "a_cmd_pitch_g", "a_cmd_yaw_g",
                        "pitch_cmd_deg", "yaw_cmd_deg", "roll_cmd_deg"):
                _number(candidate[key], key)
            for axis in AXES:
                if candidate[f"aim_{axis}_deg"] is not None:
                    _number(candidate[f"aim_{axis}_deg"], f"aim_{axis}_deg")
                for field, unit in (("aim_error", "deg"), ("a_png", "g"), ("a_aim", "g"),
                                    ("a_measured_raw", "g"), ("a_measured", "g")):
                    _number(candidate[f"{field}_{axis}_{unit}"], f"{field}_{axis}_{unit}")
                pid = candidate[f"pid_{axis}"]
                if not isinstance(pid, dict) or set(pid) != set(_pid_state()):
                    raise ValueError("Invalid PID state")
                for key, value in pid.items():
                    if key == "saturated":
                        if not isinstance(value, bool):
                            raise ValueError("Invalid PID saturation flag")
                    elif key != "previous_measured_g" or value is not None:
                        _number(value, key)
                if abs(pid["integral_g_s"]) > self._c[f"pid_{axis}_integral_limit_g_s"] + 1e-12:
                    raise ValueError("PID integral exceeds configured limit")
            if not isinstance(candidate["command_deg"], list) or len(candidate["command_deg"]) != 4:
                raise ValueError("Invalid mixed command")
            for value in candidate["command_deg"]:
                _number(value, "command_deg")
            for key in ("autopilot_pitch", "autopilot_yaw"):
                if set(candidate[key]) != {"alpha_hat_deg", "integral_g_s"}:
                    raise ValueError("Invalid autopilot state")
                for value in candidate[key].values():
                    _number(value, key)
            latest = candidate["latest_imu"]
            if latest is not None:
                if not isinstance(latest, dict) or set(latest) not in (IMU_KEYS, ALL_IMU_KEYS):
                    raise ValueError("Invalid latest IMU snapshot")
                if self._c["roll_mode"] == "angle_rate" and ROLL_IMU_KEY not in latest:
                    raise ValueError("angle_rate snapshot requires body +X roll rate")
                for key, value in latest.items():
                    _number(value, key)
            if not isinstance(candidate["roll_active"], bool):
                raise ValueError("Invalid roll activity")
            for key in ("roll_rate_command_deg_s", "roll_angle_error_deg"):
                if candidate[key] is not None:
                    _number(candidate[key], key)
            if set(candidate["axis_diagnostics"]) != {"pitch", "yaw", "roll"}:
                raise ValueError("Invalid axis diagnostics")
            for axis, values in candidate["axis_diagnostics"].items():
                if set(values) != set(_axis_diagnostic()) or not isinstance(values["saturated"], bool):
                    raise ValueError("Invalid axis diagnostic fields")
                for key, value in values.items():
                    if key != "saturated" and value is not None:
                        _number(value, key)
            if not isinstance(candidate["visual_queue"], list):
                raise ValueError("Invalid visual queue")
            frames = candidate["visual_queue"]
            checker = FlightController(self.config)
            for frame in frames:
                if not isinstance(frame, dict):
                    raise ValueError("Invalid queued visual frame")
                checker.submit_visual(**frame)
            previous = candidate["previous_valid_visual"]
            if previous is not None:
                checker = FlightController(self.config)
                checker.submit_visual(**previous)
                if previous["los_pitch_deg"] is None:
                    raise ValueError("Invalid previous valid visual frame")
        except (TypeError, KeyError, ValueError, OverflowError) as exc:
            raise ValueError(f"Invalid flight controller snapshot: {exc}") from exc
        self._state = candidate

    def observation(self):
        """Read the most recent tick without changing any clock or history."""
        result = {key: copy.deepcopy(value) for key, value in self._state.items()
                  if key not in ("visual_queue", "previous_valid_visual", "last_submitted_visual_t_s")}
        result["visual_valid"] = result.pop("last_visual_valid")
        result["next_update_time_s"] = self.next_update_time_s
        result["pending_visual_frames"] = len(self._state["visual_queue"])
        result["autopilot_mode"] = self._c["autopilot_mode"]
        result["axis_enabled"] = {axis: self._c[f"{axis}_enabled"] for axis in ("pitch", "yaw", "roll")}
        result["effective_axis_config"] = {axis: {field: self._axis_c[axis][field]
            for field in AXIS_OVERRIDE_FIELDS} for axis in AXES}
        result["roll_mode"] = self._c["roll_mode"]
        result["roll_target_deg"] = self._c["roll_target_deg"]
        result["axis_signs"] = {axis: {field: self._c[f"{axis}_{field}"]
                                      for field in SIGN_FIELDS} for axis in AXES}
        for axis in AXES:
            result[f"a_error_{axis}_g"] = result[f"a_cmd_{axis}_g"] - result[f"a_measured_{axis}_g"]
        return result

    def metadata(self):
        """Record active formulas, original source hashes and all effective parameters."""
        try:
            archive = json.loads(SOURCE_MANIFEST.read_text(encoding="utf-8"))
            hashes = {item["path"]: item["sha256"] for item in archive["files"]}
        except (OSError, ValueError, KeyError, TypeError) as exc:
            raise ValueError(f"Cannot read archived Guidedemo source provenance: {exc}") from exc
        return {"model": self.config["model"], "source_manifest": str(SOURCE_MANIFEST),
                "source_hashes": hashes, "config": copy.deepcopy(self.config),
                "config_sha256": self._config_hash, "limitations": list(LIMITATIONS),
                "tail_layout": copy.deepcopy(self.config["tail_layout"]),
                "source_mixing_matrix": [list(row) for row in SOURCE_MIXING_MATRIX],
                "mixing_columns": ["pitch", "yaw", "roll"],
                "control_mechanics": "aggregate control derivatives; not per-fin hinges or per-fin CFD loads",
                "input_units": {"attitude": "deg", "angular_rate": "deg/s", "specific_force": "g"},
                "input_imu_keys": sorted(ALL_IMU_KEYS),
                "legacy_optional_imu_key": ROLL_IMU_KEY,
                "roll_policy": "legacy_p keeps visual gate; angle_rate uses wrapped angle outer P and body +X rate inner P independently of vision; overall enable is owned by simulation", "output": "four mixed servo commands in deg",
                "autopilot_mode": self._c["autopilot_mode"],
                "overload_convention": "specific force in g; pitch positive down, yaw positive right; measured_raw is supplied IMU, measured includes accel_feedback_sign",
                "aim_policy": "PNG uses real target LOS derivatives; explicit red-reference error adds bounded proportional overload correction; no reference preserves legacy PNG",
                "pid_policy": "optional acceleration_pid: negative Kp/Ki acceleration-error output, derivative on measured acceleration, optional angular-rate damping, bounded conditional-integration anti-windup",
                "axis_sign_policy": "q_dot=los_sign*target_LOS_derivative-gyro_compensation_sign*body_rate; guidance_sign multiplies PNG+aim; feedback signs act before controller; output_sign acts before X-tail mixing",
                "tick_policy": "absolute simulation time; tick at t=0, then integer multiples of period_s",
                "vision_policy": "strictly ordered frames; consume only at/after frame timestamp; timeout neutralizes pitch/yaw and legacy roll; enabled angle_rate roll continues independently"}
