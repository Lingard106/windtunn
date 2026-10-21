"""Traditional predictive PN partition for the original complete physical plant.

No real aerodynamic/servo/rigid-body dynamics are replaced. The predictor's
normal-force state is measured TOTAL normal specific force, not isolated control
surface force: treating it as a first-order decaying state is an approximation.
"""
from __future__ import annotations

import copy
import hashlib
from functools import partial
import json
import math
from collections.abc import Mapping
from pathlib import Path
import sys

import mujoco
import numpy as np

PROJECT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = Path(__file__).with_name("config.json")
PROTOTYPES = PROJECT / "prototypes"
for directory in (PROJECT, PROTOTYPES):
    if str(directory) not in sys.path:
        sys.path.insert(0, str(directory))

from dart_demo import DemoSimulation, build_xml
from guidedemo_control import FlightController, load_flight_config
from pn_expert_v2.expert import BallisticPNExpert

G = 9.80665
CANDIDATE_DEFAULTS = dict(max_control_g=.2, png_N=3., response_tau_s=.1,
    max_slew_g_s=2.5, update_period_s=.02, prediction_dt_s=.025,
    vision_timeout_s=.15, vision_filter_tau_s=.08, drag_enabled=True)


def candidate_parameters(value=None):
    if value is not None and not isinstance(value, Mapping):
        raise ValueError("candidate_config must be a mapping")
    supplied = {} if value is None else dict(value)
    if set(supplied) - set(CANDIDATE_DEFAULTS):
        raise ValueError("Unknown physical candidate configuration field")
    result = dict(CANDIDATE_DEFAULTS, **copy.deepcopy(supplied))
    for key, value in result.items():
        if key == "drag_enabled":
            if not isinstance(value, bool):
                raise ValueError("candidate.drag_enabled must be boolean")
        elif isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"candidate.{key} must be positive and finite")
        else:
            result[key] = float(value)
    return result


PREDICTION_DEFAULTS = dict(mass_kg=.25, rho_kg_m3=1.225,
    area_m2=.0033587151005579547, drag_coefficient=.35)


def prediction_parameters(value=None):
    """Validate a fixed nominal model; never derive it from a perturbed plant."""
    if value is not None and not isinstance(value, Mapping):
        raise ValueError("prediction_model must be a mapping")
    supplied = {} if value is None else dict(value)
    if set(supplied)-set(PREDICTION_DEFAULTS):
        raise ValueError("Unknown prediction_model parameter")
    result = dict(PREDICTION_DEFAULTS, **copy.deepcopy(supplied))
    for key, value in result.items():
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or value < 0
                or (key != "drag_coefficient" and value == 0)):
            raise ValueError(f"prediction_model.{key} must be finite and "
                             + ("nonnegative" if key == "drag_coefficient" else "positive"))
        result[key] = float(value)
    return result


def _document(value):
    if isinstance(value, (str, Path)):
        try:
            return json.loads(Path(value).read_text(encoding="utf-8-sig"))
        except (OSError, json.JSONDecodeError) as exc:
            raise ValueError(f"Cannot read traditional control config: {exc}") from exc
    if not isinstance(value, Mapping):
        raise ValueError("Configuration must be a path or mapping")
    return copy.deepcopy(dict(value))


def _merge_flight(base, supplied):
    result = copy.deepcopy(base)
    for section, fields in supplied.items():
        if isinstance(fields, Mapping) and isinstance(result.get(section), dict):
            result[section].update(copy.deepcopy(dict(fields)))
        else:
            result[section] = copy.deepcopy(fields)
    return load_flight_config(result)


def load_traditional_config(value=None):
    """Load the independent bundle, with explicit nested overrides."""
    defaults = _document(DEFAULT_CONFIG)
    supplied = {} if value is None else _document(value)
    if set(supplied)-set(defaults):
        raise ValueError("Unknown traditional configuration fields")
    result = copy.deepcopy(defaults)
    for key, value in supplied.items():
        if key in ("candidate", "prediction_model"):
            if not isinstance(value, Mapping):
                raise ValueError(f"{key} must be an object")
            result[key].update(copy.deepcopy(dict(value)))
        elif key == "flight_control":
            if not isinstance(value, Mapping):
                raise ValueError("flight_control must be an object")
            result[key] = _merge_flight(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    if isinstance(result["schema_version"], bool) or result["schema_version"] != 1:
        raise ValueError("traditional schema_version must be 1")
    if result["model"] != "traditional_predictive_PN_physical":
        raise ValueError("Unsupported traditional control model")
    if not isinstance(result["provenance"], dict):
        raise ValueError("traditional provenance must be an object")
    result["candidate"] = candidate_parameters(result["candidate"])
    result["prediction_model"] = prediction_parameters(result["prediction_model"])
    result["flight_control"] = load_flight_config(result["flight_control"])
    try:
        json.dumps(result, allow_nan=False)
    except (ValueError, TypeError) as exc:
        raise ValueError("Traditional config must be finite JSON data") from exc
    return result


def candidate_flight_parameters(value=None):
    """Compatibility helper: defaults plus explicit full/partial flight config."""
    defaults = load_traditional_config()
    if value is None:
        return defaults["flight_control"]
    supplied = _document(value)
    if any(key in supplied for key in ("candidate", "flight_control", "prediction_model")):
        return load_traditional_config(supplied)["flight_control"]
    return _merge_flight(defaults["flight_control"], supplied)


def _pair(value, name):
    if np.iscomplexobj(value):
        raise ValueError(f"{name} must contain two finite real values")
    result = np.asarray(value, dtype=float)
    if result.shape != (2,) or not np.isfinite(result).all():
        raise ValueError(f"{name} must contain two finite real values")
    return result.copy()


def velocity_rotation(velocity_W, fallback_rotation=None):
    """Zero-Euler-roll velocity frame, not an imposed physical attitude."""
    velocity = np.asarray(velocity_W, dtype=float)
    if velocity.shape != (3,) or not np.isfinite(velocity).all():
        raise ValueError("own velocity must have three finite entries")
    speed = float(np.linalg.norm(velocity))
    if speed < 1e-10:
        return np.eye(3) if fallback_rotation is None else np.asarray(fallback_rotation).copy()
    forward = velocity/speed
    horizontal = math.hypot(velocity[0], velocity[1])
    left = (np.array([-velocity[1], velocity[0], 0.])/horizontal
            if horizontal >= 1e-10 else np.array([0., 1., 0.]))
    return np.column_stack((forward, left, np.cross(forward, left)))


class CommandAutopilot(FlightController):
    """Replace guidance at the existing AP boundary; keep original realization."""

    def __init__(self, config=None):
        self._external_body_g = np.zeros(2)
        self._external_time_s = None
        super().__init__(config)

    def set_external_command(self, body_g, time_s):
        request = _pair(body_g, "external_body_g")
        if isinstance(time_s, bool) or not isinstance(time_s, (int, float)) or not math.isfinite(time_s) or time_s < 0:
            raise ValueError("external command time must be finite and nonnegative")
        self._external_body_g = request
        self._external_time_s = float(time_s)

    def _autopilot_command(self, axis, imu):
        # Base advance_to still owns visual validity, enabled axes, measured
        # feedback, state resets, roll behavior and four-fin mixing.
        index = 0 if axis == "pitch" else 1
        limit = self._c["max_accel_g"]
        request = min(limit, max(-limit, float(self._external_body_g[index])))
        self._state[f"a_cmd_{axis}_g"] = request
        self._state[f"a_png_{axis}_g"] = request
        self._state[f"a_aim_{axis}_g"] = 0.
        return super()._autopilot_command(axis, imu)

    def observation(self):
        result = super().observation()
        result["external_body_command_g"] = dict(pitch=float(self._external_body_g[0]),
                                                  yaw=float(self._external_body_g[1]))
        result["external_command_time_s"] = self._external_time_s
        result["guidance_adapter"] = "physical_candidate_predictive_PN"
        return result


def _snapshot_number(value, name, *, nonnegative=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    value = float(value)
    if nonnegative and value < 0:
        raise ValueError(f"{name} must be nonnegative")
    return value


def _snapshot_time(value, name, maximum=None, *, optional=False):
    if value is None and optional:
        return None
    value = _snapshot_number(value, name, nonnegative=True)
    if maximum is not None and value > maximum+1e-9:
        raise ValueError(f"{name} is later than the physical snapshot")
    return value


def _snapshot_vector(value, size, name):
    if np.iscomplexobj(value):
        raise ValueError(f"{name} must be real")
    array = np.asarray(value, dtype=float)
    if array.shape != (size,) or not np.isfinite(array).all():
        raise ValueError(f"{name} must contain {size} finite values")
    return array.copy()


_ADAPTER_ATTRIBUTES = ("_pn_expert", "_observer_calibration", "_original_nose_L",
    "_target_estimate_W", "_target_estimate_t", "_last_upper_visual_t",
    "_candidate_velocity_g", "_candidate_body_g", "_candidate_measured_total_g",
    "_candidate_axial_body_g", "_candidate_update_t", "_expert_diagnostics",
    "_candidate_plant_sha")


class PhysicalCandidate(DemoSimulation):
    """Traditional simulation; every actual force still uses the physical plant.

    Existing perfect-own-navigation assumption is explicit. The control path
    never queries target_motion, dynamic target_position_W, target seeds, true
    wind, or target crossing/contact truth.
    """

    benchmark_only = False
    snapshot_replay_supported = True

    def __init__(self, config, scene_manifest=None, dart_manifest=None, launch_profile=None,
                 *, target_mode="fixed", target_seed=2027, aero_model=None,
                 flight_control_enabled=False, flight_control_config=None,
                 candidate_config=None, prediction_model=None):
        supplied = None if flight_control_config is None else _document(flight_control_config)
        bundled = supplied is not None and any(key in supplied
            for key in ("candidate", "flight_control", "prediction_model"))
        bundle = load_traditional_config(supplied if bundled else None)
        self._traditional_provenance = copy.deepcopy(bundle["provenance"])
        parameters = dict(bundle["candidate"])
        if candidate_config is not None:
            if not isinstance(candidate_config, Mapping):
                raise ValueError("candidate_config must be a mapping")
            parameters.update(dict(candidate_config))
        self.candidate_config = candidate_parameters(parameters)
        nominal = dict(bundle["prediction_model"])
        if prediction_model is not None:
            if not isinstance(prediction_model, Mapping):
                raise ValueError("prediction_model must be a mapping")
            nominal.update(dict(prediction_model))
        self.prediction_model = prediction_parameters(nominal)
        flight = bundle["flight_control"] if supplied is None or bundled else _merge_flight(bundle["flight_control"], supplied)
        self._candidate_plant_sha = None
        # Set every adapter member BEFORE DemoSimulation.__init__ invokes reset
        # dynamically, including observation() calls inside the base reset.
        self._clear_candidate_state()
        super().__init__(config, scene_manifest=scene_manifest, dart_manifest=dart_manifest,
            launch_profile=launch_profile, target_mode=target_mode, target_seed=target_seed,
            aero_model=aero_model, flight_control_enabled=flight_control_enabled,
            flight_control_config=flight)

    def _clear_candidate_state(self):
        self._pn_expert = None
        self._observer_calibration = None
        self._original_nose_L = None
        self._target_estimate_W = None
        self._target_estimate_t = None
        self._last_upper_visual_t = None
        self._candidate_velocity_g = np.zeros(2)
        self._candidate_body_g = np.zeros(2)
        self._candidate_measured_total_g = np.zeros(2)
        self._candidate_axial_body_g = 0.
        self._candidate_update_t = None
        self._expert_diagnostics = dict(status="awaiting_visual")

    @property
    def simulation_factory(self):
        return partial(type(self), candidate_config=copy.deepcopy(self.candidate_config),
                       prediction_model=copy.deepcopy(self.prediction_model))

    @property
    def traditional_config(self):
        return dict(schema_version=1, model="traditional_predictive_PN_physical",
            provenance=copy.deepcopy(self._traditional_provenance),
            candidate=copy.deepcopy(self.candidate_config),
            prediction_model=copy.deepcopy(self.prediction_model),
            flight_control=copy.deepcopy(self.flight_config))

    def apply_candidate_config(self, supplied):
        if not isinstance(supplied, Mapping):
            raise ValueError("candidate changes must be a mapping")
        parameters = candidate_parameters(dict(self.candidate_config, **dict(supplied)))
        candidate = copy.copy(self)
        candidate.candidate_config = parameters
        candidate._external_renderer = candidate._onboard_renderer = candidate._graphics_renderer = None
        try:
            observation = candidate.reset()
        except Exception:
            candidate.close()
            raise
        self.close()
        self.__dict__ = candidate.__dict__
        return observation

    def reset(self, config=None, **kwargs):
        # Reject unsupported worlds before the base can replace the old model.
        if self.launch_profile is None or self.launch_profile.guide_light is None:
            raise ValueError("Traditional control requires static lamp/upper-target calibration")
        if self.dart_model is None:
            raise ValueError("Traditional control requires the actual dart manifest")
        previous = {name: getattr(self, name) for name in _ADAPTER_ATTRIBUTES}
        try:
            self._clear_candidate_state()
            super().reset(config, **kwargs)
            self.flight_controller = CommandAutopilot(self.flight_config)
            self._setup_candidate()
            launch = [*self.launch_profile.position_W_m[:2], self.cfg["launch_height_m"]]
            xml = build_xml(self.cfg, self.field_scene, self.dart_model, launch,
                suppress_demo_props=True, launch_profile=self.launch_profile,
                guide_light_brightness=self.cfg.get("guide_light_brightness", .8),
                target_mode=self.target_mode)
            self._candidate_plant_sha = hashlib.sha256(xml.encode("utf-8")).hexdigest()
        except Exception:
            # Base validation/compilation failures preserve the previous world;
            # likewise keep its adapter state instead of clearing it accidentally.
            for name, value in previous.items():
                setattr(self, name, value)
            raise
        return self.observation()

    def _setup_candidate(self):
        if self.launch_profile is None or self.launch_profile.guide_light is None:
            raise ValueError("PhysicalCandidate requires static launch-profile lamp/upper-target calibration")
        if self.dart_model is None:
            raise ValueError("PhysicalCandidate requires the actual dart manifest, including its nose offset")
        profile = self.launch_profile
        lamp = np.asarray(profile.guide_light.center_W_m, dtype=float)
        self._observer_calibration = dict(lamp_plane_x_m=float(lamp[0]),
            lamp_to_target_W_m=(np.asarray(profile.target_position_W_m)-lamp).tolist(),
            source="static surveyed lamp plane and rigid lamp-to-upper-target offset")
        self._original_nose_L = np.asarray(self.dart_model.nose_tip, dtype=float).copy()
        expert = {key: value for key, value in self.candidate_config.items()
                  if key != "vision_filter_tau_s"}
        nominal = self.prediction_model
        self._pn_expert = BallisticPNExpert(expert, mass_kg=nominal["mass_kg"],
            rho_kg_m3=nominal["rho_kg_m3"], area_m2=nominal["area_m2"],
            drag_coefficient=nominal["drag_coefficient"], gravity_W=np.asarray(self.model.opt.gravity).copy(),
            plane_normal_W=profile.target_normal_W, nose_offset_L=self._original_nose_L,
            wind_W_mps=(0., 0., 0.))
        self.flight_controller.set_external_command((0., 0.), self.t)

    def submit_visual_measurement(self, details):
        measurement = super().submit_visual_measurement(details)
        if measurement is None:
            return None
        reliable = False
        bbox = details.get("bbox")
        if bbox is not None:
            box = np.asarray(bbox, dtype=float)
            reliable = bool(box.shape == (4,) and np.isfinite(box).all() and
                box[2] > 0 and box[3] > 0 and box[0] > 0 and box[1] > 0 and
                box[0]+box[2] < 320 and box[1]+box[3] < 240 and details.get("locked", False))
        accepted = False
        sample_t = measurement.get("t_s")
        current_frame = (isinstance(sample_t, (int, float)) and not isinstance(sample_t, bool)
            and math.isfinite(sample_t) and abs(sample_t-self.t) <= 1e-9)
        increasing_frame = (current_frame and (self._last_upper_visual_t is None
            or sample_t > self._last_upper_visual_t))
        if measurement["valid"] and reliable and increasing_frame and self._observer_calibration is not None:
            camera_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "onboard")
            origin = np.asarray(self.data.cam_xpos[camera_id], dtype=float).copy()
            body_rotation = self.data.xmat[self.body_id].reshape(3, 3)
            ray = body_rotation @ np.asarray(measurement["los_ray_L"], dtype=float)
            if ray[0] > 1e-8:
                depth = (self._observer_calibration["lamp_plane_x_m"]-origin[0])/ray[0]
                if depth > 0.:
                    target = origin+depth*ray+np.asarray(self._observer_calibration["lamp_to_target_W_m"])
                    elapsed = 0. if self._target_estimate_t is None else self.t-self._target_estimate_t
                    alpha = (1. if self._target_estimate_W is None else
                        1.-math.exp(-elapsed/self.candidate_config["vision_filter_tau_s"]))
                    self._target_estimate_W = (target if self._target_estimate_W is None else
                        self._target_estimate_W+alpha*(target-self._target_estimate_W))
                    self._target_estimate_t = self.t
                    self._last_upper_visual_t = float(sample_t)
                    accepted = True
        measurement["upper_target_measurement_accepted"] = accepted
        measurement["upper_frame_current"] = bool(current_frame)
        measurement["upper_frame_increasing"] = bool(increasing_frame)
        if self._target_estimate_W is not None:
            measurement["upper_target_estimate_W_m"] = self._target_estimate_W.tolist()
        self._last_visual_measurement = copy.deepcopy(measurement)
        return measurement

    def _advance_candidate(self):
        if not self.flight_control_enabled or self._pn_expert is None:
            self._candidate_velocity_g[:] = self._candidate_body_g[:] = 0.
            if isinstance(self.flight_controller, CommandAutopilot):
                self.flight_controller.set_external_command((0., 0.), self.t)
            return
        rotation = self.data.xmat[self.body_id].reshape(3, 3).copy()
        velocity = self.data.qvel[:3].copy()
        frame = velocity_rotation(velocity, rotation)
        measured_specific_velocity = frame.T @ self._flight_specific_force_W / G
        self._candidate_measured_total_g = np.array([-measured_specific_velocity[2],
                                                    -measured_specific_velocity[1]])
        # Present attitude-to-velocity angle is frozen only inside the future
        # nose prediction. Real rigid-body orientation remains unconstrained.
        self._pn_expert.nose_L = frame.T @ rotation @ self._original_nose_L
        age = math.inf if self._target_estimate_t is None else self.t-self._target_estimate_t
        request, diagnostics = self._pn_expert.update(self.t, self.data.qpos[:3].copy(),
            velocity, self._candidate_measured_total_g.copy(),
            None if self._target_estimate_W is None else self._target_estimate_W.copy(),
            valid_age_s=age)
        self._candidate_velocity_g = request.copy()
        specific_W_g = frame @ np.array([0., -request[1], -request[0]])
        specific_body_g = rotation.T @ specific_W_g
        self._candidate_body_g = np.array([-specific_body_g[2], -specific_body_g[1]])
        self._candidate_axial_body_g = float(specific_body_g[0])
        self._candidate_update_t = self.t
        self._expert_diagnostics = diagnostics
        self.flight_controller.set_external_command(self._candidate_body_g, self.t)

    def step(self, steps=1):
        for _ in range(steps):
            if self.done:
                break
            self._advance_candidate()
            super().step(1)
        return self.observation()

    def _candidate_observation(self):
        age = None if self._target_estimate_t is None else self.t-self._target_estimate_t
        raw = self.flight_controller.observation()
        return dict(benchmark_only=False, snapshot_replay_supported=True,
            upper_goal_valid=age is not None and age <= self.candidate_config["vision_timeout_s"],
            upper_goal_age_s=age, raw_visual_gate_status=raw["tracking_status"],
            config=copy.deepcopy(self.candidate_config),
            nominal_prediction_model=copy.deepcopy(self.prediction_model),
            target_estimate_W_m=None if self._target_estimate_W is None else self._target_estimate_W.tolist(),
            target_estimate_time_s=self._target_estimate_t,
            target_estimate_age_s=None if self._target_estimate_t is None else self.t-self._target_estimate_t,
            velocity_frame_command_g=self._candidate_velocity_g.tolist(),
            body_frame_command_g=self._candidate_body_g.tolist(),
            omitted_axial_body_specific_force_g=self._candidate_axial_body_g,
            measured_total_normal_g_velocity=self._candidate_measured_total_g.tolist(),
            candidate_update_time_s=self._candidate_update_t,
            expert=copy.deepcopy(self._expert_diagnostics))

    def _flight_observation(self):
        result = super()._flight_observation()
        result["physical_candidate"] = self._candidate_observation()
        result["guidance_law"] = "ballistic_ZEM_PN_physical_realization"
        result["guidance_source"] = "vision"
        return result

    @property
    def flight_control_metadata(self):
        result = super().flight_control_metadata
        result["physical_candidate"] = dict(
            benchmark_only=False, snapshot_replay_supported=True,
            stale_goal_policy="expert request becomes zero after trusted upper-goal timeout; AP may continue zero-overload feedback with retained integral while raw detections remain valid, not servo neutralization",
            trusted_frame_policy="candidate EMA accepts only current-timestamp (1e-9) strictly increasing locked unclipped frames; raw base visual gate unchanged",
            config=copy.deepcopy(self.candidate_config),
            nominal_prediction_model=copy.deepcopy(self.prediction_model),
            true_plant_parameters=dict(mass_kg=self.cfg["mass_kg"],
                rho_kg_m3=self.cfg["rho_kg_m3"], area_m2=self.cfg["reference_area_m2"],
                drag_coefficient=self.cfg["drag_coefficient"]),
            prediction_model_policy="fixed nominal calibration; physical cfg perturbations are not read by predictor; explicit prediction_model override only",
            observer_calibration=copy.deepcopy(self._observer_calibration),
            original_nose_offset_body_m=None if self._original_nose_L is None else self._original_nose_L.tolist(),
            plant="unchanged DemoSimulation six-DOF / passive aero / aggregate fin loads / PWM / 30deg mechanical limits",
            navigation="perfect own position, velocity, attitude; already-cached finite-difference non-gravity specific force before optional AP IMU noise",
            predictor_input="measured TOTAL normal specific force in velocity frame, including passive normal force; not isolated actuator state",
            predictor_approximation="total normal force treated as a decaying first-order equivalent state; ignores physical passive/control coupling, requires held-out full-plant validation",
            nose_prediction="current body-to-velocity nose offset; future attitude/velocity misalignment assumed constant",
            wind_estimate_W_mps=[0., 0., 0.],
            target_policy="current/past actual image detections plus declared static calibration only; no dynamic target truth, seed or future trajectory",
            red_reference="not used by predictive target reconstruction; legacy AP boundary sets a_aim to zero")
        return result

    def _configuration_hash(self):
        compatibility = dict(candidate=self.candidate_config, prediction_model=self.prediction_model,
            flight_control=self.flight_config,
            numeric_config=self.cfg, plant_xml_sha256=self._candidate_plant_sha,
            calibration=self._observer_calibration,
            nose_offset_body_m=self._original_nose_L.tolist(),
            target_mode=self.target_mode, target_seed=self.target_seed,
            flight_control_enabled=self.flight_control_enabled)
        return hashlib.sha256(json.dumps(compatibility, sort_keys=True,
            allow_nan=False).encode("utf-8")).hexdigest()

    def capture_state(self):
        """Capture complete base physics and adapter state without taking a sensor sample."""
        if self._pn_expert is None or self._candidate_plant_sha is None:
            raise ValueError("Traditional control is not fully initialized")
        adapter = dict(external_body_g=self.flight_controller._external_body_g.tolist(),
            external_time_s=self.flight_controller._external_time_s,
            expert_state=self._pn_expert.capture_state(),
            expert_nose_L=self._pn_expert.nose_L.tolist(),
            target_estimate_W=None if self._target_estimate_W is None else self._target_estimate_W.tolist(),
            target_estimate_t=self._target_estimate_t,
            last_upper_visual_t=self._last_upper_visual_t,
            velocity_request_g=self._candidate_velocity_g.tolist(),
            body_request_g=self._candidate_body_g.tolist(),
            measured_total_normal_g=self._candidate_measured_total_g.tolist(),
            omitted_axial_body_g=self._candidate_axial_body_g,
            candidate_update_t=self._candidate_update_t,
            expert_diagnostics=copy.deepcopy(self._expert_diagnostics))
        return dict(traditional_snapshot_version=1, time_s=self.t,
            prediction_model=copy.deepcopy(self.prediction_model),
            configuration_sha256=self._configuration_hash(),
            base=DemoSimulation.capture_state(self), adapter=adapter)

    def _validated_adapter(self, saved, time_s):
        expected = {"external_body_g", "external_time_s", "expert_state", "expert_nose_L",
            "target_estimate_W", "target_estimate_t", "last_upper_visual_t",
            "velocity_request_g", "body_request_g", "measured_total_normal_g",
            "omitted_axial_body_g", "candidate_update_t", "expert_diagnostics"}
        if not isinstance(saved, Mapping) or set(saved) != expected:
            raise ValueError("Traditional adapter snapshot fields are incompatible")
        result = copy.deepcopy(dict(saved))
        for key in ("external_body_g", "velocity_request_g", "body_request_g", "measured_total_normal_g"):
            result[key] = _pair(result[key], key)
        for key in ("external_time_s", "target_estimate_t", "last_upper_visual_t", "candidate_update_t"):
            result[key] = _snapshot_time(result[key], key, time_s, optional=True)
        result["expert_nose_L"] = _snapshot_vector(result["expert_nose_L"], 3, "expert_nose_L")
        if not math.isclose(float(np.linalg.norm(result["expert_nose_L"])),
                float(np.linalg.norm(self._original_nose_L)), rel_tol=1e-9, abs_tol=1e-10):
            raise ValueError("Expert nose length does not match the original physical nose")
        target = result["target_estimate_W"]
        result["target_estimate_W"] = None if target is None else _snapshot_vector(target, 3, "target_estimate_W")
        if (target is None) != (result["target_estimate_t"] is None):
            raise ValueError("Target estimate and timestamp must be available together")
        if (target is None) != (result["last_upper_visual_t"] is None):
            raise ValueError("Trusted frame and target estimate must be available together")
        if target is not None and abs(result["target_estimate_t"]-result["last_upper_visual_t"]) > 1e-9:
            raise ValueError("Trusted frame timestamp does not match the reconstructed target")
        maximum = self.candidate_config["max_control_g"]
        for key in ("velocity_request_g", "body_request_g"):
            if np.linalg.norm(result[key]) > maximum+1e-10:
                raise ValueError("Saved candidate request exceeds its configured limit")
        if not np.array_equal(result["external_body_g"], result["body_request_g"]):
            raise ValueError("External AP request differs from the saved body-frame request")
        result["omitted_axial_body_g"] = _snapshot_number(result["omitted_axial_body_g"], "omitted_axial_body_g")
        if not isinstance(result["expert_diagnostics"], dict):
            raise ValueError("Expert diagnostics must be an object")
        json.dumps(result["expert_diagnostics"], allow_nan=False)
        json.dumps(result["expert_state"], allow_nan=False)
        expert = copy.deepcopy(self._pn_expert)
        expert.restore_state(result["expert_state"])
        _snapshot_time(expert._last_update_s, "expert.last_update_s", time_s, optional=True)
        expert.nose_L = result["expert_nose_L"].copy()
        # Estimated wind is fixed by this experiment, never restored from truth.
        expert.wind_W = np.zeros(3)
        result["expert"] = expert
        return result

    def _validate_base_snapshot(self, base, time_s):
        if not isinstance(base, Mapping):
            raise ValueError("Missing base physical snapshot")
        for key in ("done", "was_target_contact"):
            if not isinstance(base.get(key), bool):
                raise ValueError(f"Invalid base {key}")
        if not isinstance(base.get("status"), str):
            raise ValueError("Invalid base status")
        for key in ("internal_step_count", "macro_step_count"):
            count = base.get(key)
            if isinstance(count, bool) or not isinstance(count, int) or count < 0:
                raise ValueError(f"Invalid base {key}")
        _snapshot_number(base.get("ground_settle_s"), "ground_settle_s", nonnegative=True)
        for key in ("force", "torque"):
            _snapshot_vector(base.get(key), 3, key)
        integration = np.asarray(base.get("integration"), dtype=float)
        size = mujoco.mj_stateSize(self.model, mujoco.mjtState.mjSTATE_INTEGRATION)
        if integration.shape != (size,) or not np.isfinite(integration).all():
            raise ValueError("Invalid base integration state")
        trajectory = np.asarray(base.get("trajectory"), dtype=float)
        if trajectory.ndim != 2 or trajectory.shape[1] != 3 or not np.isfinite(trajectory).all():
            raise ValueError("Invalid base trajectory")
        flight = base.get("flight_control")
        if (not isinstance(flight, Mapping) or not isinstance(flight.get("enabled"), bool)
                or flight.get("enabled") != self.flight_control_enabled):
            raise ValueError("Saved flight-control enable differs from the current experiment")
        for key in ("specific_force_W", "control_force_W", "control_torque_W"):
            _snapshot_vector(flight.get(key), 3, key)
        _snapshot_number(flight.get("yaw_reference_deg"), "yaw_reference_deg")
        control = CommandAutopilot(self.flight_config)
        control.restore_state(flight["controller"])
        _snapshot_time(control._state["advanced_to_s"], "controller.advanced_to_s", time_s)
        for frame in control._state["visual_queue"]:
            _snapshot_time(frame["t_s"], "queued visual time", time_s)
        # Prevalidate the execution/noise components before touching live state.
        from control_actuation import ServoActuation
        from imu_noise import ImuNoise
        servo = ServoActuation(self.flight_config)
        servo.restore_state(flight["servo"])
        _snapshot_time(servo.observation()["time_s"], "servo.time_s", time_s)
        noise = ImuNoise(self.flight_config)
        noise.restore_state(flight["imu_noise"])
        _snapshot_time(noise.observation()["last_time_s"], "noise.last_time_s", time_s, optional=True)
        motion = copy.deepcopy(self.target_motion)
        motion.restore_state(base["target_motion"])
        if motion.mode != self.target_mode or motion.seed != self.target_seed:
            raise ValueError("Target program belongs to a different experiment")

    def restore_state(self, snapshot):
        """Validate first, restore base, then reattach the AP adapter and expert.

        The base intentionally constructs a plain FlightController. External
        guidance is stored separately and reattached only after its state has
        passed the same base-controller validator.
        """
        if (not isinstance(snapshot, Mapping) or isinstance(snapshot.get("traditional_snapshot_version"), bool)
                or snapshot.get("traditional_snapshot_version") != 1):
            raise ValueError("Unsupported traditional-control snapshot version")
        if snapshot.get("prediction_model") != self.prediction_model:
            raise ValueError("Snapshot nominal prediction model does not match")
        if snapshot.get("configuration_sha256") != self._configuration_hash():
            raise ValueError("Traditional snapshot configuration or physical world does not match")
        time_s = _snapshot_time(snapshot.get("time_s"), "time_s")
        try:
            saved = self._validated_adapter(snapshot.get("adapter"), time_s)
            self._validate_base_snapshot(snapshot.get("base"), time_s)
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            raise ValueError(f"Invalid traditional snapshot: {exc}") from exc
        previous_base = DemoSimulation.capture_state(self)
        previous_controller = self.flight_controller
        try:
            DemoSimulation.restore_state(self, snapshot["base"])
            if abs(self.t-time_s) > 1e-9:
                raise ValueError("Base integration time does not match snapshot time")
            controller = CommandAutopilot(self.flight_config)
            controller.restore_state(snapshot["base"]["flight_control"]["controller"])
            controller._external_body_g = saved["external_body_g"].copy()
            controller._external_time_s = saved["external_time_s"]
        except Exception as exc:
            # Restore even if a malformed base record failed after a partial
            # target/data assignment; retain the previous external AP object.
            DemoSimulation.restore_state(self, previous_base)
            self.flight_controller = previous_controller
            raise ValueError(f"Could not restore traditional physical state: {exc}") from exc
        self.flight_controller = controller
        self._pn_expert = saved["expert"]
        self._target_estimate_W = saved["target_estimate_W"]
        self._target_estimate_t = saved["target_estimate_t"]
        self._last_upper_visual_t = saved["last_upper_visual_t"]
        self._candidate_velocity_g = saved["velocity_request_g"]
        self._candidate_body_g = saved["body_request_g"]
        self._candidate_measured_total_g = saved["measured_total_normal_g"]
        self._candidate_axial_body_g = saved["omitted_axial_body_g"]
        self._candidate_update_t = saved["candidate_update_t"]
        self._expert_diagnostics = copy.deepcopy(saved["expert_diagnostics"])


TraditionalControlSimulation = PhysicalCandidate
