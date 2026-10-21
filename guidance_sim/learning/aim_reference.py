"""Convert a calibrated lamp-image reference to a fixed target-plane bias.

This adapter does not replace the real-camera lamp detector or the v2
lamp-to-target CAD transform.  Its zero-bias pixel preserves the previously
validated upper-target aim.  Moving the expected lamp position represents an
angular aim change at a DECLARED calibration view, converted to metres by an
explicit reference distance.  That distance is not a measured live range.

The map stays fixed through the high arc: it does not try to hold the lamp at
one pixel during every flight phase.  The supplied target normal must point
from the firing side into the target, as the launch-profile loader defines it.
"""
from __future__ import annotations

import copy
import json
import math
from pathlib import Path

import numpy as np


DEFAULT_CONFIG = Path(__file__).with_suffix(".json")
MODES = ("display_only", "image_reference")


def _pair(value, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (2,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain two finite numbers")
    return result.copy()


def _positive(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0.:
        raise ValueError(f"{name} must be positive and finite")
    return float(value)


class AimReference:
    """A round's static image-reference calibration, independent of truth.

    ``update(aim_point_px)`` returns the applied world offset in metres.
    ``observation`` explains both requested and applied offsets; optional
    original-target dimensions only produce flags and never clamp the goal.
    """

    def __init__(self, config=None, plane_normal_W=None, fovy_deg=55.):
        if config is None or isinstance(config, (str, Path)):
            config = json.loads(Path(DEFAULT_CONFIG if config is None else config)
                                .read_text(encoding="utf-8-sig"))
        if not isinstance(config, dict):
            raise ValueError("Aim reference configuration must be an object")
        allowed = {"schema_version", "mode", "image_size_px", "calibrated_aim_point_px",
                   "reference_distance_m", "description"}
        if set(config)-allowed or config.get("schema_version") != 1 or isinstance(config.get("schema_version"), bool):
            raise ValueError("Unknown aim reference schema or configuration field")
        if config.get("mode") not in MODES:
            raise ValueError(f"mode must be one of {MODES}")
        size = _pair(config.get("image_size_px"), "image_size_px")
        if np.any(size <= 0.) or np.any(size != np.floor(size)):
            raise ValueError("image_size_px must contain positive integers")
        self.config = copy.deepcopy(config)
        self.image_size_px = size
        self.reference_distance_m = _positive(config.get("reference_distance_m"), "reference_distance_m")
        self.fovy_deg = _positive(fovy_deg, "fovy_deg")
        if self.fovy_deg >= 180.:
            raise ValueError("fovy_deg must be less than 180 degrees")
        self.focal_px = float(size[1]/(2.*math.tan(math.radians(self.fovy_deg)/2.)))
        normal = np.asarray(plane_normal_W, dtype=float)
        if normal.shape != (3,) or not np.all(np.isfinite(normal)):
            raise ValueError("plane_normal_W must contain three finite numbers")
        scale = float(np.max(np.abs(normal)))
        if scale == 0.:
            raise ValueError("plane_normal_W must be nonzero")
        normal = normal/scale
        self.normal_W = normal/np.linalg.norm(normal)
        up = np.array([0., 0., 1.])-self.normal_W[2]*self.normal_W
        if np.linalg.norm(up) < 1e-10:
            raise ValueError("A horizontal target plane needs an explicit calibration-up axis")
        self.up_W = up/np.linalg.norm(up)
        self.right_W = np.cross(self.normal_W, self.up_W)
        self.calibrated_aim_point_px = self._pixel(config.get("calibrated_aim_point_px"))
        self.update(self.calibrated_aim_point_px)

    @property
    def mode(self):
        return self.config["mode"]

    def _pixel(self, value):
        result = _pair(value, "aim_point_px")
        if np.any(result < 0.) or np.any(result >= self.image_size_px):
            raise ValueError("aim_point_px must lie inside the calibrated image")
        return result

    def update(self, aim_point_px):
        pixel = self._pixel(aim_point_px)
        du, dv = pixel-self.calibrated_aim_point_px
        scale = self.reference_distance_m/self.focal_px
        # Image right is opposite camera left; image down is opposite camera up.
        # Expected lamp down means the requested aim direction moves up.
        bias = scale*(-du*self.right_W+dv*self.up_W)
        applied = bias if self.mode == "image_reference" else np.zeros(3)
        self.aim_point_px = pixel
        self.calibrated_bias_W_m = bias
        self.offset_W_m = applied.copy()
        return self.offset_W_m.copy()

    def observation(self, *, target_radius_m=None, target_half_extents_m=None):
        result = dict(mode=self.mode,
            semantics="desired_green_lamp_pixel_at_declared_calibration_view",
            calibrated_aim_point_px=self.calibrated_aim_point_px.tolist(),
            aim_point_px=self.aim_point_px.tolist(),
            image_size_px=self.image_size_px.astype(int).tolist(),
            fovy_deg=self.fovy_deg, focal_px=self.focal_px,
            reference_distance_m=self.reference_distance_m,
            distance_basis="declared_fixed_angular_to_metric_scale_not_measured_range",
            calibrated_bias_W_m=self.calibrated_bias_W_m.tolist(),
            offset_W_m=self.offset_W_m.tolist(),
            offset_plane_right_up_m=[float(self.offset_W_m @ self.right_W),
                                   float(self.offset_W_m @ self.up_W)],
            original_target_evaluation_unchanged=True, goal_clamped=False)
        if target_radius_m is not None:
            radius = _positive(target_radius_m, "target_radius_m")
            result.update(target_radius_m=radius,
                outside_original_target_radius=bool(np.linalg.norm(self.offset_W_m) > radius))
        if target_half_extents_m is not None:
            extents = _pair(target_half_extents_m, "target_half_extents_m")
            if np.any(extents <= 0.):
                raise ValueError("target_half_extents_m must be positive")
            result.update(target_half_extents_m=extents.tolist(),
                outside_original_target_plate=bool(np.any(
                    np.abs(result["offset_plane_right_up_m"]) > extents)))
        return result

    def capture_state(self):
        return dict(schema_version=1, config=copy.deepcopy(self.config),
            plane_normal_W=self.normal_W.tolist(), fovy_deg=self.fovy_deg,
            aim_point_px=self.aim_point_px.tolist())

    def restore_state(self, state):
        if not isinstance(state, dict) or state.get("schema_version") != 1 or isinstance(state.get("schema_version"), bool):
            raise ValueError("Invalid aim reference state schema")
        if state.get("config") != self.config or state.get("plane_normal_W") != self.normal_W.tolist() or state.get("fovy_deg") != self.fovy_deg:
            raise ValueError("Aim reference state calibration does not match")
        self.update(state.get("aim_point_px"))
