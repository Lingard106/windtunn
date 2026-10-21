"""Collect real-camera PN expert demonstrations; no neural-network training.

Each row is captured immediately BEFORE an expert decision, at the simulator's
actual control boundary. Labels are the bounded pitch/yaw *specific-force
requests*, not surface angles or already-realized forces. Episodes, rather than
individual rows, belong to mutually exclusive train/validation/test splits.
Privileged simulator state is confined to scenario metadata and ``eval_*``
arrays. Failed episodes remain inspectable with their training masks disabled.
"""
from __future__ import annotations

import argparse
import copy
from dataclasses import replace
from datetime import datetime, timezone
import hashlib
from importlib.metadata import PackageNotFoundError, version
import json
import math
from pathlib import Path
import platform
import sys
import time
from uuid import uuid4

import numpy as np

PROJECT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = Path(__file__).with_name("config.json")
SPLITS = ("train", "validation", "test")
AGE_CLIP_S = 1.
FEATURE_GROUPS = (
    ("estimated_goal_relative_L", ("x", "y", "z"), "m"),
    ("own_velocity_W", ("x", "y", "z"), "m/s"),
    ("rotation_WL", tuple(f"r{r}{c}" for r in range(3) for c in range(3)), "1"),
    ("actual_control", ("pitch", "yaw"), "g"),
    ("previous_command", ("pitch", "yaw"), "g"),
    ("latest_detected_ray_L", ("x", "y", "z"), "1"),
    ("latest_frame_valid", ("value",), "bool"),
    ("latest_frame_locked", ("value",), "bool"),
    ("detection_age", ("value",), "s_clipped_1"),
    ("estimated_goal_valid", ("value",), "bool"),
    ("estimated_goal_age", ("value",), "s_clipped_1"),
    ("elapsed_time", ("value",), "s"),
    ("declared_goal_offset_W", ("x", "y", "z"), "m"),
)
FEATURE_NAMES = [f"{group}_{axis}" for group, axes, _ in FEATURE_GROUPS for axis in axes]
FEATURE_UNITS = [unit for _, axes, unit in FEATURE_GROUPS for _ in axes]


def _write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2,
                                   allow_nan=False)+"\n", encoding="utf-8")


def _positive(value, name, *, zero=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or (value < 0 if zero else value <= 0):
        raise ValueError(f"{name} must be {'nonnegative' if zero else 'positive'} and finite")
    return float(value)


def load_collection_config(path=DEFAULT_CONFIG):
    value = json.loads(Path(path).read_text(encoding="utf-8-sig"))
    expected = {"schema_version", "master_seed", "episodes", "duration_s", "camera_hz",
                "goal_tolerance_m", "expert_config", "aim_reference_config",
                "vision_config", "randomization"}
    if not isinstance(value, dict) or set(value) != expected or value["schema_version"] != 1:
        raise ValueError("Unsupported dataset collection configuration")
    seed = value["master_seed"]
    if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
        raise ValueError("master_seed must be a nonnegative integer")
    counts = value["episodes"]
    if not isinstance(counts, dict) or set(counts) != set(SPLITS):
        raise ValueError("episodes must contain train, validation and test")
    for split, count in counts.items():
        if isinstance(count, bool) or not isinstance(count, int) or count < 0:
            raise ValueError(f"episodes.{split} must be a nonnegative integer")
    if counts["train"] < 1:
        raise ValueError("At least one training episode is required")
    for name in ("duration_s", "camera_hz", "goal_tolerance_m"):
        value[name] = _positive(value[name], name)
    ranges = value["randomization"]
    keys = {"position_xyz_half_width_m", "wind_cross_half_width_mps",
            "yaw_half_width_deg", "pitch_half_width_deg", "speed_half_width_mps",
            "reference_half_width_px"}
    if not isinstance(ranges, dict) or set(ranges) != keys:
        raise ValueError("Unsupported randomization fields")
    position = ranges["position_xyz_half_width_m"]
    if not isinstance(position, list) or len(position) != 3:
        raise ValueError("position_xyz_half_width_m must have three entries")
    ranges["position_xyz_half_width_m"] = [_positive(v, "position range", zero=True) for v in position]
    reference_width = ranges["reference_half_width_px"]
    if (not isinstance(reference_width, list) or len(reference_width) != 2 or
            any(isinstance(v, bool) or not isinstance(v, int) or v < 0 for v in reference_width)):
        raise ValueError("reference_half_width_px must contain two nonnegative integers")
    for key in keys-{"position_xyz_half_width_m", "reference_half_width_px"}:
        ranges[key] = _positive(ranges[key], key, zero=True)
    for key in ("expert_config", "vision_config", "aim_reference_config"):
        if not isinstance(value[key], str) or not value[key]:
            raise ValueError(f"{key} must name a configuration file")
    return value


def episode_plan(config):
    """Stable per-episode seeds: split/count changes do not reuse one seed."""
    bounds = config["randomization"]
    reference = json.loads((PROJECT/config["aim_reference_config"]).read_text(encoding="utf-8-sig"))
    baseline = np.asarray(reference.get("calibrated_aim_point_px"), dtype=float)
    image_size = np.asarray(reference.get("image_size_px"), dtype=float)
    widths = np.array(bounds["reference_half_width_px"])
    if (reference.get("mode") != "image_reference" or baseline.shape != (2,) or
            image_size.shape != (2,) or not np.isfinite(baseline).all() or
            not np.isfinite(image_size).all() or np.any(baseline != np.floor(baseline)) or
            np.any(baseline-widths < 0) or np.any(baseline+widths >= image_size)):
        raise ValueError("Calibrated reference and full reference-jitter range must fit the image")
    plan = []
    for split_index, split in enumerate(SPLITS):
        for index in range(config["episodes"][split]):
            seed_sequence = np.random.SeedSequence(config["master_seed"],
                                                   spawn_key=(split_index, index))
            seed = int(seed_sequence.generate_state(1, dtype=np.uint64)[0])
            rng = np.random.default_rng(seed)
            offset = rng.uniform(-1., 1., 3)*np.array(bounds["position_xyz_half_width_m"])
            reference_rng = np.random.default_rng(np.random.SeedSequence(seed, spawn_key=(99,)))
            reference_delta = reference_rng.integers(-widths, widths+1)
            plan.append(dict(id=f"{split}_{index:05d}", split=split, seed=seed,
                aim_point_px=(baseline.astype(int)+reference_delta).tolist(),
                reference_delta_px=reference_delta.tolist(),
                position_offset_W_m=offset.tolist(),
                wind_cross_mps=float(rng.uniform(-1., 1.)*bounds["wind_cross_half_width_mps"]),
                yaw_offset_deg=float(rng.uniform(-1., 1.)*bounds["yaw_half_width_deg"]),
                pitch_offset_deg=float(rng.uniform(-1., 1.)*bounds["pitch_half_width_deg"]),
                speed_offset_mps=float(rng.uniform(-1., 1.)*bounds["speed_half_width_mps"])))
    seeds = [case["seed"] for case in plan]
    if len(seeds) != len(set(seeds)):
        raise RuntimeError("Episode seeds unexpectedly overlap")
    return plan


def source_hashes(extra_paths=()):
    files = [p for directory in ("prototypes", "pn_expert_v2", "learning")
             for p in (PROJECT/directory).glob("*.py")]
    files += [PROJECT/"learning/config.json", *(Path(p) for p in extra_paths if p is not None)]
    result = {}
    for path in sorted(set(files)):
        path = path.resolve()
        name = path.relative_to(PROJECT).as_posix() if path.is_relative_to(PROJECT) else str(path)
        result[name] = hashlib.sha256(path.read_bytes()).hexdigest()
    return result


def runtime_versions():
    result = dict(python=platform.python_version(), implementation=platform.python_implementation(),
                  platform=platform.platform())
    for package in ("numpy", "mujoco", "opencv-python", "Pillow"):
        try:
            result[package] = version(package)
        except PackageNotFoundError:
            result[package] = "unavailable"
    return result


def _age(t, timestamp):
    if timestamp is None:
        return AGE_CLIP_S
    if timestamp > t+1e-9:
        raise RuntimeError("A future observation entered a pre-action feature")
    return min(AGE_CLIP_S, max(0., float(t-timestamp)))


def pack_features(sim, visual, previous_command):
    """Only own-state channels and causal, camera-derived target estimates."""
    t = float(sim.t)
    position = np.array(sim.data.qpos[:3], copy=True)
    rotation = np.array(sim.data.xmat[sim.body_id], copy=True).reshape(3, 3)
    estimate = sim.guidance_target_estimate_W_m
    estimate_age = _age(t, sim._target_estimate_t)
    estimate_valid = (estimate is not None and sim._target_estimate_t is not None and
                      t-sim._target_estimate_t <= sim.flight_config["controller"]["vision_timeout_s"]+1e-9)
    relative = np.zeros(3) if estimate is None else rotation.T@(np.asarray(estimate)-position)
    # The public property includes the selected target-reference convention.
    offset = np.asarray(sim.guidance_goal_offset_W_m, dtype=float)
    if offset.shape != (3,) or not np.isfinite(offset).all():
        raise RuntimeError("Invalid declared guidance-goal offset")
    detection_age = _age(t, visual["last_detection_t"])
    _age(t, visual["sample_t"])
    vector = np.r_[relative, sim.data.qvel[:3], rotation.ravel(),
        sim._ideal_actual_g, previous_command, visual["last_detected_ray_L"],
        float(visual["valid"]), float(visual["locked"]), detection_age,
        float(estimate_valid), estimate_age, t, offset]
    if vector.shape != (len(FEATURE_NAMES),) or not np.isfinite(vector).all():
        raise RuntimeError("Malformed or nonfinite pre-action feature")
    return vector


def _restart_episode(sim, baseline, numeric, case):
    changed = dict(numeric,
        speed_mps=numeric["speed_mps"]+case["speed_offset_mps"],
        yaw_deg=numeric["yaw_deg"]+case["yaw_offset_deg"],
        pitch_deg=numeric["pitch_deg"]+case["pitch_offset_deg"],
        wind_cross_mps=case["wind_cross_mps"])
    sim.restart_episode(baseline, changed,
                        position_offset_W=case["position_offset_W_m"])


def collect_episode(sim, baseline, numeric, settings, case, config, output):
    """Render serially, recording expert decisions before their first action."""
    from flight_sensors import sample_flight_vision
    from openmv_tracker import OpenMVTracker
    from playback import SampleClock

    _restart_episode(sim, baseline, numeric, case)
    case_settings = replace(settings, aim_point_px=tuple(case["aim_point_px"]))
    tracker = OpenMVTracker(case_settings)
    clock = SampleClock(config["camera_hz"])
    clock.reset(sim.t)
    visual = dict(valid=False, locked=False, sample_t=None, last_detection_t=None,
                  last_detected_ray_L=np.zeros(3))
    rows, times, labels, statuses, sample_valid, update_events = [], [], [], [], [], []
    eval_times, eval_nose, eval_position, eval_velocity, eval_actual = [], [], [], [], []
    frames = detections = accepted = 0
    maximum = float(sim.dynamics_config["max_control_g"])
    expert_period = float(sim.expert_config["update_period_s"])
    original_advance = sim._advance_expert
    last_record_t = -math.inf
    begin = time.monotonic()

    def advance_and_record():
        nonlocal last_record_t
        previous_command = sim._limited_command_g().copy()
        features = pack_features(sim, visual, previous_command)
        previous_update = sim._pn_expert._last_update_s
        original_advance()
        current_update = sim._pn_expert._last_update_s
        event = current_update is not None and current_update != previous_update
        if not event and sim.t-last_record_t < expert_period-1e-9:
            return
        label = sim._limited_command_g().copy()
        status = sim._expert_diagnostics.get("status", "unknown")
        valid = bool(event and status == "active" and np.isfinite(label).all() and
                     np.linalg.norm(label) <= maximum+1e-9)
        rows.append(features)
        times.append(float(sim.t))
        labels.append(label)
        statuses.append(status)
        sample_valid.append(valid)
        update_events.append(event)
        last_record_t = float(sim.t)

    def sample():
        nonlocal frames, detections, accepted
        details = sample_flight_vision(sim, tracker)
        measurement = sim._last_visual_measurement or {}
        visual.update(valid=bool(measurement.get("valid", False)),
                      locked=bool(details.get("locked", False)), sample_t=sim.t)
        frames += 1
        detections += int(visual["valid"])
        accepted += int(measurement.get("upper_target_measurement_accepted", False))
        if visual["valid"]:
            visual["last_detection_t"] = float(measurement["t_s"])
            visual["last_detected_ray_L"] = np.array(measurement["los_ray_L"], dtype=float)

    def evaluation_sample():
        eval_times.append(float(sim.t))
        eval_nose.append(sim.nose_position_W().copy())
        eval_position.append(sim.data.qpos[:3].copy())
        eval_velocity.append(sim.data.qvel[:3].copy())
        eval_actual.append(sim._ideal_actual_g.copy())

    sim._advance_expert = advance_and_record
    try:
        sample()
        evaluation_sample()
        while sim.t < config["duration_s"]-1e-12 and not sim.done:
            sim.step()
            if clock.consume(sim.t):
                sample()
            evaluation_sample()
    finally:
        sim._advance_expert = original_advance
    x = np.asarray(rows, dtype=np.float32).reshape(-1, len(FEATURE_NAMES))
    y = np.asarray(labels, dtype=np.float32).reshape(-1, 2)
    t = np.asarray(times, dtype=np.float64)
    valid = np.asarray(sample_valid, dtype=bool)
    result = copy.deepcopy(sim.target_result)
    goal_result = copy.deepcopy(sim.evaluate_guidance_goal())
    hit = bool(result is not None and result.get("hit", False))
    goal_pass = bool(goal_result is not None and
                     math.isfinite(goal_result["selected_goal_miss_m"]) and
                     goal_result["selected_goal_miss_m"] <= config["goal_tolerance_m"])
    finite = bool(np.isfinite(x).all() and np.isfinite(y).all() and
                  np.isfinite(np.asarray(eval_position)).all())
    actual = np.asarray(eval_actual)
    differences = np.diff(np.asarray(eval_times))
    slew = np.linalg.norm(np.diff(actual, axis=0), axis=1)/differences if len(actual) > 1 else np.zeros(1)
    max_actual = float(np.max(np.linalg.norm(actual, axis=1)))
    max_slew = float(np.max(slew))
    constraints = bool(finite and max_actual <= maximum+1e-9 and
                       max_slew <= sim.dynamics_config["max_slew_g_s"]+1e-8)
    episode_trusted = bool(hit and goal_pass and constraints and valid.any())
    # A failed episode is retained, but cannot silently become trusted SL data.
    trusted = valid & episode_trusted
    target = sim.launch_profile
    destination = output/case["split"]/(case["id"]+".npz")
    destination.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(destination, X=x, y_command_g=y, t_s=t,
        sample_valid=valid, trusted_training_mask=trusted,
        expert_update_event=np.asarray(update_events, dtype=bool),
        expert_status=np.asarray(statuses, dtype="U40"),
        eval_t_s=np.asarray(eval_times), eval_nose_W_m=np.asarray(eval_nose),
        eval_position_W_m=np.asarray(eval_position), eval_velocity_W_mps=np.asarray(eval_velocity),
        eval_actual_control_g=actual,
        eval_target_center_W_m=np.asarray(target.target_position_W_m),
        eval_target_normal_W=np.asarray(target.target_normal_W),
        eval_target_radius_m=np.array(target.target_radius_m),
        eval_selected_goal_W_m=(np.full(3, np.nan) if goal_result is None else
                               np.asarray(goal_result["selected_goal_W_m"])),
        aim_point_px=np.array(case["aim_point_px"], dtype=int))
    return dict(**case, file=destination.relative_to(output).as_posix(),
        sample_count=len(t), valid_samples=int(valid.sum()), trusted_samples=int(trusted.sum()),
        episode_trusted=episode_trusted, hit=hit, constraints_pass=constraints,
        status=sim.status, final_time_s=float(sim.t), target_result=result,
        selected_goal_result=goal_result, selected_goal_pass=goal_pass,
        aim_reference=copy.deepcopy(sim.aim_reference_observation),
        max_actual_control_g=max_actual, max_actual_slew_g_s=max_slew,
        camera_frames=frames, detected_frames=detections, accepted_target_frames=accepted,
        wall_s=time.monotonic()-begin,
        sha256=hashlib.sha256(destination.read_bytes()).hexdigest())


def collect(config, output):
    if str(PROJECT/"prototypes") not in sys.path:
        sys.path.insert(0, str(PROJECT/"prototypes"))
    from dart_demo import load_config
    from ideal_guidance_sim import IdealSimulation
    from simulation_setup import resolve_startup_paths
    from vision_settings import VisionSettings

    paths = resolve_startup_paths()
    expert_path = PROJECT/config["expert_config"]
    vision_path = PROJECT/config["vision_config"]
    reference_path = PROJECT/config["aim_reference_config"]
    reference_config = json.loads(reference_path.read_text(encoding="utf-8-sig"))
    settings = VisionSettings(**json.loads(vision_path.read_text(encoding="utf-8-sig")))
    profile = json.loads(expert_path.read_text(encoding="utf-8-sig"))
    if profile.get("expert", {}).get("law") != "ballistic_pn":
        raise ValueError("Collection requires the verified ballistic PN expert profile")
    hashed_inputs = (expert_path, vision_path, reference_path, paths.config, paths.launch)
    hashes = source_hashes(hashed_inputs)
    plan = episode_plan(config)
    if output.exists() and any(output.iterdir()):
        raise FileExistsError("Dataset output is nonempty; choose a fresh run directory")
    output.mkdir(parents=True, exist_ok=True)
    manifest = dict(schema_version=1, collection_status="in_progress",
        created_utc=datetime.now(timezone.utc).isoformat(), collection_config=copy.deepcopy(config),
        runtime_versions=runtime_versions(),
        self_state_assumptions=dict(pose="perfect own position and orientation",
            velocity="perfect own inertial velocity", actual_control="perfect realized control-force state",
            imu_noise_enabled=False, target="real rendered-camera recognition plus declared static CAD prior",
            unknown_true_wind="not provided to expert or features"),
        features=dict(names=FEATURE_NAMES, units=FEATURE_UNITS, size=len(FEATURE_NAMES),
            dtype="float32", missing_target_relative="zeros until first camera estimate",
            missing_ray="zeros until first valid detection", age_clip_s=AGE_CLIP_S,
            privileges="perfect own pose/velocity and force state; measured detector ray and estimated goal; declared static map/offset prior",
            forbidden=["true wind", "true target position", "future state", "expert future predictions"]),
        labels=dict(names=["pitch_specific_force_request_g", "yaw_specific_force_request_g"],
            sign="pitch down/yaw right positive; gravity excluded", bounded_norm_g=profile["reduced_order"]["max_control_g"],
            timing="X immediately before same-time expert update, y after update and before action",
            sample_valid="fresh active expert update, finite bounded command",
            trusted_training_mask="sample_valid AND original physical target hit AND selected-goal error <= goal_tolerance_m AND episode dynamics constraints",
            failed_episodes="retained with zero trusted mask; unavailable/terminal rows also retained"),
        split_policy="entire independently seeded flights; never random row splitting",
        normalization="none; any future scaler must fit training split only",
        goal_tolerance_m=config["goal_tolerance_m"], aim_reference_config=reference_config,
        vision_reference_policy="Each episode uses calibrated_aim_point_px plus independent integer jitter; operator vision file remains untouched; actual point is in episode metadata",
        vision_settings=settings.as_dict(), expert_profile=profile,
        source_hashes_before=hashes, episode_plan=plan, episodes=[])
    _write_json(output/"manifest.json", manifest)
    sim = None
    try:
        sim = IdealSimulation(load_config(paths.config), paths.scene, paths.dart, paths.launch,
            guidance_source="vision", flight_control_config=expert_path,
            aim_reference_config=reference_path)
        if not hasattr(sim, "restart_episode") or not hasattr(sim, "guidance_goal_offset_W_m"):
            raise RuntimeError("Simulator target-reference/restart API is unavailable")
        numeric = dict(sim.cfg, **paths.explicit_values)
        numeric["t_max_s"] = max(numeric["t_max_s"], config["duration_s"])
        sim.reset(numeric)
        baseline = sim.capture_state()
        manifest.update(numeric_baseline=copy.deepcopy(numeric),
            flight_model=sim.flight_control_metadata, launch=sim.launch_metadata)
        for index, case in enumerate(plan):
            row = collect_episode(sim, baseline, numeric, settings, case, config, output)
            manifest["episodes"].append(row)
            _write_json(output/"manifest.json", manifest)
            print(json.dumps(dict(episode=f"{index+1}/{len(plan)}", id=case["id"],
                hit=row["hit"], samples=row["sample_count"], trusted=row["trusted_samples"],
                miss_m=None if row["target_result"] is None else row["target_result"]["miss_distance_m"],
                selected_goal_miss_m=None if row["selected_goal_result"] is None else row["selected_goal_result"]["selected_goal_miss_m"],
                wall_s=round(row["wall_s"], 2)), ensure_ascii=False), flush=True)
        manifest["source_hashes_after"] = source_hashes(hashed_inputs)
        unchanged = hashes == manifest["source_hashes_after"]
        manifest["sources_unchanged"] = unchanged
        manifest["collection_status"] = "complete" if unchanged else "invalid_sources_changed"
        reports = {}
        for split in SPLITS:
            episodes = [row for row in manifest["episodes"] if row["split"] == split]
            misses = [row["target_result"]["miss_distance_m"] for row in episodes if row["target_result"] is not None]
            goal_misses = [row["selected_goal_result"]["selected_goal_miss_m"] for row in episodes if row["selected_goal_result"] is not None]
            reports[split] = dict(episodes=len(episodes), hits=sum(row["hit"] for row in episodes),
                selected_goal_passes=sum(row["selected_goal_pass"] for row in episodes),
                trusted_episodes=sum(row["episode_trusted"] for row in episodes),
                samples=sum(row["sample_count"] for row in episodes),
                trusted_samples=sum(row["trusted_samples"] for row in episodes),
                max_miss_m=max(misses) if misses else None,
                p95_miss_m=float(np.quantile(misses, .95)) if misses else None,
                max_selected_goal_miss_m=max(goal_misses) if goal_misses else None,
                p95_selected_goal_miss_m=float(np.quantile(goal_misses, .95)) if goal_misses else None)
        report = dict(schema_version=1, collection_status=manifest["collection_status"],
            episode_count=len(plan), sample_count=sum(row["sample_count"] for row in manifest["episodes"]),
            trusted_sample_count=sum(row["trusted_samples"] for row in manifest["episodes"]),
            splits=reports, sources_unchanged=unchanged,
            limitations="Local fixed-target envelope, perfect self-state, static CAD prior, constant unknown crosswind, assumed control authority; no NN has been trained.")
        manifest["summary"] = report
        _write_json(output/"manifest.json", manifest)
        if not unchanged:
            raise RuntimeError("Sources/configurations changed during collection; this run is invalid")
        return report
    except BaseException as exc:
        if manifest["collection_status"] == "in_progress":
            manifest["collection_status"] = "interrupted_or_failed"
        manifest["error"] = f"{type(exc).__name__}: {exc}"
        _write_json(output/"manifest.json", manifest)
        raise
    finally:
        if sim is not None:
            sim.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path,
        help="Fresh dataset directory; defaults to outputs/expert_dataset/<unique run>")
    parser.add_argument("--counts", type=int, nargs=3, metavar=("TRAIN", "VALIDATION", "TEST"))
    parser.add_argument("--seed", type=int)
    parser.add_argument("--dry-run", action="store_true", help="Validate configuration and show planned flights without rendering")
    args = parser.parse_args(argv)
    config = load_collection_config(args.config)
    if args.counts is not None:
        if min(args.counts) < 0 or args.counts[0] < 1:
            parser.error("--counts requires TRAIN >= 1 and other counts >= 0")
        config["episodes"] = dict(zip(SPLITS, args.counts))
    if args.seed is not None:
        if args.seed < 0:
            parser.error("--seed must be nonnegative")
        config["master_seed"] = args.seed
    plan = episode_plan(config)
    if args.dry_run:
        print(json.dumps(dict(config=config, feature_count=len(FEATURE_NAMES),
            episode_count=len(plan), episode_plan=plan), ensure_ascii=False, indent=2))
        return 0
    name = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")+"_"+uuid4().hex[:6]
    output = args.output or PROJECT/"outputs/expert_dataset"/name
    result = collect(config, output.resolve())
    print(json.dumps(dict(output=str(output.resolve()), **result), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
