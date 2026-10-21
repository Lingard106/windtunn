"""Physical classical-control viewer and reproducible real-camera evaluation.

The original v1 entry is unchanged. This entry keeps its complete physical plant
and substitutes a visual ballistic-ZEM guidance request plus physical overload PID.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import random
import sys
import time

import numpy as np

PROJECT = Path(__file__).resolve().parents[1]
for directory in (PROJECT, PROJECT / "prototypes"):
    if str(directory) not in sys.path:
        sys.path.insert(0, str(directory))

from dart_demo import DemoSimulation, load_config
from field_scene import available_scenes
from flight_sensors import sample_flight_vision
from openmv_tracker import OpenMVTracker
from playback import SampleClock
from simulation_setup import resolve_startup_paths
from target_motion import TARGET_MODES
from vision_settings import VisionSettings
from traditional_control.simulation import PhysicalCandidate
from traditional_control.vision import ReacquiringTracker

RANDOMIZATION_LEVELS = {
    "match": dict(mass_fraction=.01, speed_mps=.1, roll_deg=1., drag_fraction=.05),
    "small": dict(mass_fraction=.02, speed_mps=.2, roll_deg=2., drag_fraction=.1),
    "large": dict(mass_fraction=.05, speed_mps=.5, roll_deg=5., drag_fraction=.2),
}


def tolerance_sample(seed, level, master_seed=91003):
    """Evaluation-only bounded independent uniform draws, identical for variants.

    Always draw all four values so single-factor/combined comparisons are paired.
    No perturbation sample is submitted to a controller or predictor.
    """
    bounds = RANDOMIZATION_LEVELS[level]
    rng = random.Random(f"dart-tolerance-v1:{master_seed}:{seed}")
    return {key: rng.uniform(-bound, bound) for key, bound in bounds.items()}


def summarize(runs):
    n = len(runs)
    hits = sum(row["hit"] for row in runs)
    p = hits / max(n, 1)
    z = 1.95996398454
    denominator = 1 + z*z/max(n, 1)
    center = (p + z*z/(2*max(n, 1))) / denominator
    half = z*math.sqrt(p*(1-p)/max(n, 1) + z*z/(4*max(n, 1)**2))/denominator
    result = dict(n=n, hits=hits, hit_rate=p,
                  wilson95=[max(0., center-half), min(1., center+half)],
                  errors=sum("error" in row for row in runs),
                  no_event=sum(not row.get("target_contact") and not row.get("target_crossing") for row in runs))
    for key, event_key, distance_key in (("contact", "target_contact", "in_plane_distance_m"),
                                          ("crossing", "target_crossing", "miss_distance_m")):
        distances = [row[event_key][distance_key] for row in runs if row.get(event_key)]
        result[key] = dict(n=len(distances),
            mean_distance_m=float(np.mean(distances)) if distances else None,
            p95_distance_m=float(np.quantile(distances, .95)) if distances else None)
    return result


def fly(simulation, settings, duration, *, trace=False, tracker_factory=ReacquiringTracker):
    """Real image sampling. Evaluation truth below is never sent to guidance."""
    tracker = tracker_factory(settings)
    clock = SampleClock(90.)
    frames = detected = locked = 0
    first_lock = None
    max_fin = saturation = steps = max_total_g = 0
    first_hit = None
    rows = []

    def sample():
        nonlocal frames, detected, locked, first_lock
        details = sample_flight_vision(simulation, tracker)
        frames += 1
        detected += bool(details.get("detected"))
        locked += bool(details.get("locked"))
        if details.get("locked") and first_lock is None:
            first_lock = simulation.t

    sample()
    while simulation.t < duration-1e-12 and not simulation.done:
        observation = simulation.step()
        steps += 1
        if clock.consume(simulation.t):
            sample()
        flight = observation["flight_control"]
        actual = np.asarray(flight["servo_actual_deg"])
        max_fin = max(max_fin, float(np.max(np.abs(actual))))
        saturation += bool(np.any(np.abs(actual) >= 29.5))
        if (simulation.target_contact is None and simulation.target_result is None
                and simulation.data.ncon == 0 and observation["p_W"][2] > .15):
            measured = flight["actual_overload_g"]
            max_total_g = max(max_total_g, math.hypot(measured["pitch"], measured["yaw"]))
        if trace:
            flight = simulation.observation()["flight_control"]
            rows.append(dict(t_s=simulation.t, p_W_m=observation["p_W"].tolist(),
                             v_W_mps=observation["v_W"].tolist(), flight_control=flight))
        events = (simulation.target_contact, simulation.target_result)
        if any(event and event.get("hit") for event in events) and first_hit is None:
            first_hit = simulation.t
        # Failed rounds keep running to the same horizon; native contact may
        # follow a failed nose crossing. A valid hit is already irreversible.
        if first_hit is not None and simulation.t >= first_hit+.05:
            break
    contact = copy.deepcopy(simulation.target_contact)
    crossing = copy.deepcopy(simulation.target_result)
    return dict(seed=simulation.target_seed, target_mode=simulation.target_mode,
        hit=any(event and event.get("hit") for event in (contact, crossing)),
        target_contact=contact, target_crossing=crossing, simulated_s=simulation.t,
        status=simulation.status, first_lock_s=first_lock, vision_frames=frames,
        detected_frames=detected, locked_frames=locked, max_fin_deg=max_fin,
        fin_saturation_fraction=saturation/max(steps, 1), max_free_flight_lateral_g=max_total_g,
        numeric_config=copy.deepcopy(simulation.cfg), flight_config=copy.deepcopy(simulation.flight_config),
        metadata=simulation.flight_control_metadata,
        tracker_metadata=tracker.metadata() if hasattr(tracker, "metadata") else {"model":"OpenMVTracker_original"},
        final_flight_control=simulation.observation()["flight_control"]), rows


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, help="Numeric/physical overrides JSON")
    parser.add_argument("--scene", type=Path)
    parser.add_argument("--dart", type=Path)
    parser.add_argument("--launch", type=Path)
    parser.add_argument("--flight-config", type=Path, default=Path(__file__).with_name("config.json"))
    parser.add_argument("--vision-config", type=Path, default=PROJECT/"profiles/vision_desktop.json")
    parser.add_argument("--target-mode", choices=tuple(TARGET_MODES), default="random_moving")
    parser.add_argument("--target-seed", type=int, default=2027)
    parser.add_argument("--seeds", type=int, nargs="+", help="Headless batch, including every seed in denominator")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--compare-v1", action="store_true", help="Paired original-v1 control with the same seed and repaired real-vision tracker")
    parser.add_argument("--duration", type=float, default=2.6)
    parser.add_argument("--output", type=Path, help="Explicit headless result directory; defaults to no files")
    parser.add_argument("--trace", action="store_true", help="Write detailed per-step JSON with --headless and --output")
    parser.add_argument("--auto-run", action="store_true")
    parser.add_argument("--auto-close", type=float)
    parser.add_argument("--flight-control", choices=("on", "off"), default="on")
    parser.add_argument("--wind-cross-mps", type=float)
    parser.add_argument("--yaw-offset-deg", type=float, default=0.)
    parser.add_argument("--pitch-offset-deg", type=float, default=0.)
    parser.add_argument("--speed-offset-mps", type=float, default=0.)
    parser.add_argument("--imu-noise-scale", type=float)
    parser.add_argument("--roll-deg", type=float, default=0., help="Initial body roll; launch velocity direction is unchanged")
    parser.add_argument("--randomize", choices=("none", *RANDOMIZATION_LEVELS), default="none",
                        help="Bounded mass/speed/release-roll/Cd uncertainty, nominal predictor stays fixed")
    parser.add_argument("--randomization-seed", type=int, default=91003)
    parser.add_argument("--randomization-components", choices=("mass", "speed", "roll", "drag"),
                        nargs="+", default=["mass", "speed", "roll", "drag"])
    args = parser.parse_args(argv)
    for key in ("duration", "auto_close", "wind_cross_mps", "yaw_offset_deg", "pitch_offset_deg",
                "speed_offset_mps", "imu_noise_scale", "roll_deg"):
        value = getattr(args, key)
        if value is not None and (not math.isfinite(value) or
            (key in ("duration", "auto_close") and value <= 0) or
            (key == "imu_noise_scale" and value < 0)):
            parser.error(f"Invalid --{key.replace('_', '-')}")
    if (args.seeds or args.compare_v1 or args.trace) and not args.headless:
        parser.error("--seeds, --compare-v1 and --trace require --headless")
    if args.output is not None and not args.headless:
        parser.error("--output requires --headless")
    if args.trace and args.output is None:
        parser.error("--trace requires --output")
    paths = resolve_startup_paths(args.config, args.scene or available_scenes()["2027"], args.dart, args.launch)
    settings = VisionSettings(**json.loads(args.vision_config.read_text(encoding="utf-8-sig")))

    def create(seed, baseline=False):
        cls = DemoSimulation if baseline else PhysicalCandidate
        profile = PROJECT/"profiles/flight_control_v1_direct_2027.json" if baseline else args.flight_config
        sim = cls(load_config(paths.config), paths.scene, paths.dart, paths.launch,
            target_mode=args.target_mode, target_seed=seed,
            flight_control_enabled=args.flight_control == "on", flight_control_config=profile)
        try:
            numeric = dict(sim.cfg, **paths.explicit_values)
            numeric["yaw_deg"] += args.yaw_offset_deg
            numeric["pitch_deg"] += args.pitch_offset_deg
            numeric["speed_mps"] += args.speed_offset_mps
            if args.roll_deg != 0.:
                numeric["roll_deg"] = args.roll_deg
            if args.randomize != "none":
                perturbation = tolerance_sample(seed, args.randomize, args.randomization_seed)
                components = set(args.randomization_components)
                if "mass" in components:
                    numeric["mass_kg"] *= 1+perturbation["mass_fraction"]
                if "speed" in components:
                    numeric["speed_mps"] += perturbation["speed_mps"]
                if "roll" in components:
                    numeric["roll_deg"] = args.roll_deg+perturbation["roll_deg"]
                if "drag" in components:
                    numeric["drag_coefficient"] *= 1+perturbation["drag_fraction"]
            numeric["t_max_s"] = max(numeric["t_max_s"], args.duration)
            if args.wind_cross_mps is not None:
                numeric["wind_cross_mps"] = args.wind_cross_mps
            sim.reset(numeric)
            if args.imu_noise_scale is not None:
                sim.set_imu_noise(args.imu_noise_scale > 0, args.imu_noise_scale)
            return sim
        except Exception:
            sim.close()
            raise

    if not args.headless:
        import tkinter as tk
        from traditional_control.viewer import PhysicalTraditionalViewer
        sim = create(args.target_seed)
        root = None
        viewer = None
        try:
            root = tk.Tk()
            viewer = PhysicalTraditionalViewer(root, sim, auto_run=args.auto_run, vision_settings=settings)
            if args.auto_close is not None:
                root.after(round(args.auto_close*1000), viewer.close)
            root.mainloop()
        finally:
            if viewer is not None:
                viewer.close()
            else:
                sim.close()
                if root is not None:
                    root.destroy()
        return 0

    seeds = args.seeds or [args.target_seed]
    if len(seeds) != len(set(seeds)):
        parser.error("Duplicate seeds are not independent evaluation rounds")
    if args.output is not None:
        args.output.mkdir(parents=True, exist_ok=True)
    variants = {"predictive": []}
    if args.compare_v1:
        variants["baseline_v1_same_vision"] = []
    manifest = dict(planned_seeds=seeds, target_mode=args.target_mode, duration_s=args.duration,
        randomization=dict(level=args.randomize, master_seed=args.randomization_seed,
            components=args.randomization_components,
            bounds=RANDOMIZATION_LEVELS.get(args.randomize),
            distribution="independent bounded uniform; each draw held constant for one flight; mass scales inertia proportionally"),
        vision_config=settings.as_dict(), vision_sha256=hashlib.sha256(args.vision_config.read_bytes()).hexdigest(),
        vision_tracker="ReacquiringTracker: original detection/threshold/confirmation, reset unconfirmed lost ROI for next frame; shared by compared variants",
        flight_profile_sha256=hashlib.sha256(args.flight_config.read_bytes()).hexdigest(),
        assumptions="Physical aero/servo unchanged; ideal own navigation; static lamp/upper-plane calibration; predictor wind zero")
    for name, runs in variants.items():
        for seed in seeds:
            started = time.perf_counter()
            sim = None
            try:
                sim = create(seed, baseline=name == "baseline_v1_same_vision")
                result, rows = fly(sim, settings, args.duration, trace=args.trace)
                if args.randomize != "none":
                    result["randomization_draw"] = tolerance_sample(seed, args.randomize, args.randomization_seed)
                if args.trace:
                    (args.output/f"{name}_{seed}_trace.json").write_text(json.dumps(rows, ensure_ascii=False)+"\n", encoding="utf-8")
            except Exception as exc:
                result = dict(seed=seed, hit=False, error=repr(exc))
            finally:
                if sim is not None:
                    sim.close()
            result["wall_s"] = time.perf_counter()-started
            runs.append(result)
            report = dict(manifest=manifest, complete=all(len(value)==len(seeds) for value in variants.values()),
                          results={key: dict(summary=summarize(value), runs=value)
                                                    for key, value in variants.items()})
            if args.output is not None:
                (args.output/"result.json").write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
            print(json.dumps(dict(variant=name, seed=seed, hit=result["hit"], error=result.get("error"),
                                  summary=summarize(runs)), ensure_ascii=False), flush=True)
    return int(any("error" in row for rows in variants.values() for row in rows))


if __name__ == "__main__":
    raise SystemExit(main())
