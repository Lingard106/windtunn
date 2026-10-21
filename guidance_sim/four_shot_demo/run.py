"""Four live shots in the original single-shot traditional-control window.

Only a shot scheduler and a status line are added. Rendering, mouse controls,
waveforms, real vision and per-shot in-memory replay belong to the old viewer.
No automatic files, frame caches, preparation processes or presentation scene.
"""
from __future__ import annotations

import argparse
import copy
import json
import math
from pathlib import Path
import sys
import time

sys.dont_write_bytecode = True
PROJECT = Path(__file__).resolve().parents[1]
for directory in (PROJECT, PROJECT / "prototypes"):
    if str(directory) not in sys.path:
        sys.path.insert(0, str(directory))

from dart_demo import load_config
from simulation_setup import resolve_startup_paths
from target_motion import TARGET_MODES, TargetMotion
from traditional_control.run import fly, tolerance_sample
from traditional_control.simulation import PhysicalCandidate
from traditional_control.viewer import PhysicalTraditionalViewer
from vision_settings import VisionSettings


class LocalTargetMotion:
    """Flight/control time stays local; the target uses one match clock."""
    def __init__(self, shared, offset):
        self.shared, self.offset = shared, float(offset)

    @property
    def mode(self):
        return self.shared.mode

    @property
    def seed(self):
        return self.shared.seed

    def sample(self, t):
        return self.shared.sample(self.offset+t)

    def notify_hit(self, t):
        return self.shared.notify_hit(self.offset+t)

    def capture_state(self):
        return dict(self.shared.capture_state(), flight_offset_s=self.offset)

    def restore_state(self, state):
        if abs(float(state.get("flight_offset_s", self.offset))-self.offset) > 1e-9:
            raise ValueError("回放属于另一发飞镖。")
        self.shared.restore_state(state)

    def metadata(self):
        return dict(self.shared.metadata(), flight_offset_s=self.offset)


def shot_config(nominal, seed, index, randomize):
    config = dict(nominal, t_max_s=2.6)
    if randomize == "match":
        draw = tolerance_sample(seed+index*100003, "match")
        config["mass_kg"] *= 1+draw["mass_fraction"]
        config["speed_mps"] = max(0., config["speed_mps"]+draw["speed_mps"])
        config["roll_deg"] = nominal.get("roll_deg", 0.)+draw["roll_deg"]
        config["drag_coefficient"] *= 1+draw["drag_fraction"]
    return config


def bind_target(sim, shared, offset):
    sim.target_motion = LocalTargetMotion(shared, offset)
    sim._sync_target_motion()


class FourShotViewer(PhysicalTraditionalViewer):
    def __init__(self, root, sim, nominal, *, interval=2.6, randomize="match", **kwargs):
        import tkinter as tk
        from tkinter import ttk
        self.nominal = dict(nominal)
        self._applied_shot = dict(sim.cfg)
        self.interval, self.randomize = interval, randomize
        self.shot_index, self.launch_time = 0, 0.
        self.results = [None]*4
        self.shared_target = TargetMotion(sim.target_mode, sim.target_seed)
        self.waiting, self.wait_remaining = False, 0.
        self._automatic_next = False
        self.sequence_status = tk.StringVar(root)
        bind_target(sim, self.shared_target, 0.)
        super().__init__(root, sim, **kwargs)
        root.title("传统控制仿真 · 四发连续演示")
        ttk.Label(self.external_host, textvariable=self.sequence_status,
            style="Section.TLabel", padding=(3, 5), wraplength=1100).grid(row=3, column=0, sticky="ew")
        self._update_sequence()

    def _fill_parameters(self):
        super()._fill_parameters()
        # Edits refer to nominal values, never accumulate per-dart tolerances.
        for key, variable in self.entries.items():
            if key in self.nominal:
                variable.set(f"{self.nominal[key]:g}")

    def _new_recording(self):
        # All inherited manual apply/reset actions arrive here. Bind the target
        # BEFORE the inherited method takes its first camera/history sample.
        if not self._automatic_next:
            old_roll = self.nominal.get("roll_deg", 0.)
            nominal = dict(self.sim.cfg, roll_deg=old_roll)
            for key in ("mass_kg", "speed_mps", "drag_coefficient"):
                if nominal[key] == self._applied_shot[key]:
                    nominal[key] = self.nominal[key]
            self.nominal = nominal
            self.shot_index, self.launch_time = 0, 0.
            self.results = [None]*4
            self.waiting, self.wait_remaining = False, 0.
            self.shared_target = TargetMotion(self.sim.target_mode, self.sim.target_seed)
            self.sim.reset(shot_config(self.nominal, self.sim.target_seed, 0, self.randomize))
            self._applied_shot = dict(self.sim.cfg)
            bind_target(self.sim, self.shared_target, 0.)
        super()._new_recording()
        self._fill_parameters()
        self._update_sequence()

    def _update_result(self, final=False):
        events = (self.sim.target_contact, self.sim.target_result)
        if final or any(events):
            event = next((e for e in events if e and e.get("hit")), None)
            event = event or next((e for e in events if e), None)
            self.results[self.shot_index] = dict(hit=bool(any(e and e.get("hit") for e in events)),
                event=copy.deepcopy(event), status=self.sim.status)

    def _update_sequence(self):
        local = self.sim.t
        if self.waiting:
            local += max(0., self.interval-self.sim.t-self.wait_remaining)
        states = [f"{i+1} " + (("命中" if r["hit"] else "未中") if r else
            ("当前" if i == self.shot_index else "待发")) for i, r in enumerate(self.results)]
        suffix = f"等发 {self.wait_remaining:.1f}s（保持末帧）" if self.waiting else f"第{self.shot_index+1}发"
        self.sequence_status.set(f"{'本发回放' if self.replaying else '四发实跑'} · 赛钟 {self.launch_time+local:.2f}s · "
            + " / ".join(states) + f" · {suffix}")

    def _next_shot(self):
        next_time = self.launch_time+max(self.interval, self.sim.t)
        next_index = self.shot_index+1
        self.sim.reset(shot_config(self.nominal, self.sim.target_seed, next_index, self.randomize))
        self.shared_target.on_launch(next_time)
        self.launch_time, self.shot_index = next_time, next_index
        self._applied_shot = dict(self.sim.cfg)
        bind_target(self.sim, self.shared_target, self.launch_time)
        self.waiting = False
        self._automatic_next = True
        try:
            self._new_recording()
        finally:
            self._automatic_next = False
        self.running = True
        self.accumulator = 0.
        self.last_wall = time.perf_counter()
        self.play_button.configure(text="暂停")
        self.state_label.set("运行中")

    def toggle_running(self):
        if self.waiting and not self.replaying:
            self.running = not self.running
            self.accumulator = 0.
            self.last_wall = time.perf_counter()
            self.play_button.configure(text="暂停" if self.running else "开始")
            self.state_label.set("等发中" if self.running else "暂停中")
            self._update_sequence()
        else:
            super().toggle_running()

    def _tick(self):
        if self.closed:
            return
        if self.waiting and not self.replaying:
            now = time.perf_counter()
            elapsed = min(.25, max(0., now-self.last_wall))
            self.last_wall = now
            if self.running:
                self.wait_remaining = max(0., self.wait_remaining-elapsed*float(self.speed_choice.get().replace("×", "")))
                if self.wait_remaining <= 1e-9:
                    try:
                        self._next_shot()
                    except Exception as exc:
                        self.running = self.waiting = False
                        self.play_button.configure(text="开始")
                        self.message.set(f"换发失败：{exc}；可重置整轮。")
            if self.dirty:
                self._render()
            self._update_sequence()
            self._after_id = self.root.after(16, self._tick)
            return
        was_running, was_replaying = self.running, self.replaying
        super()._tick()  # Original physics, 90Hz vision, scene, curves and camera.
        if self.closed:
            return
        if not self.replaying:
            hit_times = [float(e.get("time_s", e.get("crossing_time_s")))
                for e in (self.sim.target_contact, self.sim.target_result) if e and e.get("hit")]
            if was_running and not self.sim.done and hit_times and self.sim.t >= min(hit_times)+.05:
                # Same terminal window as the accepted physical runner: retain
                # actual contact/initial rebound, then wait for the next launch.
                self.sim.done, self.sim.status = True, "target_hit"
                if self._vision_t != self.sim.t:
                    self._sample_vision()
                self._record_flight_debug()
                self.dirty = True
            self._update_result(final=self.sim.done)
            if was_running and not was_replaying and self.sim.done:
                self.running = False
                self.accumulator = 0.
                self.play_button.configure(text="开始")
                self.state_label.set("本次已结束")
                if self.shot_index < 3 and self.sim.status != "invalid":
                    self.waiting = self.running = True
                    self.wait_remaining = max(0., self.interval-self.sim.t)
                    self.play_button.configure(text="暂停")
                    self.state_label.set("等发中")
                else:
                    hits = sum(bool(r and r["hit"]) for r in self.results)
                    self.message.set(f"本轮结束：{hits}/4 命中。重置可重开四发；时间轴回放当前发。")
        self._update_sequence()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", "--target-seed", type=int, default=2027)
    parser.add_argument("--target-mode", choices=tuple(TARGET_MODES), default="random_moving")
    parser.add_argument("--interval", type=float, default=2.6)
    parser.add_argument("--randomize", choices=("none", "match"), default="match")
    parser.add_argument("--auto-run", action="store_true")
    parser.add_argument("--auto-close", type=float)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--output", type=Path, help="Explicit headless JSON only; no images")
    args = parser.parse_args(argv)
    if not math.isfinite(args.interval) or not 2.6 <= args.interval <= 9.:
        parser.error("间隔必须为2.6至9秒")
    if args.auto_close is not None and (not math.isfinite(args.auto_close) or args.auto_close <= 0):
        parser.error("auto-close必须为正数")
    if args.output and not args.headless:
        parser.error("仅--headless配合--output保存JSON，窗口默认不写文件")
    paths = resolve_startup_paths()
    settings = VisionSettings(**json.loads((PROJECT/"profiles/vision_desktop.json").read_text(encoding="utf-8-sig")))
    sim = PhysicalCandidate(load_config(paths.config), paths.scene, paths.dart, paths.launch,
        target_mode=args.target_mode, target_seed=args.seed, flight_control_enabled=True)
    root = viewer = None
    try:
        nominal = dict(sim.cfg, **paths.explicit_values)
        sim.reset(shot_config(nominal, args.seed, 0, args.randomize))
        if args.headless:
            shared, rows = TargetMotion(args.target_mode, args.seed), []
            for index in range(4):
                if index:
                    shared.on_launch(index*args.interval)
                    sim.reset(shot_config(nominal, args.seed, index, args.randomize))
                bind_target(sim, shared, index*args.interval)
                row, _ = fly(sim, settings, 2.6)
                rows.append(dict(shot=index+1, launch_time_s=index*args.interval, **row))
            document = dict(hits=sum(r["hit"] for r in rows), shots=rows, interval_s=args.interval)
            if args.output:
                args.output.parent.mkdir(parents=True, exist_ok=True)
                args.output.write_text(json.dumps(document, ensure_ascii=False, indent=2), encoding="utf-8")
            print(json.dumps(dict(hits=document["hits"], shots=4), ensure_ascii=False))
        else:
            import tkinter as tk
            root = tk.Tk()
            viewer = FourShotViewer(root, sim, nominal, interval=args.interval, randomize=args.randomize,
                vision_settings=settings, auto_run=args.auto_run)
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


if __name__ == "__main__":
    raise SystemExit(main())
