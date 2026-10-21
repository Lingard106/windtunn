"""2027 基地四种飞镖目标的单开门窗运动；随机轨迹是可复现的仿真假设。

规则依据手册 V1.0.0 §5.7.3（PDF 111–112 / 印刷 110–111 页）。
t=0 表示闸门已完全打开且第一发飞镖离架，不模拟此前约 7 秒的开门过程。
sample 只读预排轨迹，反复渲染或回放不会抽取新的随机位置。
"""

from __future__ import annotations

import copy
import math
import random


TARGET_MODES = {
    "fixed": "固定目标",
    "random_fixed": "随机固定目标",
    "random_moving": "随机移动目标",
    "end_moving": "末端移动目标",
}


class TargetMotion:
    """沿世界 Y 平移的事件程序，位移、速度分别使用 m 和 m/s。"""

    LIMIT_M = .28
    MOVE_S = .6
    END_DELAY_S = 1.2
    ROAM_LEG_S = 2.
    WINDOW_S = 40.
    LAUNCH_WINDOW_S = 30.
    RESUME_S = 10.
    HIT_COOLDOWN_S = 2.

    def __init__(self, mode="fixed", seed=2027):
        if mode not in TARGET_MODES:
            raise ValueError(f"未知目标模式：{mode!r}")
        if isinstance(seed, bool) or not isinstance(seed, int):
            raise ValueError("目标随机种子必须是整数。")
        self.mode, self.seed = mode, seed
        self._rng = random.Random(seed)
        self._segments = []
        self._closed_intervals = []
        self._last_event_t = 0.
        self._initial_offset = 0. if mode == "fixed" else self._rng.uniform(-self.LIMIT_M, self.LIMIT_M)
        self._last_launch_offset = self._initial_offset
        self._hit_count = 0
        initially_eligible = mode in ("fixed", "random_fixed")
        self._append(0., 0., self._initial_offset, self._initial_offset,
                     mode if initially_eligible else "initial", True, initially_eligible)
        self.on_launch(0.)
        if initially_eligible:
            self._close_window()

    @staticmethod
    def _time(value):
        try:
            value = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError("目标时间必须是非负有限秒数。") from exc
        if not math.isfinite(value) or value < 0:
            raise ValueError("目标时间必须是非负有限秒数。")
        return value

    def _event_time(self, value):
        value = self._time(value)
        if value < self._last_event_t:
            raise ValueError("目标发射和命中事件必须按时间顺序输入。")
        return value

    def _destination(self, origin):
        for _ in range(100):
            point = self._rng.uniform(-self.LIMIT_M, self.LIMIT_M)
            if abs(point - origin) > 1e-6:
                return point
        return -self.LIMIT_M if origin >= 0 else self.LIMIT_M

    def _append(self, start, end, origin, destination, phase, settled=False, eligible=False):
        self._segments.append({"start": float(start), "end": float(end),
            "origin": float(origin), "destination": float(destination),
            "phase": phase, "settled": settled, "eligible": eligible})

    def _position(self, t):
        # 新事件覆盖其时刻之后的程序；保留旧段原始时长，过去的插值不受影响。
        segment = next(item for item in reversed(self._segments) if item["start"] <= t)
        duration = segment["end"] - segment["start"]
        u = min(1., max(0., (t-segment["start"])/duration)) if duration > 0 else 1.
        ease = u*u*u*(10.+u*(-15.+6.*u))
        derivative = 30.*u*u*(u-1.)*(u-1.)/duration if duration > 0 else 0.
        delta = segment["destination"] - segment["origin"]
        return segment, segment["origin"]+delta*ease, delta*derivative

    def _truncate(self, t):
        self._segments = [item for item in self._segments if item["start"] < t]

    def _roam(self, t, position):
        while t < self.WINDOW_S:
            destination = self._destination(position)
            end = t+self.ROAM_LEG_S
            self._append(t, end, position, destination, "roaming")
            t, position = end, destination

    def _close_window(self):
        _, position, _ = self._position(self.WINDOW_S)
        if self.mode == "random_fixed":
            self._append(self.WINDOW_S, self.WINDOW_S+self.MOVE_S, position, 0., "returning")
            self._append(self.WINDOW_S+self.MOVE_S, self.WINDOW_S+self.MOVE_S,
                         0., 0., "window_closed", True)
        else:
            self._append(self.WINDOW_S, self.WINDOW_S, position, position, "window_closed", True)

    def sample(self, t):
        """查询任意非负时刻；不改变轨迹、事件状态或随机数流。"""
        t = self._time(t)
        segment, position, velocity = self._position(t)
        detection_open = t < self.WINDOW_S and not any(a <= t < b for a, b in self._closed_intervals)
        settled = bool(segment["settled"])
        return {"mode": self.mode, "label": TARGET_MODES[self.mode],
            "offset_m": position, "velocity_mps": velocity,
            "moving": not settled, "settled": settled, "phase": segment["phase"],
            "detection_open": detection_open,
            "hit_eligible": bool(detection_open and segment["eligible"] and settled)}

    def on_launch(self, t):
        """通知同一开门窗内的新一次发射；首次发射已在构造时记录。"""
        t = self._event_time(t)
        if t > self.LAUNCH_WINDOW_S:
            raise ValueError("完全开门后的发射窗口只有 30 秒。")
        _, position, _ = self._position(t)
        self._last_launch_offset = position
        self._last_event_t = t
        if self.mode in ("fixed", "random_fixed"):
            # 这两种模式不响应发出事件，尤其不能中断命中后正在进行的换位。
            return
        self._truncate(t)
        delay = self.END_DELAY_S if self.mode == "end_moving" else 0.
        start, end = t+delay, t+delay+self.MOVE_S
        if delay:
            # 物理上已静止，但本发的规定变位尚未完成，因此尚不能有效命中。
            self._append(t, start, position, position, "waiting_delay", True, False)
        destination = self._destination(position)
        self._append(start, end, position, destination, "reacting")
        self._append(end, end, destination, destination, "settled", True, True)
        # 规则的 10 s 只锚定本开门窗第一发，不为后续每发重新计时。
        # 若新一次反应跨过该时刻，先完成规定变位，再开始游走。
        if t < self.RESUME_S:
            self._roam(max(end, self.RESUME_S), destination)
        self._close_window()

    def notify_hit(self, t):
        """仅有效命中触发换位和 2 秒检测禁用；否则返回 False 且不改程序。"""
        t = self._event_time(t)
        state = self.sample(t)
        if not state["hit_eligible"]:
            return False
        self._last_event_t = t
        self._hit_count += 1
        self._closed_intervals.append((t, t+self.HIT_COOLDOWN_S))
        if self.mode == "fixed":
            return True
        position = state["offset_m"]
        self._truncate(t)
        if self.mode == "random_fixed":
            destination = self._destination(position)
            end = t+self.MOVE_S
            self._append(t, end, position, destination, "repositioning")
            self._append(end, end, destination, destination, "random_fixed", True, True)
        else:
            self._roam(t, position)
        self._close_window()
        return True

    def capture_state(self):
        """保存预排轨迹和事件随机状态，供同轮回放或继续计算。"""
        return copy.deepcopy({"version": 1, "mode": self.mode, "seed": self.seed,
            "segments": self._segments, "closed_intervals": self._closed_intervals,
            "last_event_t": self._last_event_t, "initial_offset": self._initial_offset,
            "last_launch_offset": self._last_launch_offset, "hit_count": self._hit_count,
            "rng_state": self._rng.getstate()})

    def restore_state(self, state):
        """恢复 capture_state 生成的状态；不抽样随机数，不推进仿真。"""
        if not isinstance(state, dict) or state.get("version") != 1 or state.get("mode") not in TARGET_MODES:
            raise ValueError("目标回放状态无效。")
        state = copy.deepcopy(state)
        self.mode, self.seed = state["mode"], state["seed"]
        self._segments, self._closed_intervals = state["segments"], state["closed_intervals"]
        self._last_event_t, self._initial_offset = state["last_event_t"], state["initial_offset"]
        self._last_launch_offset, self._hit_count = state["last_launch_offset"], state["hit_count"]
        self._rng.setstate(state["rng_state"])

    def metadata(self):
        """区分手册明确约束、事件状态和本实现必须补充的运动假设。"""
        return {"mode": self.mode, "label": TARGET_MODES[self.mode], "seed": self.seed,
            "initial_offset_m": self._initial_offset, "last_launch_offset_m": self._last_launch_offset,
            "hit_count": self._hit_count, "axis_W": [0., 1., 0.],
            "rules": {"source": "2027 规则手册 V1.0.0 §5.7.3，PDF 111–112 / 印刷 110–111 页",
                "offset_range_m": [-self.LIMIT_M, self.LIMIT_M], "reaction_move_limit_s": self.MOVE_S,
                "end_move_delay_s": self.END_DELAY_S, "first_launch_resume_s": self.RESUME_S,
                "launch_window_s": self.LAUNCH_WINDOW_S, "detection_window_s": self.WINDOW_S,
                "hit_cooldown_s": self.HIT_COOLDOWN_S, "requires_completed_move_and_stationary": True},
            "simulation_assumptions": [
                "t=0 为门已全开且首镖离架；随机固定已于开门阶段完成首次随机就位，不模拟闸门动画",
                "随机位置在 ±0.28 m 内均匀抽样，与发出时位置至少相差 1 µm；分布和最小差不是官方指定",
                "变位用 0.6 s 五次 smoothstep，两端速度和加速度为零；曲线和采用完整 0.6 s 是仿真假设",
                "非等待发射阶段的游走由连续 2 s 随机变位组成；规则未指定游走速度或周期",
                "末端等待 1.2 s 时保持发出位置，物理静止但本发移动未完成，命中无效",
                "10 s 自动恢复只以本窗第一发计时一次；后续发射若已超过此时刻，停稳后保持至命中或窗结束",
                "若后续变位跨过首次 10 s 恢复点，先完成本发变位再游走；此多发重叠顺序为训练解释",
                "随机固定命中后换位和窗结束回中均采用 0.6 s；另外两种窗结束在当时位置停止",
                "本模块不模拟官方红外投影识别、完整四发弹药、伤害经验与护甲展开"],
        }
