"""仿真时间采样时钟与内存中的回放状态缓冲区。"""
from __future__ import annotations

from bisect import bisect_right
from copy import deepcopy
from dataclasses import dataclass
import math
from typing import Any


_TIME_EPSILON_S = 1e-12


def _finite_time(value: float) -> float:
    """把时间转换成有限浮点数，避免 NaN 破坏时间顺序。"""
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("时间必须是有限数值")
    return result


class SampleClock:
    """按仿真时间安排固定频率采样，并报告跨过的采样格数量。"""

    def __init__(self, hz: float = 90.0):
        frequency = float(hz)
        if not math.isfinite(frequency) or frequency <= 0.0:
            raise ValueError("采样频率必须是大于零的有限数值")
        self.hz = frequency
        self.period = 1.0 / frequency
        self.reset()

    def reset(self, t: float = 0.0) -> None:
        """将时钟锚定到 t；下一格位于 t + 1/hz。"""
        self._origin = _finite_time(t)
        self._next_index = 1
        self.next = self._origin + self.period

    def due(self, t: float) -> bool:
        """只判断 t 是否到达下一格，不改变时钟状态。"""
        current = _finite_time(t)
        return current + _TIME_EPSILON_S >= self.next

    def consume(self, t: float) -> int:
        """消费 t 前已经跨过的格数；调用方只处理当前真实状态一次。"""
        current = _finite_time(t)
        if not self.due(current):
            return 0

        # 用锚点和整数格号计算，避免反复累加周期带来的长期漂移。
        latest_index = math.floor((current - self._origin + _TIME_EPSILON_S) * self.hz)
        crossed = max(1, latest_index - (self._next_index - 1))
        self._next_index += crossed
        self.next = self._origin + self._next_index * self.period
        return crossed


@dataclass(frozen=True)
class ReplayFrame:
    """某一仿真时刻的状态与视觉结果；回放记录不保存图像。"""

    t: float
    state: dict[str, Any]
    vision: dict[str, Any]


class ReplayBuffer:
    """按时间保存有限数量的独立状态快照，供时间轴定位和回放。"""

    def __init__(self, max_frames: int = 10000):
        if isinstance(max_frames, bool) or not isinstance(max_frames, int) or max_frames < 1:
            raise ValueError("max_frames 必须是正整数")
        self.max_frames = max_frames
        self._frames: list[ReplayFrame] = []
        self._times: list[float] = []

    def append(self, t: float, state: dict[str, Any], vision: dict[str, Any]) -> None:
        """追加时间递增的快照；相同时间会替换末帧。"""
        timestamp = _finite_time(t)
        if not isinstance(state, dict) or not isinstance(vision, dict):
            raise TypeError("state 和 vision 必须是字典")
        frame = ReplayFrame(timestamp, deepcopy(state), deepcopy(vision))

        if self._times:
            latest = self._times[-1]
            if timestamp < latest:
                raise ValueError("回放帧时间必须单调递增")
            if timestamp == latest:
                self._frames[-1] = frame
                return

        self._times.append(timestamp)
        self._frames.append(frame)
        if len(self._frames) > self.max_frames:
            del self._frames[0]
            del self._times[0]

    def clear(self) -> None:
        """清空本轮记录。"""
        self._frames.clear()
        self._times.clear()

    @property
    def frames(self) -> list[ReplayFrame]:
        """返回按时间排序的快照副本，调用方不能改写缓冲区内容。"""
        return [self._copy_frame(frame) for frame in self._frames]

    def __len__(self) -> int:
        return len(self._frames)

    @property
    def start_time(self) -> float | None:
        return self._times[0] if self._times else None

    @property
    def end_time(self) -> float | None:
        return self._times[-1] if self._times else None

    def index_at(self, t: float) -> int:
        """返回不晚于 t 的最近帧索引，并将越界时间夹到首尾帧。"""
        if not self._frames:
            raise IndexError("回放缓冲区为空")
        timestamp = _finite_time(t)
        return max(0, min(len(self._frames) - 1, bisect_right(self._times, timestamp) - 1))

    def at(self, t: float) -> ReplayFrame:
        """返回不晚于 t 的最近状态副本。"""
        return self._copy_frame(self._frames[self.index_at(t)])

    @staticmethod
    def _copy_frame(frame: ReplayFrame) -> ReplayFrame:
        return ReplayFrame(frame.t, deepcopy(frame.state), deepcopy(frame.vision))
