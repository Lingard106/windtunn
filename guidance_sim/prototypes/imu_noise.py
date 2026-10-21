"""七个原飞控语义通道及独立滚转陀螺通道的可复现 IMU 高斯噪声，不代表 BMI088 实测误差。

只消费传入的理想 IMU 字典。每个绝对时间 epoch 保存一个噪声向量，
同一 epoch 的重复读取仍使用当前理想输入，不会冻结传感器的理想值。
"""

from __future__ import annotations

import copy
import math
from numbers import Real
from typing import Any, Mapping

import numpy as np


IMU_KEYS = ("pitch_deg", "yaw_deg", "roll_deg", "pitch_rate_deg_s",
            "yaw_rate_deg_s", "a_pitch_g", "a_yaw_g")
ROLL_KEY = "roll_rate_deg_s"
_PARAMETER_KEYS = ("enabled", "scale", "attitude_std_deg", "rate_std_deg_s",
                   "accel_std_g", "seed", "period_s")
_EPOCH_TOL = 1e-9


def _finite(value: Any, name: str, *, nonnegative: bool = False,
            positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(f"{name} 必须是有限实数。")
    result = float(value)
    if (nonnegative and result < 0) or (positive and result <= 0):
        raise ValueError(f"{name} 必须{'大于零' if positive else '非负'}。")
    return result


def _channels(value: Any, name: str) -> dict[str, float]:
    if not isinstance(value, Mapping) or set(value) not in (set(IMU_KEYS), set(IMU_KEYS) | {ROLL_KEY}):
        raise ValueError(f"{name} 必须包含七个原IMU通道，可另含roll_rate_deg_s。")
    keys = IMU_KEYS + ((ROLL_KEY,) if ROLL_KEY in value else ())
    return {key: _finite(value[key], f"{name}.{key}") for key in keys}


class ImuNoise:
    """绝对仿真时钟上的独立零均值高斯噪声；开关与参数在构造时确定。"""

    def __init__(self, config: Mapping[str, Any]):
        section = config.get("imu_noise")
        if not isinstance(section, Mapping):
            raise ValueError("飞控配置缺少 imu_noise 参数。")
        if not all(key in section for key in _PARAMETER_KEYS):
            raise ValueError("imu_noise 参数不完整。")
        if not isinstance(section["enabled"], bool):
            raise ValueError("imu_noise.enabled 必须是布尔值。")
        seed = section["seed"]
        if isinstance(seed, bool) or not isinstance(seed, (int, np.integer)) or seed < 0:
            raise ValueError("imu_noise.seed 必须是非负整数。")
        self.config = {
            "enabled": section["enabled"],
            "scale": _finite(section["scale"], "imu_noise.scale", nonnegative=True),
            "attitude_std_deg": _finite(section["attitude_std_deg"], "imu_noise.attitude_std_deg", nonnegative=True),
            "rate_std_deg_s": _finite(section["rate_std_deg_s"], "imu_noise.rate_std_deg_s", nonnegative=True),
            "accel_std_g": _finite(section["accel_std_g"], "imu_noise.accel_std_g", nonnegative=True),
            "seed": int(seed),
            "period_s": _finite(section["period_s"], "imu_noise.period_s", positive=True),
        }
        category_std = (self.config["attitude_std_deg"],)*3 + (self.config["rate_std_deg_s"],)*2 + (self.config["accel_std_g"],)*2
        with np.errstate(over="ignore", invalid="ignore"):
            self._std = np.asarray(category_std)*self.config["scale"]
        if not np.isfinite(self._std).all():
            raise ValueError("imu_noise 的标准差乘 scale 后必须有限。")
        self._rng = np.random.default_rng(self.config["seed"])
        # A separate stream leaves every original seven-channel draw unchanged.
        self._roll_rng = np.random.default_rng(np.random.SeedSequence(
            self.config["seed"], spawn_key=(0x524F4C4C,)))
        self._roll_last_epoch: int | None = None
        self._roll_cached_noise = 0.
        self._last_epoch: int | None = None
        self._last_time_s: float | None = None
        self._cached_noise = np.zeros(7)

    def _epoch(self, t_s: float) -> int:
        quotient = t_s/self.config["period_s"]
        if not math.isfinite(quotient):
            raise ValueError("IMU 时间与噪声周期之比必须有限。")
        return math.floor(quotient+_EPOCH_TOL)

    def measure(self, ideal_imu_dict: Mapping[str, float], t_s: float) -> dict[str, float]:
        """返回带噪语义值；关闭或 scale=0 时原值复制且不消耗随机数。"""
        ideal = _channels(ideal_imu_dict, "ideal_imu")
        sample_time = _finite(t_s, "IMU t_s", nonnegative=True)
        if self._last_time_s is not None and sample_time < self._last_time_s:
            raise ValueError("IMU 测量时间不能倒退；回放必须先 restore_state。")
        epoch = self._epoch(sample_time)
        if not self.config["enabled"] or self.config["scale"] == 0:
            self._last_time_s = sample_time
            # Preserve values exactly, including their numeric representation.
            return {key: ideal_imu_dict[key] for key in ideal}
        if epoch != self._last_epoch:
            previous_epoch = -1 if self._last_epoch is None else self._last_epoch
            remaining = epoch-previous_epoch
            # Consume skipped epochs too: draw count depends on absolute time, not
            # how often UI/physics happened to request a measurement in between.
            while remaining > 0:
                count = min(remaining, 4096)
                last_draw = self._rng.standard_normal((count, 7))[-1]
                remaining -= count
            self._cached_noise = last_draw*self._std
            self._last_epoch = epoch
        result = {key: ideal[key]+float(self._cached_noise[index]) for index, key in enumerate(IMU_KEYS)}
        if ROLL_KEY in ideal:
            self._advance_roll(epoch)
            result[ROLL_KEY] = ideal[ROLL_KEY] + self._roll_cached_noise
        if not all(math.isfinite(value) for value in result.values()):
            raise ValueError("IMU 加噪结果必须有限。")
        self._last_time_s = sample_time
        return result

    def _advance_roll(self, epoch):
        if epoch != self._roll_last_epoch:
            remaining = epoch - (-1 if self._roll_last_epoch is None else self._roll_last_epoch)
            while remaining > 0:
                count = min(remaining, 4096)
                last_draw = self._roll_rng.standard_normal(count)[-1]
                remaining -= count
            self._roll_cached_noise = float(last_draw*self._std[3])
            self._roll_last_epoch = epoch

    def capture_state(self) -> dict[str, Any]:
        """独立、JSON 兼容的随机数和 epoch 快照。"""
        return {
            "schema_version": 2, "parameters": dict(self.config),
            "rng_state": copy.deepcopy(self._rng.bit_generator.state),
            "last_epoch": self._last_epoch, "last_time_s": self._last_time_s,
            "cached_noise": {key: float(self._cached_noise[i]) for i, key in enumerate(IMU_KEYS)},
            "roll_rng_state": copy.deepcopy(self._roll_rng.bit_generator.state),
            "roll_last_epoch": self._roll_last_epoch,
            "roll_cached_noise": self._roll_cached_noise,
        }

    def restore_state(self, snapshot: Mapping[str, Any]):
        """原子恢复；参数不匹配、非法缓存或时间时钟会拒绝。"""
        if (not isinstance(snapshot, Mapping) or isinstance(snapshot.get("schema_version"), bool)
                or snapshot.get("schema_version") not in (1, 2)):
            raise ValueError("IMU 噪声快照 schema_version 必须为 1 或 2。")
        if snapshot.get("parameters") != self.config:
            raise ValueError("IMU 噪声快照参数与本次配置不一致。")
        try:
            time_s = snapshot["last_time_s"]
            if time_s is not None:
                time_s = _finite(time_s, "IMU last_time_s", nonnegative=True)
            epoch = snapshot["last_epoch"]
            if epoch is not None:
                if isinstance(epoch, bool) or not isinstance(epoch, (int, np.integer)) or epoch < 0:
                    raise ValueError("IMU last_epoch 必须是非负整数或 None。")
                epoch = int(epoch)
                if time_s is None or epoch != self._epoch(time_s):
                    raise ValueError("IMU 噪声快照的 epoch 与测量时间不一致。")
            noise_dict = _channels(snapshot["cached_noise"], "IMU cached_noise")
            if set(noise_dict) != set(IMU_KEYS):
                raise ValueError("原七通道噪声缓存不应混入滚转速率。")
            noise = np.array([noise_dict[key] for key in IMU_KEYS])
            if epoch is None and np.any(noise != 0):
                raise ValueError("没有采样 epoch 的快照不能含非零缓存噪声。")
            candidate_rng = np.random.default_rng(self.config["seed"])
            candidate_rng.bit_generator.state = copy.deepcopy(snapshot["rng_state"])
            candidate_roll_rng = np.random.default_rng(np.random.SeedSequence(
                self.config["seed"], spawn_key=(0x524F4C4C,)))
            roll_epoch, roll_noise = None, 0.
            if snapshot["schema_version"] == 2:
                roll_epoch = snapshot["roll_last_epoch"]
                roll_noise = _finite(snapshot["roll_cached_noise"], "roll_cached_noise")
                if roll_epoch is not None:
                    if (isinstance(roll_epoch, bool) or not isinstance(roll_epoch, (int, np.integer))
                            or roll_epoch < 0 or epoch is None or roll_epoch > epoch):
                        raise ValueError("滚转噪声 epoch 必须不晚于原通道 epoch。")
                elif roll_noise != 0.:
                    raise ValueError("未采样的滚转通道不能含非零噪声。")
                candidate_roll_rng.bit_generator.state = copy.deepcopy(snapshot["roll_rng_state"])
        except (KeyError, TypeError, OverflowError) as exc:
            raise ValueError("IMU 噪声快照字段或随机数状态无效。") from exc
        self._rng, self._last_epoch = candidate_rng, epoch
        self._last_time_s, self._cached_noise = time_s, noise
        self._roll_rng, self._roll_last_epoch = candidate_roll_rng, roll_epoch
        self._roll_cached_noise = roll_noise

    def metadata(self) -> dict[str, Any]:
        return {
            "model": "independent_gaussian_imu",
            "parameters": dict(self.config),
            "effective_std": {
                "attitude_deg": float(self._std[0]), "rate_deg_s": float(self._std[3]),
                "accel_g": float(self._std[5]),
            },
            "provenance": "仿真初始误差参数；未经 BMI088 实测标定。",
            "epoch_rule": "floor(t_s / period_s + 1e-9); noise held within an epoch",
            "channels": list(IMU_KEYS) + [ROLL_KEY],
            "roll_stream": "independent SeedSequence(seed, spawn_key=(0x524F4C4C,)); rate_std_deg_s; original seven-channel RNG unchanged",
            "not_modeled": ["bias", "drift", "saturation", "cross_channel_correlation"],
            "validation_status": "unvalidated",
        }

    def observation(self) -> dict[str, Any]:
        return {
            "enabled": self.config["enabled"], "scale": self.config["scale"],
            "last_epoch": self._last_epoch, "last_time_s": self._last_time_s,
            "cached_noise": {key: float(self._cached_noise[i]) for i, key in enumerate(IMU_KEYS)},
            "roll_last_epoch": self._roll_last_epoch,
            "roll_cached_noise": self._roll_cached_noise,
        }
