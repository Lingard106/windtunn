"""四舵执行机构与聚合舵效估计，不读取目标信息，也不修改被动气动。

四通道顺序遵循 Guidedemo 的混控代数；其真实安装方向尚待标定。
本次仿真约定 p 正为抬头、y 正为向左、r 正为绕局部 +X 转动。
这是一组作用于重心的增量导数，不代表 CAD 四片尾翼的逐片载荷。
"""

from __future__ import annotations

import math
from numbers import Real
from typing import Any, Mapping

import numpy as np

if __package__:
    from .aerodynamics import AeroLoads, AeroState
else:
    from aerodynamics import AeroLoads, AeroState


_TIME_EPS = 1e-12
_SERVO_KEYS = ("limit_deg", "rate_deg_s", "tau_s", "pwm_period_s", "center_us",
               "us_per_deg", "pwm_min_us", "pwm_max_us")
_ACTUATION_KEYS = ("normal_per_rad", "pitch_moment_per_rad", "yaw_moment_per_rad",
                   "roll_moment_per_rad", "drag_per_rad2")


def _number(value: Any, label: str, *, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(f"{label} 必须是有限实数。")
    value = float(value)
    if value < 0 or (positive and value == 0):
        raise ValueError(f"{label} 必须{'大于零' if positive else '非负'}。")
    return value


def _angles(value: Any, label: str) -> np.ndarray:
    if np.iscomplexobj(value):
        raise ValueError(f"{label} 必须包含四个有限实数。")
    try:
        angles = np.array(value, dtype=float, copy=True)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{label} 必须包含四个有限实数。") from exc
    if angles.shape != (4,) or not np.isfinite(angles).all():
        raise ValueError(f"{label} 必须包含四个有限实数。")
    return angles


def _section(config: Mapping[str, Any], name: str, keys: tuple[str, ...],
             *, positive: bool = False) -> dict[str, float]:
    section = config.get(name)
    if not isinstance(section, Mapping):
        raise ValueError(f"飞控配置缺少 {name} 参数。")
    try:
        return {key: _number(section[key], f"{name}.{key}", positive=positive) for key in keys}
    except KeyError as exc:
        raise ValueError(f"飞控配置缺少 {name}.{exc.args[0]} 参数。") from exc


class ServoActuation:
    """50 Hz PWM 保持、行程限位和限速一阶舵机。

    advance 更新当前待采样指令，并积分半开区间 [time_s, time_s+dt_s)。
    恰好落在终点的 PWM 事件留待下次调用，让同刻控制器的新指令先到达。
    首个 PWM 事件在 t=0；dt=0 只更新指令，不消耗事件。
    """

    def __init__(self, full_flight_config: Mapping[str, Any]):
        self.config = _section(full_flight_config, "servo", _SERVO_KEYS, positive=True)
        if not self.config["pwm_min_us"] <= self.config["center_us"] <= self.config["pwm_max_us"]:
            raise ValueError("servo.center_us 必须处于 PWM 范围内。")
        if self.config["pwm_min_us"] >= self.config["pwm_max_us"]:
            raise ValueError("servo.pwm_min_us 必须小于 pwm_max_us。")
        self._time_s = 0.
        self._next_pwm_index = 0
        self._command_deg = np.zeros(4)
        self._held_deg = np.zeros(4)
        self._actual_deg = np.zeros(4)
        self._pwm_us = np.full(4, self.config["center_us"])

    @property
    def next_pwm_time_s(self) -> float:
        return self._next_pwm_index * self.config["pwm_period_s"]

    def _sample_pwm(self):
        cfg = self.config
        # Clip before multiplication as well, avoiding overflow on huge finite commands.
        command = np.clip(self._command_deg,
                          (cfg["pwm_min_us"]-cfg["center_us"])/cfg["us_per_deg"],
                          (cfg["pwm_max_us"]-cfg["center_us"])/cfg["us_per_deg"])
        self._pwm_us = np.clip(cfg["center_us"] + command*cfg["us_per_deg"],
                               cfg["pwm_min_us"], cfg["pwm_max_us"])
        self._held_deg = np.clip((self._pwm_us-cfg["center_us"])/cfg["us_per_deg"],
                                 -cfg["limit_deg"], cfg["limit_deg"])
        self._next_pwm_index += 1

    def _integrate_held(self, dt_s: float):
        """Exact constant-command solution of dδ/dt=clip((held-δ)/tau,±rate).

        The rate-limited part is linear until |error|=rate*tau, followed by the
        exponential part. This remains consistent when a physical step is split.
        """
        if dt_s <= 0:
            return
        rate, tau = self.config["rate_deg_s"], self.config["tau_s"]
        error = self._held_deg - self._actual_deg
        sign, magnitude = np.sign(error), np.abs(error)
        knee = rate*tau
        linear_time = np.maximum((magnitude-knee)/rate, 0.)
        linear_duration = np.minimum(dt_s, linear_time)
        remaining_error = magnitude-rate*linear_duration
        remaining_time = np.maximum(dt_s-linear_time, 0.)
        final_error = remaining_error*np.exp(-remaining_time/tau)
        self._actual_deg = self._held_deg-sign*final_error

    def advance(self, command_deg: Any, dt_s: float) -> np.ndarray:
        command = _angles(command_deg, "command_deg")
        duration = _number(dt_s, "dt_s")
        end_s = self._time_s + duration
        if not math.isfinite(end_s):
            raise ValueError("舵机累计时间必须有限。")
        self._command_deg = command
        if duration == 0:
            return self._actual_deg.copy()
        while self.next_pwm_time_s < end_s-_TIME_EPS:
            boundary = self.next_pwm_time_s
            self._integrate_held(max(boundary-self._time_s, 0.))
            self._time_s = max(boundary, self._time_s)
            self._sample_pwm()
        self._integrate_held(end_s-self._time_s)
        self._time_s = end_s
        return self._actual_deg.copy()

    def capture_state(self) -> dict[str, Any]:
        """JSON 兼容快照；包括 PWM 相位和最后待采样指令。"""
        return {
            "schema_version": 1, "time_s": self._time_s,
            "next_pwm_index": self._next_pwm_index,
            "command_deg": self._command_deg.tolist(),
            "held_deg": self._held_deg.tolist(),
            "actual_deg": self._actual_deg.tolist(), "pwm_us": self._pwm_us.tolist(),
        }

    def restore_state(self, snapshot: Mapping[str, Any]):
        """先完整校验后恢复，失败时保留原执行机构状态。"""
        if not isinstance(snapshot, Mapping) or snapshot.get("schema_version") != 1:
            raise ValueError("舵机快照 schema_version 必须为 1。")
        try:
            time_s = _number(snapshot["time_s"], "servo.time_s")
            index = snapshot["next_pwm_index"]
            if isinstance(index, bool) or not isinstance(index, (int, np.integer)) or index < 0:
                raise ValueError("舵机快照的 next_pwm_index 必须是非负整数。")
            next_s = index*self.config["pwm_period_s"]
            if not math.isfinite(next_s) or not time_s-_TIME_EPS <= next_s <= time_s+self.config["pwm_period_s"]+_TIME_EPS:
                raise ValueError("舵机快照的 PWM 时钟与时间不一致。")
            command = _angles(snapshot["command_deg"], "servo.command_deg")
            held = _angles(snapshot["held_deg"], "servo.held_deg")
            actual = _angles(snapshot["actual_deg"], "servo.actual_deg")
            pwm = _angles(snapshot["pwm_us"], "servo.pwm_us")
        except KeyError as exc:
            raise ValueError(f"舵机快照缺少 {exc.args[0]}。") from exc
        limit = self.config["limit_deg"]
        if np.any(np.abs(held) > limit+1e-10) or np.any(np.abs(actual) > limit+1e-10):
            raise ValueError("舵机快照角度超过机械行程。")
        if np.any(pwm < self.config["pwm_min_us"]) or np.any(pwm > self.config["pwm_max_us"]):
            raise ValueError("舵机快照 PWM 超过范围。")
        expected_held = np.clip((pwm-self.config["center_us"])/self.config["us_per_deg"], -limit, limit)
        if not np.allclose(held, expected_held, rtol=0, atol=1e-10):
            raise ValueError("舵机快照的 PWM 与保持角度不一致。")
        self._time_s, self._next_pwm_index = time_s, int(index)
        self._command_deg, self._held_deg = command, held
        self._actual_deg, self._pwm_us = actual, pwm

    def observation(self) -> dict[str, Any]:
        observation = self.capture_state()
        observation["next_pwm_time_s"] = self.next_pwm_time_s
        return observation


def estimated_control_loads(state: AeroState, aero_config: Mapping[str, float],
                            servo_actual_deg: Any, flight_config: Mapping[str, Any]) -> AeroLoads:
    """由实际四舵角产生聚合增量载荷，所有系数均为未标定工程估计。

    力先按局部 Y/Z 方向给出，再投影到垂直相对来流的平面；只有额外
    阻力消耗平动能。力矩是独立聚合导数，不是四片尾翼 r×F 的替代计算。
    """
    servo = _angles(servo_actual_deg, "servo_actual_deg")
    derivative = _section(flight_config, "actuation", _ACTUATION_KEYS)
    try:
        rho = _number(aero_config["rho_kg_m3"], "rho_kg_m3", positive=True)
        area = _number(aero_config["reference_area_m2"], "reference_area_m2", positive=True)
        length = _number(aero_config["reference_length_m"], "reference_length_m", positive=True)
    except KeyError as exc:
        raise ValueError(f"气动配置缺少 {exc.args[0]}。") from exc
    # Inverse of Guidedemo Autopilot::mix; servo angles and controller outputs are degrees.
    p_deg = (servo[0]+servo[1]-servo[2]-servo[3])/4.
    y_deg = (servo[0]-servo[1]+servo[2]-servo[3])/4.
    r_deg = (servo[0]-servo[1]-servo[2]+servo[3])/4.
    p, y, r = np.radians([p_deg, y_deg, r_deg])
    relative = state.velocity_W_mps-state.wind_W_mps
    speed = float(np.linalg.norm(relative))
    details = {
        "model": "estimated_aggregate_control_increment",
        "classification": "engineering_estimate", "installation_mapping": "simulation_convention",
        "servo_actual_deg": servo.tolist(),
        "effective_angles_deg": {"pitch": float(p_deg), "yaw": float(y_deg), "roll": float(r_deg)},
    }
    if speed < 1e-7 or (p == 0 and y == 0 and r == 0):
        return AeroLoads(np.zeros(3), np.zeros(3), details=details)
    direction = relative/speed
    qS = .5*rho*speed*speed*area
    normal_W = state.rotation_WL @ np.array([0., derivative["normal_per_rad"]*y,
                                             derivative["normal_per_rad"]*p])
    transverse_W = normal_W-np.dot(normal_W, direction)*direction
    force = qS*(transverse_W-derivative["drag_per_rad2"]*(p*p+y*y+r*r)*direction)
    torque = qS*length*(state.rotation_WL @ np.array([
        derivative["roll_moment_per_rad"]*r,
        -derivative["pitch_moment_per_rad"]*p,
        derivative["yaw_moment_per_rad"]*y,
    ]))
    if not np.isfinite(force).all() or not np.isfinite(torque).all():
        raise ValueError("估计舵效产生非有限载荷。")
    return AeroLoads(force, torque, details=details)
