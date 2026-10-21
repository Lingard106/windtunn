"""可替换的无状态气动接口；当前内置模型均不包含 CFD 数据。

状态和载荷都沿世界轴，载荷作用于重心，单位为 N 与 N·m。模型局部轴
为 X 前、Y 左、Z 上；诊断攻角/侧滑角采用航空 B 系 X 前、Y 右、Z 下。
自定义后端必须无状态，复现所需参数由 config 和 metadata 完整记录。
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
from numbers import Real
from pathlib import Path
from typing import Any, Mapping, Protocol, runtime_checkable

import numpy as np


DEFAULT_AERO_PARAMETERS_PATH = Path(__file__).resolve().parents[1] / "profiles" / "aero_default.json"
_PARAMETER_KEYS = (
    "drag_coefficient", "normal_coefficient", "stability_coefficient",
    "angular_damping_coefficient", "rho_kg_m3",
)


def _finite_array(value: Any, shape: tuple[int, ...], label: str) -> np.ndarray:
    if np.iscomplexobj(value):
        raise ValueError(f"{label} 必须是实数数组，形状为 {shape}。")
    try:
        array = np.array(value, dtype=float, copy=True)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{label} 必须是有限数字数组，形状为 {shape}。") from exc
    if array.shape != shape or not np.isfinite(array).all():
        raise ValueError(f"{label} 必须是有限数字数组，形状为 {shape}。")
    return array


def _json_copy(value: Any, label: str) -> Any:
    try:
        return json.loads(json.dumps(value, allow_nan=False, ensure_ascii=False))
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{label} 必须可以保存为不含 NaN/Inf 的 JSON。") from exc


@dataclass(frozen=True)
class AeroState:
    """一次载荷计算的输入；t_s 是积分前的计算时刻。

    rotation_WL 把模型局部向量转为世界向量；wind_W_mps 是空气的世界速度。
    构造时复制数组并设为只读，防止后端意外修改传入的仿真状态。
    """

    t_s: float
    velocity_W_mps: np.ndarray
    rotation_WL: np.ndarray
    omega_W_radps: np.ndarray
    wind_W_mps: np.ndarray

    def __post_init__(self):
        if isinstance(self.t_s, bool) or not isinstance(self.t_s, Real) or not math.isfinite(self.t_s):
            raise ValueError("AeroState.t_s 必须是有限数字。")
        object.__setattr__(self, "t_s", float(self.t_s))
        for name, shape in (("velocity_W_mps", (3,)), ("rotation_WL", (3, 3)),
                            ("omega_W_radps", (3,)), ("wind_W_mps", (3,))):
            array = _finite_array(getattr(self, name), shape, f"AeroState.{name}")
            array.flags.writeable = False
            object.__setattr__(self, name, array)


@dataclass(frozen=True)
class AeroLoads:
    """重心处的世界载荷；details 应为可保存的 JSON 字典。"""

    force_W_N: np.ndarray
    torque_W_Nm: np.ndarray
    domain_status: str = "unvalidated"
    details: dict[str, Any] = field(default_factory=dict)


@runtime_checkable
class AeroModel(Protocol):
    """无状态后端协议；metadata 返回来源、版本及真实生效参数。"""

    def evaluate(self, state: AeroState, config: Mapping[str, float]) -> AeroLoads: ...

    def metadata(self, config: Mapping[str, float]) -> dict[str, Any]: ...


def _default_parameter_document() -> dict[str, Any]:
    try:
        document = json.loads(DEFAULT_AERO_PARAMETERS_PATH.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"无法读取默认气动参数 {DEFAULT_AERO_PARAMETERS_PATH}：{exc}") from exc
    if (not isinstance(document, dict) or isinstance(document.get("schema_version"), bool)
            or document.get("schema_version") != 1):
        raise ValueError("默认气动参数的 schema_version 必须为 1。")
    if document.get("model") != "estimated_passive":
        raise ValueError("默认气动参数的 model 必须为 estimated_passive。")
    if not isinstance(document.get("parameter_version"), str) or not document["parameter_version"].strip():
        raise ValueError("默认气动参数缺少 parameter_version。")
    parameters = document.get("parameters")
    if not isinstance(parameters, dict) or set(parameters) != set(_PARAMETER_KEYS):
        raise ValueError("默认气动参数必须恰好包含四个系数与 rho_kg_m3。")
    for key in _PARAMETER_KEYS:
        value = parameters[key]
        if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
            raise ValueError(f"默认气动参数 {key} 必须是有限数字。")
        if value < 0 or (key == "rho_kg_m3" and value == 0):
            raise ValueError(f"默认气动参数 {key} 必须非负，密度必须大于零。")
        parameters[key] = float(value)
    if not isinstance(document.get("provenance"), dict) or not document["provenance"]:
        raise ValueError("默认气动参数必须记录 provenance 来源。")
    return _json_copy(document, "默认气动参数")


def load_default_aero_parameters() -> dict[str, float]:
    """读取并校验五项默认数字；返回新字典，调用方的配置覆盖应在此之后。"""
    return dict(_default_parameter_document()["parameters"])


class EstimatedPassiveAero:
    """沿用既有 passive_aero 的估计公式与运算次序，不作真实性升级。"""

    __slots__ = ()

    def evaluate(self, state: AeroState, config: Mapping[str, float]) -> AeroLoads:
        relative = np.asarray(state.velocity_W_mps) - np.asarray(state.wind_W_mps)
        speed = float(np.linalg.norm(relative))
        if speed < 1e-7:
            return AeroLoads(np.zeros(3), np.zeros(3))
        direction = relative / speed
        forward = state.rotation_WL[:, 0]
        lateral = direction - np.dot(direction, forward) * forward
        dynamic_pressure = .5 * config["rho_kg_m3"] * speed * speed
        area = config["reference_area_m2"]
        length = config["reference_length_m"]
        force = -dynamic_pressure * area * (
            config["drag_coefficient"] * direction + config["normal_coefficient"] * lateral)
        torque = dynamic_pressure * area * length * (
            config["stability_coefficient"] * np.cross(forward, direction)
            - config["angular_damping_coefficient"] * length / (2 * speed) * np.asarray(state.omega_W_radps))
        return AeroLoads(force, torque)

    def metadata(self, config: Mapping[str, float]) -> dict[str, Any]:
        document = _default_parameter_document()
        keys = (*_PARAMETER_KEYS, "reference_area_m2", "reference_length_m")
        return _json_copy({
            "model": "estimated_passive",
            "parameter_version": document["parameter_version"],
            "parameter_file": str(DEFAULT_AERO_PARAMETERS_PATH),
            "parameters": {key: config[key] for key in keys},
            "provenance": document["provenance"],
            "domain_status": "unvalidated",
            "load_frame": "world",
            "moment_reference": "center_of_mass",
            "local_axes": "X forward, Y left, Z up",
            "force_unit": "N", "torque_unit": "N*m",
        }, "气动模型元数据")


class NoAerodynamics:
    """关闭气动；重力仍由仿真引擎处理。"""

    __slots__ = ()

    def evaluate(self, state: AeroState, config: Mapping[str, float]) -> AeroLoads:
        return AeroLoads(np.zeros(3), np.zeros(3), domain_status="disabled")

    def metadata(self, config: Mapping[str, float]) -> dict[str, Any]:
        return {
            "model": "none", "parameter_version": "none-v1", "parameters": {},
            "provenance": {"classification": "disabled", "description": "明确关闭气动载荷。"},
            "domain_status": "disabled", "load_frame": "world",
            "moment_reference": "center_of_mass", "force_unit": "N", "torque_unit": "N*m",
        }


def resolve_aero_model(name_or_object: str | AeroModel | None = None) -> AeroModel:
    """解析内置名称或无状态自定义对象；未知名称在建立仿真前失败。"""
    if name_or_object is None or (isinstance(name_or_object, str) and name_or_object == "estimated_passive"):
        return EstimatedPassiveAero()
    if isinstance(name_or_object, str):
        if name_or_object == "none":
            return NoAerodynamics()
        raise ValueError(f"未知气动模型 {name_or_object!r}；可选 estimated_passive 或 none。")
    if isinstance(name_or_object, type) or not all(
            callable(getattr(name_or_object, method, None)) for method in ("evaluate", "metadata")):
        raise TypeError("自定义气动模型必须是实现 evaluate(state, config) 和 metadata(config) 的无状态对象。")
    return name_or_object


def evaluate_aerodynamics(model: AeroModel, state: AeroState,
                          config: Mapping[str, float]) -> tuple[AeroLoads, dict[str, Any]]:
    """计算并校验载荷；返回独立副本和可直接保存为 JSON 的工况诊断。"""
    raw = model.evaluate(state, config)
    if not isinstance(raw, AeroLoads):
        raise TypeError("气动模型 evaluate 必须返回 AeroLoads。")
    force = _finite_array(raw.force_W_N, (3,), "AeroLoads.force_W_N")
    torque = _finite_array(raw.torque_W_Nm, (3,), "AeroLoads.torque_W_Nm")
    if not isinstance(raw.domain_status, str) or not raw.domain_status.strip():
        raise ValueError("AeroLoads.domain_status 必须是非空字符串。")
    if not isinstance(raw.details, dict):
        raise ValueError("AeroLoads.details 必须是 JSON 字典。")
    details = _json_copy(raw.details, "AeroLoads.details")
    loads = AeroLoads(force, torque, raw.domain_status, details)
    relative = state.velocity_W_mps - state.wind_W_mps
    relative_local = state.rotation_WL.T @ relative
    speed = float(np.linalg.norm(relative))
    if speed < 1e-7:
        alpha, beta = 0., 0.
    else:
        x, y, z = relative_local
        alpha = float(np.degrees(np.arctan2(-z, x)))
        beta = float(np.degrees(np.arctan2(-y, np.hypot(x, z))))
    diagnostics = {
        "t_s": state.t_s, "airspeed_mps": speed,
        "alpha_deg": alpha, "beta_deg": beta,
        "relative_velocity_W_mps": relative.tolist(),
        "relative_velocity_L_mps": relative_local.tolist(),
        "omega_W_radps": state.omega_W_radps.tolist(),
        "force_W_N": force.tolist(), "torque_W_Nm": torque.tolist(),
        "domain_status": loads.domain_status, "details": _json_copy(details, "气动诊断 details"),
    }
    return loads, _json_copy(diagnostics, "气动工况诊断")
