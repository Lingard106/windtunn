"""读取镖体可视网格、几何估计物理参数和独立发射方案。

长度统一用米，质量用千克，转动惯量用千克·米²。镖体坐标约定是
X 向镖头、Y 向左、Z 向上，局部原点放在所采用的估计重心。
网格只负责外观；质量、惯量及气动参考量分别填写，不能从画面推断。
"""

from __future__ import annotations

from dataclasses import dataclass
import copy
import json
import math
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET

import numpy as np


def _numbers(value: Any, length: int, label: str) -> tuple[float, ...]:
    """验证固定长度的 JSON 数组；拒绝布尔值、无穷大和非数字。"""
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f"{label} 必须包含 {length} 个数字。")
    if any(isinstance(item, bool) or not isinstance(item, (int, float))
           or not math.isfinite(item) for item in value):
        raise ValueError(f"{label} 必须全部是有限数字。")
    return tuple(float(item) for item in value)


def _positive(value: Any, label: str) -> float:
    """读取必须大于零的有限标量。"""
    number = _numbers([value], 1, label)[0]
    if number <= 0:
        raise ValueError(f"{label} 必须大于零。")
    return number


def _read_json(path: Path | str, label: str) -> tuple[Path, dict[str, Any]]:
    """读取带版本号的清单；文件解析失败时给出可读的中文原因。"""
    resolved = Path(path).expanduser().resolve()
    try:
        raw = json.loads(resolved.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"无法读取{label} {resolved}：{exc}") from exc
    if not isinstance(raw, dict) or raw.get("schema_version") != 1:
        raise ValueError(f"{label}的 schema_version 必须为 1。")
    return resolved, raw


@dataclass(frozen=True)
class DartMesh:
    """一块已经转换为米制镖体坐标的可视网格。"""

    name: str
    file: Path
    rgba: tuple[float, ...]


@dataclass(frozen=True)
class DartModel:
    """已验证的镖体资料；物理属性来自清单并记录其估计来源。"""

    manifest: Path
    source: dict[str, Any]
    meshes: tuple[DartMesh, ...]
    bounds_min: tuple[float, ...]
    bounds_max: tuple[float, ...]
    mass_kg: float
    full_inertia: tuple[float, ...]
    reference_length_m: float
    reference_area_m2: float
    provenance: dict[str, Any]
    camera_position: tuple[float, ...]
    camera_xyaxes: tuple[float, ...]
    camera_fovy_deg: float
    nose_tip: tuple[float, ...]

    def metadata(self) -> dict[str, Any]:
        """生成可保存的来源记录；声明简化碰撞和未标定气动。"""
        return {
            "mode": "cad_visual_with_estimated_mass_properties",
            "manifest": str(self.manifest), "source": copy.deepcopy(self.source),
            "mesh_count": len(self.meshes), "mass_kg": self.mass_kg,
            "center_of_mass_m": [0., 0., 0.],
            "full_inertia_kg_m2": list(self.full_inertia),
            "reference_length_m": self.reference_length_m,
            "reference_area_m2": self.reference_area_m2,
            "nose_tip_m": list(self.nose_tip),
            "provenance": copy.deepcopy(self.provenance),
            "collision": "简化胶囊碰撞体；CAD 网格关闭碰撞，场地外观不参与碰撞",
            "aerodynamics": "仍使用可配置的演示气动模型，未由 CAD 自动计算气动系数",
        }

    def xml_assets(self) -> str:
        """声明网格文件；XML 库转义路径，避免空格或特殊字符破坏 XML。"""
        elements = []
        for index, mesh in enumerate(self.meshes):
            element = ET.Element("mesh", name=f"dart_mesh_{index}",
                                 file=mesh.file.as_posix(), inertia="shell")
            elements.append(ET.tostring(element, encoding="unicode"))
        return "\n".join(elements)

    def xml_geoms(self) -> str:
        """放置可视网格，显式关闭碰撞及自动质量贡献。"""
        elements = []
        for index, mesh in enumerate(self.meshes):
            element = ET.Element("geom", name=f"dart_visual_{index}",
                                 type="mesh", mesh=f"dart_mesh_{index}",
                                 rgba=" ".join(str(value) for value in mesh.rgba),
                                 contype="0", conaffinity="0", density="0", group="1")
            elements.append(ET.tostring(element, encoding="unicode"))
        return "\n".join(elements)


def load_dart_model(path: Path | str | DartModel) -> DartModel:
    """读取镖体清单；可直接传入已验证对象以复用转换后的资料。

    full_inertia 的六项顺序是 [Ixx, Iyy, Izz, Ixy, Ixz, Iyz]。
    这里接受关于重心的完整对称惯量张量，不能传关于 CAD 世界原点的惯量。
    """
    if isinstance(path, DartModel):
        return path
    manifest, raw = _read_json(path, "镖体清单")
    source = raw.get("source")
    if not isinstance(source, dict) or not isinstance(source.get("name"), str) or not source["name"].strip():
        raise ValueError("镖体清单缺少 source.name。")
    bounds = raw.get("bounds_m")
    if not isinstance(bounds, dict):
        raise ValueError("镖体清单缺少米制 bounds_m。")
    lower = _numbers(bounds.get("min"), 3, "bounds_m.min")
    upper = _numbers(bounds.get("max"), 3, "bounds_m.max")
    if any(low >= high for low, high in zip(lower, upper)):
        raise ValueError("镖体包围盒的各轴 max 必须大于 min。")
    raw_meshes = raw.get("meshes")
    if not isinstance(raw_meshes, list) or not raw_meshes:
        raise ValueError("镖体清单至少需要一个可视网格。")
    meshes = []
    for index, item in enumerate(raw_meshes):
        if not isinstance(item, dict) or not isinstance(item.get("file"), str):
            raise ValueError(f"镖体第 {index + 1} 个网格缺少 file。")
        relative = Path(item["file"])
        if relative.is_absolute():
            raise ValueError("镖体网格路径必须相对 manifest.json。")
        file = (manifest.parent / relative).resolve()
        if not file.is_file() or file.suffix.lower() not in (".obj", ".stl", ".msh"):
            raise ValueError(f"镖体网格不存在或格式不受支持：{file}")
        rgba = _numbers(item.get("rgba", [.7, .7, .7, 1.]), 4, f"meshes[{index}].rgba")
        if any(not 0 <= value <= 1 for value in rgba):
            raise ValueError("镖体网格颜色必须在 0 到 1 之间。")
        meshes.append(DartMesh(str(item.get("name", f"mesh_{index}")), file, rgba))
    physics = raw.get("physics")
    if not isinstance(physics, dict):
        raise ValueError("镖体清单缺少 physics。")
    mass = _positive(physics.get("mass_kg"), "physics.mass_kg")
    center = _numbers(physics.get("center_of_mass_m"), 3, "physics.center_of_mass_m")
    if max(abs(value) for value in center) > 1e-8:
        raise ValueError("本原型要求镖体网格原点位于采用的重心，center_of_mass_m 应为 [0,0,0]。")
    inertia = _numbers(physics.get("full_inertia_kg_m2"), 6, "physics.full_inertia_kg_m2")
    xx, yy, zz, xy, xz, yz = inertia
    principal = np.linalg.eigvalsh([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])
    if principal[0] <= 0 or principal[2] > principal[0] + principal[1] + 1e-12:
        raise ValueError("镖体转动惯量必须正定，并满足主惯量三角关系。")
    length = _positive(physics.get("reference_length_m"), "physics.reference_length_m")
    area = _positive(physics.get("reference_area_m2"), "physics.reference_area_m2")
    provenance = physics.get("provenance", {})
    if not isinstance(provenance, dict):
        raise ValueError("physics.provenance 必须是对象。")
    camera = raw.get("camera", {})
    if not isinstance(camera, dict):
        raise ValueError("camera 必须是对象。")
    camera_position = _numbers(camera.get("position_m", [upper[0] + .01, 0., 0.]), 3, "camera.position_m")
    camera_axes = _numbers(camera.get("xyaxes", [0., -1., 0., 0., 0., 1.]), 6, "camera.xyaxes")
    if np.linalg.norm(np.cross(camera_axes[:3], camera_axes[3:])) < 1e-8:
        raise ValueError("camera.xyaxes 的两个方向必须非零且不能平行。")
    fovy = _positive(camera.get("fovy_deg", 55.), "camera.fovy_deg")
    if not 5 <= fovy <= 150:
        raise ValueError("camera.fovy_deg 必须在 5 到 150 度之间。")
    nose = _numbers(raw.get("nose_tip_m", [upper[0], 0., 0.]), 3, "nose_tip_m")
    return DartModel(manifest, copy.deepcopy(source), tuple(meshes), lower, upper,
                     mass, inertia, length, area, copy.deepcopy(provenance),
                     camera_position, camera_axes, fovy, nose)


@dataclass(frozen=True)
class TargetCollision:
    """固定靶的简化被动接触参数，不代表实际装甲或镖头材料标定。

    half_extents_m 是靶面两个切向的半宽/半高；厚度沿发射侧到背侧延伸。
    solref 的正值格式是 [时间常数秒, 阻尼比]，阻尼比小于 1 可产生
    欠阻尼回弹。它不是可直接填写“恢复系数”的接口，回弹量需要实测标定。
    """

    enabled: bool = False
    half_extents_m: tuple[float, ...] = (.03825, .03325)
    thickness_m: float = .01
    nose_radius_m: float = .0185
    nose_length_m: float = .005
    friction: tuple[float, ...] = (.35, .005, .0001)
    solref: tuple[float, ...] = (.001, .5)
    solimp: tuple[float, ...] = (.9, .95, .001, .5, 2.)
    provenance: str = "接触材料未实测；靶板厚度、镖头半径、摩擦与软接触参数均为演示估计"

    def metadata(self) -> dict[str, Any]:
        """按 JSON 字段公开实际接触设置和静态靶板假设。"""
        return {
            "enabled": self.enabled, "half_extents_m": list(self.half_extents_m),
            "thickness_m": self.thickness_m, "nose_radius_m": self.nose_radius_m,
            "nose_length_m": self.nose_length_m,
            "friction": list(self.friction), "solref": list(self.solref),
            "solimp": list(self.solimp), "provenance": self.provenance,
            "model": "固定薄盒靶板 + 沿镖体X轴的短圆柱触发头/后缩镖身胶囊；MuJoCo 被动软接触，不手写反射或状态吸附",
            "limitations": "靶板固定在世界中，不模拟支架运动、材料破坏或真实弹性形变；软接触只能近似接触耗散与回弹",
            "thickness_direction": "靶前表面为 target.position_W_m，厚度沿发射侧指向靶背侧的 normal_W 延展",
        }


def _load_target_collision(raw: Any) -> TargetCollision:
    """兼容没有碰撞配置的旧方案，并验证接触参数保持耗散、有限且物理可用。"""
    if raw is None:
        return TargetCollision()
    if not isinstance(raw, dict) or not isinstance(raw.get("enabled", False), bool):
        raise ValueError("target.collision 必须是对象，enabled 必须为布尔值。")
    defaults = TargetCollision()
    half = _numbers(raw.get("half_extents_m", defaults.half_extents_m), 2,
                    "target.collision.half_extents_m")
    friction = _numbers(raw.get("friction", defaults.friction), 3, "target.collision.friction")
    solref = _numbers(raw.get("solref", defaults.solref), 2, "target.collision.solref")
    solimp = _numbers(raw.get("solimp", defaults.solimp), 5, "target.collision.solimp")
    if min(half) <= 0 or min(friction) < 0:
        raise ValueError("靶面半尺寸必须为正，摩擦系数不能为负。")
    # 本原型只接受有正阻尼的标准时间常数格式，避免把负阻尼误当恢复系数。
    if min(solref) <= 0:
        raise ValueError("target.collision.solref 使用正值 [时间常数, 阻尼比]，两项必须大于零。")
    d0, dw, width, midpoint, power = solimp
    if not (0 < d0 <= dw < 1 and width > 0 and 0 < midpoint < 1 and power >= 1):
        raise ValueError("target.collision.solimp 必须满足 0<d0<=dw<1、width>0、0<midpoint<1、power>=1。")
    provenance = raw.get("provenance", defaults.provenance)
    if not isinstance(provenance, str) or not provenance.strip():
        raise ValueError("target.collision.provenance 必须说明接触参数来源。")
    return TargetCollision(enabled=raw.get("enabled", False), half_extents_m=half,
        thickness_m=_positive(raw.get("thickness_m", defaults.thickness_m), "target.collision.thickness_m"),
        nose_radius_m=_positive(raw.get("nose_radius_m", defaults.nose_radius_m), "target.collision.nose_radius_m"),
        nose_length_m=_positive(raw.get("nose_length_m", defaults.nose_length_m), "target.collision.nose_length_m"),
        friction=friction, solref=solref, solimp=solimp, provenance=provenance)


@dataclass(frozen=True)
class GuideLight:
    """从原始场地 STL 提取的发光透镜，顶点相对灯孔中心、仍沿世界轴。

    normal_W 记录原灯具的朝外方向，不随靶板法向翻转，也不用于再次旋转
    网格。radius_m 是量测的发光口径半径，不缩放原始穹顶几何。
    """

    center_W_m: tuple[float, ...]
    normal_W: tuple[float, ...]
    radius_m: float
    mesh_file: Path
    provenance: dict[str, Any]

    def metadata(self) -> dict[str, Any]:
        """公开实际透镜来源、口径和网格坐标约定，便于保存画面后复核。"""
        return {
            "shape": "original_stl_lens_cap", "mesh_file": str(self.mesh_file),
            "center_W_m": list(self.center_W_m), "normal_W": list(self.normal_W),
            "radius_m": self.radius_m, "diameter_m": 2*self.radius_m,
            "coordinates": "网格原点为灯孔平面中心；顶点沿世界轴，仅按 center_W_m 平移，不额外旋转或缩放",
            "provenance": copy.deepcopy(self.provenance),
        }


def _load_guide_light(raw: Any, manifest: Path) -> GuideLight | None:
    """可选透镜几何独立于目标碰撞；没有配置时保留旧方案的演示球体。"""
    if raw is None:
        return None
    if not isinstance(raw, dict):
        raise ValueError("guide_light 必须是对象。")
    center = _numbers(raw.get("center_W_m"), 3, "guide_light.center_W_m")
    normal = np.asarray(_numbers(raw.get("normal_W"), 3, "guide_light.normal_W"))
    # 先按最大分量缩放，避免有限但极大/极小的向量在求模时溢出或下溢。
    scale = float(np.max(np.abs(normal)))
    if scale == 0:
        raise ValueError("guide_light.normal_W 不能为零向量。")
    normal = normal/scale
    normal /= np.linalg.norm(normal)
    radius = _positive(raw.get("radius_m"), "guide_light.radius_m")
    value = raw.get("mesh_file")
    if not isinstance(value, str) or not value.strip():
        raise ValueError("guide_light.mesh_file 必须是相对发射方案的网格路径。")
    relative = Path(value)
    if relative.is_absolute() or relative.drive or relative.root:
        raise ValueError("guide_light.mesh_file 必须使用相对路径。")
    file = (manifest.parent / relative).resolve()
    if not file.is_file():
        raise ValueError(f"guide_light.mesh_file 不存在：{file}")
    if file.suffix.lower() not in (".stl", ".obj", ".msh"):
        raise ValueError(f"guide_light.mesh_file 格式不受支持：{file.suffix}")
    provenance = raw.get("provenance", {})
    if not isinstance(provenance, dict):
        raise ValueError("guide_light.provenance 必须是对象。")
    return GuideLight(center, tuple(normal), radius, file, copy.deepcopy(provenance))


@dataclass(frozen=True)
class LaunchProfile:
    """发射重心起点、初始姿态/速度和静止靶平面的几何定义。"""

    manifest: Path
    name: str
    position_W_m: tuple[float, ...]
    pitch_deg: float
    yaw_deg: float
    speed_mps: float
    target_position_W_m: tuple[float, ...]
    target_normal_W: tuple[float, ...]
    target_radius_m: float
    provenance: dict[str, Any]
    target_collision: TargetCollision = TargetCollision()
    guide_light: GuideLight | None = None

    def metadata(self) -> dict[str, Any]:
        """保存几何判定假设；这不是比赛裁判系统的命中认定。"""
        return {
            "manifest": str(self.manifest), "name": self.name,
            "launch": {"position_W_m": list(self.position_W_m),
                       "pitch_deg": self.pitch_deg, "yaw_deg": self.yaw_deg,
                       "speed_mps": self.speed_mps},
            "target": {"position_W_m": list(self.target_position_W_m),
                       "normal_W": list(self.target_normal_W), "radius_m": self.target_radius_m,
                       "collision": self.target_collision.metadata()},
            "provenance": copy.deepcopy(self.provenance),
            "guide_light": self.guide_light.metadata() if self.guide_light is not None else None,
            "evaluation": (
                "启用碰撞：target_contact 记录真实积分接触；target_result 仅评价首次接触前的自由飞行过面，碰撞后不再评价；非官方比赛判定"
                if self.target_collision.enabled else
                "未启用碰撞：镖头首次正向穿过静止靶平面，相邻物理步线性插值后停止；非官方比赛判定"),
            "launch_position_meaning": "position_W_m 是离架瞬间采用的镖体重心世界位置，不模拟架内加速",
        }


def load_launch_profile(path: Path | str | LaunchProfile) -> LaunchProfile:
    """读取独立发射方案；法向量归一化并自动朝发射点到目标的方向。

    目标平面由靶心和法向量确定，radius_m 是该平面中的简化圆形判定半径。
    pitch_deg 向上为正；yaw_deg 从世界 +X 朝 +Y 旋转为正，单位均为度。
    """
    if isinstance(path, LaunchProfile):
        return path
    manifest, raw = _read_json(path, "发射方案")
    launch, target = raw.get("launch"), raw.get("target")
    if not isinstance(launch, dict) or not isinstance(target, dict):
        raise ValueError("发射方案需要 launch 和 target 对象。")
    position = _numbers(launch.get("position_W_m"), 3, "launch.position_W_m")
    if position[2] <= 0:
        raise ValueError("发射重心高度必须大于零。")
    pitch, yaw, speed = _numbers([launch.get("pitch_deg"), launch.get("yaw_deg"),
                                  launch.get("speed_mps")], 3, "发射角度和速度")
    if not -90 < pitch < 90 or speed < 0:
        raise ValueError("发射仰角须介于 -90 与 90 度之间，速度不能为负。")
    target_position = _numbers(target.get("position_W_m"), 3, "target.position_W_m")
    normal = np.array(_numbers(target.get("normal_W"), 3, "target.normal_W"))
    magnitude = float(np.linalg.norm(normal))
    if magnitude < 1e-8:
        raise ValueError("目标平面 normal_W 不能为零向量。")
    normal /= magnitude
    separation = float(normal @ (np.asarray(target_position) - np.asarray(position)))
    if abs(separation) < 1e-6:
        raise ValueError("发射重心不能位于目标平面上。")
    if separation < 0:
        normal = -normal
    radius = _positive(target.get("radius_m"), "target.radius_m")
    provenance = raw.get("provenance", {})
    if not isinstance(provenance, dict):
        raise ValueError("发射方案 provenance 必须是对象。")
    return LaunchProfile(manifest, str(raw.get("name", manifest.stem)), position,
                         pitch, yaw, speed, target_position, tuple(normal), radius,
                         copy.deepcopy(provenance), _load_target_collision(target.get("collision")),
                         _load_guide_light(raw.get("guide_light"), manifest))
