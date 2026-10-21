"""读取已经转换好的场地网格；源文件转换由独立工具完成。

这里的网格仅负责外观。所有顶点都应已经使用米、世界 Z 轴向上，
本模块不会猜测单位，也不会为场地建立碰撞体或改变镖体动力学。
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET


_ASSET_ROOT = Path(__file__).resolve().parents[1] / "assets"
_2027_LITE_MANIFEST = _ASSET_ROOT / "field_2027_lite" / "manifest.json"
_2027_FULL_MANIFEST = _ASSET_ROOT / "field_2027" / "manifest.json"


def _preferred_scene_manifest() -> Path:
    """默认只加载当前维护的 2027 场地，优先轻量版。"""
    candidates = (_2027_LITE_MANIFEST, _2027_FULL_MANIFEST)
    return next((path for path in candidates if path.is_file()), _2027_FULL_MANIFEST)


DEFAULT_SCENE_MANIFEST = _preferred_scene_manifest()


def available_scenes() -> dict[str, Path]:
    """只登记当前维护的场地；归档或自定义清单仍可通过显式路径加载。"""
    candidates = (("2027", _2027_LITE_MANIFEST if _2027_LITE_MANIFEST.is_file()
                   else _2027_FULL_MANIFEST),
                  ("2027完整", _2027_FULL_MANIFEST))
    return {label: path.resolve() for label, path in candidates if path.is_file()}


def _numbers(value: Any, length: int, label: str) -> tuple[float, ...]:
    """把 JSON 数组转成有限浮点数，避免无效参数进入 MuJoCo。"""
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f"{label} 必须包含 {length} 个数字。")
    if any(isinstance(item, bool) or not isinstance(item, (int, float))
           or not math.isfinite(item) for item in value):
        raise ValueError(f"{label} 必须全部是有限数字。")
    return tuple(float(item) for item in value)


def resolve_scene_manifest(explicit_path: Path | str | None = None) -> Path | None:
    """启动时自动寻找默认场地；用户明确指定错误路径时直接报错。"""
    if explicit_path is None:
        return DEFAULT_SCENE_MANIFEST if DEFAULT_SCENE_MANIFEST.is_file() else None
    path = Path(explicit_path).expanduser().resolve()
    if not path.is_file():
        raise ValueError(f"找不到场地清单：{path}")
    return path


@dataclass(frozen=True)
class FieldMesh:
    """一个只用于显示的网格及其颜色。"""

    name: str
    file: Path
    rgba: tuple[float, ...]


@dataclass(frozen=True)
class FieldScene:
    """场地清单的已验证内容，和数字气动配置分开保存。"""

    manifest: Path
    source: dict[str, Any]
    bounds_min: tuple[float, ...]
    bounds_max: tuple[float, ...]
    meshes: tuple[FieldMesh, ...]
    overview: dict[str, Any]
    target_meshes: tuple[FieldMesh, ...] = ()
    target_assembly: dict[str, Any] | None = None

    @property
    def label(self) -> str:
        """窗口中明确显示模型年份，避免把旧场地当成当前规则场地。"""
        season = str(self.source.get("season", "年份未标明"))
        if self.source.get("visual_detail") == "lite":
            detail = (" · 整体轻量版" if self.source.get("simplification_scope") == "global"
                      else " · 远景轻量版")
        else:
            detail = ""
        return f"{season} 场地 · {self.source['name']}{detail} · 仅视觉模型"

    def metadata(self) -> dict[str, Any]:
        """生成可以随截图保存的来源记录。"""
        return {"mode": "cad_visual", "manifest": str(self.manifest),
                "source": dict(self.source), "mesh_count": len(self.meshes),
                "target_assembly": self.target_assembly}

    def xml_assets(self) -> str:
        """声明 MuJoCo 的 mesh 资源；ET 负责正确转义文件路径。"""
        elements = []
        for index, mesh in enumerate(self.meshes):
            element = ET.Element("mesh", name=f"field_mesh_{index}",
                                 file=mesh.file.as_posix(), inertia="shell")
            elements.append(ET.tostring(element, encoding="unicode"))
        return "\n".join(elements)

    def xml_geoms(self) -> str:
        """把网格放在世界中，两个碰撞掩码同时置零以关闭碰撞。"""
        elements = []
        for index, mesh in enumerate(self.meshes):
            element = ET.Element("geom", name=f"field_visual_{index}",
                                 type="mesh", mesh=f"field_mesh_{index}",
                                 rgba=" ".join(str(value) for value in mesh.rgba),
                                 contype="0", conaffinity="0", group="2", density="0")
            elements.append(ET.tostring(element, encoding="unicode"))
        return "\n".join(elements)

    def xml_target_assets(self) -> str:
        """声明由同一源 STL 分离的滑动检测模块，顶点仍使用世界坐标。"""
        return "\n".join(ET.tostring(ET.Element("mesh", name=f"target_mesh_{index}",
            file=mesh.file.as_posix(), inertia="shell"), encoding="unicode")
            for index, mesh in enumerate(self.target_meshes))

    def xml_target_geoms(self) -> str:
        """由仿真放入同一随动靶体；其网格不增加碰撞或质量。"""
        return "\n".join(ET.tostring(ET.Element("geom", name=f"target_visual_{index}",
            type="mesh", mesh=f"target_mesh_{index}",
            rgba=" ".join(str(value) for value in mesh.rgba),
            contype="0", conaffinity="0", density="0", group="2"), encoding="unicode")
            for index, mesh in enumerate(self.target_meshes))


def _assembly_file(root: Path, value: Any) -> Path:
    """分离资产路径必须相对其清单，允许复用同源场地的兄弟文件夹。"""
    if not isinstance(value, str) or not value.strip():
        raise ValueError("target_assembly 网格路径不能为空。")
    relative = Path(value)
    if relative.is_absolute() or relative.drive or relative.root:
        raise ValueError("target_assembly 文件路径必须相对清单。")
    file = (root / relative).resolve()
    if not file.is_file():
        raise ValueError(f"target_assembly 文件不存在：{file}")
    return file


def _load_target_assembly(raw: Any, manifest: Path, meshes: list[FieldMesh]):
    """替换受影响的静态余量块，同时读取已从这些块移除的活动模块。"""
    if raw is None:
        return meshes, (), None
    if not isinstance(raw, dict) or raw.get("variant") not in ("full", "lite"):
        raise ValueError("target_assembly 需要 manifest 和 full/lite variant。")
    file = _assembly_file(manifest.parent, raw.get("manifest"))
    try:
        assembly = json.loads(file.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"无法读取 target_assembly：{exc}") from exc
    if not isinstance(assembly, dict) or assembly.get("schema_version") != 1:
        raise ValueError("target_assembly schema_version 必须为 1。")
    variants = assembly.get("variants")
    if not isinstance(variants, dict):
        raise ValueError("target_assembly variants 必须是对象。")
    variant = variants.get(raw["variant"])
    if not isinstance(variant, dict):
        raise ValueError("target_assembly 缺少所选 variant。")
    replacements = variant.get("static_replacements")
    parts = variant.get("meshes")
    if not isinstance(replacements, dict) or not replacements or not isinstance(parts, list) or not parts:
        raise ValueError("target_assembly 必须同时给出静态余量替换和随动网格。")
    known = {mesh.name for mesh in meshes}
    if not set(replacements).issubset(known):
        raise ValueError("target_assembly 静态替换引用了不存在的场地网格。")
    for replacement in replacements.values():
        _assembly_file(file.parent, replacement)
    remaining = []
    for mesh in meshes:
        replacement = replacements.get(mesh.name)
        path = mesh.file if replacement is None else _assembly_file(file.parent, replacement)
        if path.suffix.lower() not in (".obj", ".stl", ".msh"):
            raise ValueError("target_assembly 静态网格格式不受支持。")
        remaining.append(FieldMesh(mesh.name, path, mesh.rgba))
    moving = []
    for index, item in enumerate(parts):
        if not isinstance(item, dict):
            raise ValueError("target_assembly 随动网格必须是对象。")
        path = _assembly_file(file.parent, item.get("file"))
        if path.suffix.lower() not in (".obj", ".stl", ".msh"):
            raise ValueError("target_assembly 随动网格格式不受支持。")
        rgba = _numbers(item.get("rgba", [.58, .62, .66, 1.]), 4, "target_assembly.rgba")
        if any(not 0 <= channel <= 1 for channel in rgba):
            raise ValueError("target_assembly 颜色必须在 0 到 1 之间。")
        moving.append(FieldMesh(str(item.get("name", f"part_{index}")), path, rgba))
    metadata = {"manifest": str(file), "variant": raw["variant"],
                "mesh_count": len(moving), "static_replacements": sorted(replacements),
                "source": assembly.get("source", {}),
                "coordinates": "原始 STL 世界坐标；只沿场地宽边世界 Y 平移；静态余量已排除该模块"}
    return remaining, tuple(moving), metadata


def load_field_scene(path: Path | str) -> FieldScene:
    """读取 manifest.json；所有网格路径相对清单，坐标已经是世界坐标。"""
    manifest = Path(path).expanduser().resolve()
    try:
        raw = json.loads(manifest.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"无法读取场地清单 {manifest}：{exc}") from exc
    if not isinstance(raw, dict) or raw.get("schema_version") != 1:
        raise ValueError("场地清单的 schema_version 必须为 1。")
    source = raw.get("source")
    if not isinstance(source, dict) or not isinstance(source.get("name"), str) or not source["name"].strip():
        raise ValueError("场地清单缺少 source.name 来源名称。")
    bounds = raw.get("bounds_m")
    if not isinstance(bounds, dict):
        raise ValueError("场地清单缺少以米为单位的 bounds_m。")
    lower = _numbers(bounds.get("min"), 3, "bounds_m.min")
    upper = _numbers(bounds.get("max"), 3, "bounds_m.max")
    if any(low > high for low, high in zip(lower, upper)):
        raise ValueError("场地包围盒的 min 不能大于 max。")
    raw_meshes = raw.get("meshes")
    if not isinstance(raw_meshes, list) or not raw_meshes:
        raise ValueError("场地清单必须包含至少一个网格。")
    meshes = []
    for index, item in enumerate(raw_meshes):
        if not isinstance(item, dict) or not isinstance(item.get("file"), str):
            raise ValueError(f"第 {index + 1} 个场地网格缺少 file 路径。")
        relative = Path(item["file"])
        if relative.is_absolute():
            raise ValueError("场地网格路径必须相对 manifest.json。")
        file = (manifest.parent / relative).resolve()
        if not file.is_file():
            raise ValueError(f"场地网格文件不存在：{file}")
        if file.suffix.lower() not in (".obj", ".stl", ".msh"):
            raise ValueError(f"场地网格格式不受支持：{file.suffix}")
        rgba = _numbers(item.get("rgba", [.65, .65, .65, 1]), 4, f"meshes[{index}].rgba")
        if any(not 0 <= number <= 1 for number in rgba):
            raise ValueError("场地颜色 rgba 必须在 0 到 1 之间。")
        meshes.append(FieldMesh(str(item.get("name", f"mesh_{index}")), file, rgba))
    raw_overview = raw.get("overview", {})
    if not isinstance(raw_overview, dict):
        raise ValueError("场地 overview 必须是对象。")
    center = [(low + high) / 2 for low, high in zip(lower, upper)]
    span = max(high - low for low, high in zip(lower, upper))
    overview = {"lookat": _numbers(raw_overview.get("lookat", center), 3, "overview.lookat")}
    for key, default in (("distance", max(5., span * 1.3)), ("azimuth", 135.), ("elevation", -45.)):
        overview[key] = _numbers([raw_overview.get(key, default)], 1, f"overview.{key}")[0]
    if overview["distance"] <= 0:
        raise ValueError("场地全景相机距离必须大于零。")
    meshes, target_meshes, assembly = _load_target_assembly(raw.get("target_assembly"), manifest, meshes)
    return FieldScene(manifest, dict(source), lower, upper, tuple(meshes), overview, target_meshes, assembly)
