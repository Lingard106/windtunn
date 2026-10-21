"""为窗口与命令行统一选择场地、镖体、发射预设和数字配置。

本模块只决定读取哪些文件，不计算运动。直接构造 DemoSimulation 时仍可
不使用任何默认资产，这样原有小型测试与无场地参数求解保持独立。
"""

from dataclasses import dataclass
import json
from pathlib import Path

if __package__:
    from .field_scene import available_scenes, resolve_scene_manifest
else:
    from field_scene import available_scenes, resolve_scene_manifest


PROJECT = Path(__file__).resolve().parents[1]
DEFAULT_DART = PROJECT / "assets/dart_shenda/manifest.json"
DEFAULT_2027_LAUNCH = PROJECT / "profiles/field_2027_fixed_target.json"


@dataclass(frozen=True)
class StartupPaths:
    """启动输入：路径与数字分开，explicit_values 保存建模后仍需采用的覆盖项。"""

    config: Path | None
    scene: Path | None
    dart: Path | None
    launch: Path | None
    explicit_values: dict


def _optional_path(explicit, default: Path, label: str) -> Path | None:
    """默认资源缺失时回退；明确指定的文件缺失时给出中文错误。"""
    if explicit is None:
        return default.resolve() if default.is_file() else None
    path = Path(explicit).expanduser().resolve()
    if not path.is_file():
        raise ValueError(f"找不到{label}：{path}")
    return path


def resolve_startup_paths(config=None, scene=None, dart=None, launch=None) -> StartupPaths:
    """按已登记的 2027 场地选择预设；其他场地需显式指定发射方案。

    首次构造时模型/方案会设置质量和发射基线。调用方随后应用覆盖项：显式
    --config 使用实际出现的字段；自动重放使用标定时保存的物性与空气参数。
    自动重放的起点高度、角度和速度仍以同一份发射方案为准。
    """
    scene_path = resolve_scene_manifest(scene)
    dart_path = _optional_path(dart, DEFAULT_DART, "镖体清单")
    scene_presets = {}
    for label, path in available_scenes().items():
        if label in {"2027", "2027完整"}:
            scene_presets[path.resolve()] = DEFAULT_2027_LAUNCH
    default_launch = scene_presets.get(scene_path.resolve()) if scene_path is not None else None
    launch_path = (_optional_path(launch, default_launch or DEFAULT_2027_LAUNCH, "发射方案")
                   if launch is not None or default_launch is not None else None)
    explicit_values = {}
    config_path = None
    try:
        if config is not None:
            config_path = _optional_path(config, Path("unused"), "数字配置")
            explicit_values = json.loads(config_path.read_text(encoding="utf-8"))
            if not isinstance(explicit_values, dict):
                raise ValueError("数字配置必须是 JSON 对象")
        elif launch_path is not None:
            # 标定报告记录了当时完整气动配置，重放必须继续使用同一组系数。
            raw = json.loads(launch_path.read_text(encoding="utf-8"))
            saved = raw.get("calibration", {}).get("config")
            if saved:
                candidate = Path(saved)
                if not candidate.is_absolute():
                    candidate = launch_path.parent / candidate
                config_path = _optional_path(candidate, Path("unused"), "发射预设对应的数字配置")
                saved_values = json.loads(config_path.read_text(encoding="utf-8"))
                if not isinstance(saved_values, dict):
                    raise ValueError("标定数字配置必须是 JSON 对象")
                # 例如用 100 g 配置求解后，不能在重放时又被 250 g CAD 基线覆盖。
                # 发射量由方案负责，其余保存量覆盖清单基线，保持求解与重放一致。
                launch_keys = {"speed_mps", "pitch_deg", "yaw_deg", "launch_height_m"}
                explicit_values = {key: value for key, value in saved_values.items() if key not in launch_keys}
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"启动配置读取失败：{error}") from error
    return StartupPaths(config_path, scene_path, dart_path, launch_path, explicit_values)
