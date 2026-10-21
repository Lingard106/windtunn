"""桌面 RGB 预览使用的 OpenMV LAB 与曝光近似设置。"""
from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache

import cv2
import numpy as np

try:
    from .camera_optics import apply_green_bloom
except ImportError:
    from camera_optics import apply_green_bloom


DEFAULT_LAB = (31, 48, -49, 10, 21, 47)


@lru_cache(maxsize=32)
def _exposure_table(exposure_us):
    """8位颜色只有256个输入值，预先查表避免每帧创建多份浮点整图。"""
    return np.rint(np.clip(np.arange(256)*exposure_us/3000., 0, 255)).astype(np.uint8)


@dataclass(frozen=True)
class VisionSettings:
    """冻结的视觉预览参数；曝光只对 RGB 显示图做简单亮度缩放。"""

    lab: tuple[int, int, int, int, int, int] = DEFAULT_LAB
    exposure_us: int = 3000
    profile: str = "custom"
    bloom_radius_px: int = 0
    bloom_gain: float = 1.0
    aim_point_px: tuple[int, int] = (130, 180)
    dart_reference_px: tuple[int, int] = (130, 120)

    def __post_init__(self) -> None:
        try:
            values = tuple(self.lab)
        except TypeError as exc:
            raise ValueError("LAB 必须包含 6 个整数边界") from exc
        if len(values) != 6 or any(isinstance(value, bool) or not isinstance(value, int)
                                   for value in values):
            raise ValueError("LAB 必须包含 6 个整数边界")

        l_min, l_max, a_min, a_max, b_min, b_max = values
        if not 0 <= l_min <= l_max <= 100:
            raise ValueError("LAB 的 L 范围必须有序且位于 0..100")
        if not -128 <= a_min <= a_max <= 127:
            raise ValueError("LAB 的 a 范围必须有序且位于 -128..127")
        if not -128 <= b_min <= b_max <= 127:
            raise ValueError("LAB 的 b 范围必须有序且位于 -128..127")
        if isinstance(self.exposure_us, bool) or not isinstance(self.exposure_us, int):
            raise ValueError("曝光必须是整数微秒")
        if not 100 <= self.exposure_us <= 11000:
            raise ValueError("曝光必须位于 100..11000 微秒")
        if not isinstance(self.profile, str) or not self.profile:
            raise ValueError("profile 必须是非空字符串")
        if isinstance(self.bloom_radius_px, bool) or not isinstance(self.bloom_radius_px, int) or not 0 <= self.bloom_radius_px <= 12:
            raise ValueError("光晕半径必须是 0..12 像素的整数")
        if (isinstance(self.bloom_gain, bool) or not isinstance(self.bloom_gain, (int, float))
                or not np.isfinite(self.bloom_gain) or not 0 <= self.bloom_gain <= 8):
            raise ValueError("光晕增益必须位于 0..8")

        # 即使调用者传入 list，冻结对象内部也只保存不可变 tuple。
        object.__setattr__(self, "lab", values)
        for name in ("aim_point_px", "dart_reference_px"):
            try:
                point = tuple(getattr(self, name))
            except TypeError as exc:
                raise ValueError(f"{name} 必须包含两个整数像素坐标") from exc
            if len(point) != 2 or any(isinstance(value, bool) or not isinstance(value, (int, np.integer))
                                      for value in point):
                raise ValueError(f"{name} 必须包含两个整数像素坐标")
            if not (0 <= point[0] < 320 and 0 <= point[1] < 240):
                raise ValueError(f"{name} 必须位于 320×240 图像内（x:0..319，y:0..239）")
            object.__setattr__(self, name, tuple(int(value) for value in point))

    @classmethod
    def for_profile(cls, name: str) -> "VisionSettings":
        """返回桌面仿真建议值或原 OpenMV 阈值。"""
        if name == "simulation":
            return cls(lab=(15, 95, -95, -20, 8, 95), exposure_us=2500,
                       profile="simulation", bloom_radius_px=4, bloom_gain=2.)
        if name == "openmv":
            return cls(lab=DEFAULT_LAB, exposure_us=3000, profile="openmv")
        raise ValueError(f"未知视觉预设: {name!r}")

    def as_dict(self) -> dict[str, object]:
        """返回可直接写入 JSON 的独立参数快照。"""
        return {"lab": list(self.lab), "exposure_us": self.exposure_us,
                "profile": self.profile, "bloom_radius_px": self.bloom_radius_px,
                "bloom_gain": self.bloom_gain, "aim_point_px": list(self.aim_point_px),
                "dart_reference_px": list(self.dart_reference_px)}

    def prepare_image(self, rgb):
        """先近似可见绿光的像素扩散，再做曝光显示近似；不读取目标坐标。"""
        image = apply_green_bloom(rgb, self.bloom_radius_px, self.bloom_gain)
        return self.apply_exposure(image)

    def apply_exposure(self, rgb: np.ndarray) -> np.ndarray:
        """以 exposure_us/3000 缩放 uint8 RGB 亮度；这是显示近似，不模拟真实 ISP。"""
        image = np.asarray(rgb)
        if image.ndim != 3 or image.shape[2] != 3:
            raise ValueError("曝光近似要求 H×W×3 RGB 图像")
        if image.dtype == np.uint8:
            return cv2.LUT(image, _exposure_table(self.exposure_us))
        image = image.astype(np.float32, copy=False)
        scale = self.exposure_us / 3000.0
        return np.rint(np.clip(image * scale, 0, 255)).astype(np.uint8)
