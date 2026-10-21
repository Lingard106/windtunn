"""桌面成像近似；仅从实际 RGB 提取绿色源，处理结果用于显示和仿真识别。"""
from __future__ import annotations

import math
from numbers import Real

import cv2
import numpy as np


def apply_green_bloom(rgb, radius_px: int = 0, gain: float = 1.0) -> np.ndarray:
    """为已渲染的 uint8 RGB 图像添加局部绿色光晕，不改变输入。

    绿色光源仅由像素本身判断：G 必须比 R、B 中较大的值高至少 7，且高于 12。
    对这些像素作椭圆膨胀和轻微高斯柔化，再将放大的绿色通道与原图逐像素取大值。
    这是未经实拍标定的成像预处理，会改变送入识别的像素；不读取场景/目标
    坐标，不改变物理计算或识别算法，不代表真实相机的光度/PSF模型。
    """
    if isinstance(radius_px, bool) or not isinstance(radius_px, (int, np.integer)):
        raise ValueError("radius_px 必须是 0..12 的整数")
    radius = int(radius_px)
    if not 0 <= radius <= 12:
        raise ValueError("radius_px 必须是 0..12 的整数")
    if isinstance(gain, bool) or not isinstance(gain, Real):
        raise ValueError("gain 必须是 0..8 的有限数值")
    strength = float(gain)
    if not math.isfinite(strength) or not 0.0 <= strength <= 8.0:
        raise ValueError("gain 必须是 0..8 的有限数值")

    image = np.asarray(rgb)
    if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
        raise ValueError("rgb 必须是 H×W×3 的 uint8 RGB 图像")

    output = image.copy()
    # 任一开关为零时都保持原图；不创建新光源或改写输入。
    if radius == 0 or strength == 0.0:
        return output

    channels = image.astype(np.int16, copy=False)
    green = channels[:, :, 1]
    dominant = green > (np.maximum(channels[:, :, 0], channels[:, :, 2]) + 6)
    source = dominant & (green > 12)
    if not np.any(source):
        return output

    # 用源像素的实际 G 亮度而非整幅白色掩膜作种子，保留混色子像素的强弱。
    seed = np.where(source, image[:, :, 1], 0).astype(np.uint8)
    diameter = radius * 2 + 1
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (diameter, diameter))
    expanded = cv2.dilate(seed, kernel)

    # 半径决定主要扩散距离；小 sigma 只柔化膨胀边缘。
    sigma = 0.5 + radius * 0.08
    pad = max(2, int(math.ceil(3.0 * sigma)))
    padded = cv2.copyMakeBorder(expanded, pad, pad, pad, pad,
                                cv2.BORDER_CONSTANT, value=0)
    softened = cv2.GaussianBlur(padded, (0, 0), sigmaX=sigma, sigmaY=sigma)
    softened = softened[pad:-pad, pad:-pad]
    bloom = np.rint(np.clip(softened.astype(np.float32) * strength, 0.0, 255.0)).astype(np.uint8)
    output[:, :, 1] = np.maximum(output[:, :, 1], bloom)
    return output
