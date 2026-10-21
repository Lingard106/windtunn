"""飞控传感器适配：仅接受自状态与实际图像识别，不接受目标世界真值。"""

from __future__ import annotations

import math
import numpy as np


GRAVITY = 9.80665


def rotation_from_imu(imu, yaw_reference_deg=0.):
    """Self-attitude only: undo startup yaw zero and reconstruct body axes.

    Sensor noise in the fused Euler channels is intentionally retained. No
    target position, velocity or line-of-sight truth is used.
    """
    pitch, yaw, roll = np.radians([imu["pitch_deg"],
        imu["yaw_deg"] + yaw_reference_deg, imu["roll_deg"]])
    cp, sp, cy, sy, cr, sr = (math.cos(pitch), math.sin(pitch), math.cos(yaw),
                              math.sin(yaw), math.cos(roll), math.sin(roll))
    return (np.array([[cy, -sy, 0.], [sy, cy, 0.], [0., 0., 1.]]) @
            np.array([[cp, 0., -sp], [0., 1., 0.], [sp, 0., cp]]) @
            np.array([[1., 0., 0.], [0., cr, -sr], [0., sr, cr]]))


def _array(value, shape, name):
    array = np.asarray(value, dtype=float)
    if array.shape != shape or not np.isfinite(array).all():
        raise ValueError(f"{name} 必须是形状{shape}的有限数组。")
    return array


def imu_from_state(rotation_WL, omega_W, force_W, mass):
    """理想重心IMU；force_W是包括接触的非重力合力，不是含g的加速度。

    这属于控制通道适配，不能解释为原任务未标定的BMI088安装矩阵。
    俯仰角/角速为抬头正，偏航角/角速为向左正；过载采用向下/向右正。
    roll_rate_deg_s是机体系+X陀螺角速率，不是欧拉滚转角差分。
    """
    rotation = _array(rotation_WL, (3, 3), "rotation_WL")
    omega = rotation.T @ _array(omega_W, (3,), "omega_W")
    force = rotation.T @ _array(force_W, (3,), "force_W")
    if not math.isfinite(mass) or mass <= 0:
        raise ValueError("IMU质量必须为正有限数。")
    return {
        "pitch_deg": float(np.degrees(np.arctan2(rotation[2, 0],
                                  np.hypot(rotation[0, 0], rotation[1, 0])))),
        "yaw_deg": float(np.degrees(np.arctan2(rotation[1, 0], rotation[0, 0]))),
        "roll_deg": float(np.degrees(np.arctan2(rotation[2, 1], rotation[2, 2]))),
        "roll_rate_deg_s": float(np.degrees(omega[0])),
        "pitch_rate_deg_s": float(np.degrees(-omega[1])),
        "yaw_rate_deg_s": float(np.degrees(omega[2])),
        "a_pitch_g": float(-force[2] / mass / GRAVITY),
        "a_yaw_g": float(-force[1] / mass / GRAVITY),
    }


def vision_from_detection(details, t_s, camera_rotation_LC, fovy_deg):
    """320x240针孔反投影；C为CV右/下/前，输出原任务的下/右正视线角。

    使用真实检测中心和相机安装外参；红色瞄准点单独反投影为参考角。
    忽略error_px、locked和任何目标真值字段，不改变物理光心。
    detected=False时显式无效，不能把丢失时的(0,0)误差解释为居中。
    """
    sample_t = float(details.get("sample_t", t_s))
    if not math.isfinite(sample_t) or sample_t < 0 or sample_t > t_s + 1e-9:
        raise ValueError("视觉时间戳必须是当前或已发生的仿真时刻。")
    result = {"t_s": sample_t, "valid": False,
              "los_pitch_deg": None, "los_yaw_deg": None, "center_px": None,
              "aim_point_px": None, "aim_pitch_deg": None, "aim_yaw_deg": None,
              "aim_error_pitch_deg": None, "aim_error_yaw_deg": None}
    if not details.get("detected", False):
        return result
    center = details.get("center_px")
    if center is None:
        bbox = details.get("bbox")
        if bbox is None:
            return result
        x, y, w, h = _array(bbox, (4,), "bbox")
        if w <= 0 or h <= 0:
            return result
        center = [x+w*.5, y+h*.5]
    u, v = _array(center, (2,), "center_px")
    if not (0 <= u < 320 and 0 <= v < 240):
        raise ValueError("检测中心必须位于320x240画面内。")
    if not math.isfinite(fovy_deg) or not 0 < fovy_deg < 180:
        raise ValueError("相机垂直视场角无效。")
    rotation = _array(camera_rotation_LC, (3, 3), "camera_rotation_LC")
    focal = 120. / math.tan(math.radians(fovy_deg)*.5)
    # OpenGL's viewport centre lies between pixel indices 159/160 and 119/120.
    # Detection centroids are array indices; +0.5 converts to pixel centres.
    ray_local = rotation @ np.array([(u+.5-160.)/focal, (v+.5-120.)/focal, 1.])
    if ray_local[0] <= 1e-7:
        return result  # 该控制器只处理镖头前方视线。
    result.update(valid=True, center_px=[float(u), float(v)],
        los_ray_L=(ray_local / np.linalg.norm(ray_local)).tolist(),
        los_pitch_deg=float(np.degrees(np.arctan2(-ray_local[2], ray_local[0]))),
        los_yaw_deg=float(np.degrees(np.arctan2(-ray_local[1], ray_local[0]))))
    if details.get("aim_point_px") is not None:
        au, av = _array(details["aim_point_px"], (2,), "aim_point_px")
        if not (0 <= au < 320 and 0 <= av < 240):
            raise ValueError("瞄准参考必须位于320x240画面内。")
        aim_ray = rotation @ np.array([(au+.5-160.)/focal, (av+.5-120.)/focal, 1.])
        if aim_ray[0] <= 1e-7:
            raise ValueError("瞄准参考必须位于镖头前方。")
        result.update(aim_point_px=[float(au), float(av)],
            aim_pitch_deg=float(np.degrees(np.arctan2(-aim_ray[2], aim_ray[0]))),
            aim_yaw_deg=float(np.degrees(np.arctan2(-aim_ray[1], aim_ray[0]))))
        for axis in ("pitch", "yaw"):
            result[f"aim_error_{axis}_deg"] = result[f"los_{axis}_deg"]-result[f"aim_{axis}_deg"]
    return result


def sample_flight_vision(simulation, tracker):
    """无窗口runner与窗口采用相同渲染→成像近似→识别→提交顺序。"""
    source = simulation.render("onboard")
    image = tracker.prepare_image(source)
    _annotated, details = tracker.process(image)
    details["sample_t"] = simulation.t
    simulation.submit_visual_measurement(details)
    return details
