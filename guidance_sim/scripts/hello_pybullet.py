# -*- coding: utf-8 -*-
"""环境验证: PyBullet 渲染链路 — 弹出 3D 窗口 (或 DIRECT 模式出图像) 即 OK。"""
import pybullet as p
import pybullet_data
import numpy as np
import os

DIRECT = os.environ.get("PB_DIRECT", "0") == "1"
cid = p.connect(p.DIRECT if DIRECT else p.GUI)
p.setGravity(0, 0, -9.81)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
p.loadURDF("r2d2.urdf", [0, 0, 1])
view = p.computeViewMatrix([2, 2, 2], [0, 0, 0.5], [0, 0, 1])
proj = p.computeProjectionMatrixFOV(60, 4 / 3, 0.1, 100)
_, _, rgb, _, _ = p.getCameraImage(320, 240, view, proj,
                                   renderer=p.ER_TINY_RENDERER if DIRECT else p.ER_KEEP_OPENGL)
img = np.reshape(rgb, (240, 320, 4))
print(f"hello_pybullet: mode={'DIRECT' if DIRECT else 'GUI'} img={img.shape} "
      f"mean={img[:, :, :3].mean():.1f} — 渲染链路 OK")
if DIRECT:
    p.disconnect()
else:
    input("按回车退出...")
    p.disconnect()
