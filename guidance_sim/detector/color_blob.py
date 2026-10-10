# -*- coding: utf-8 -*-
"""占位目标识别 — 红色靶板 HSV 阈值分割 → 质心像素 → 相机系视线向量。
接口对齐 dev_spec §4.4; ArUco/真模型为后续升级项。"""
import cv2
import numpy as np

LOWER = np.array([0, 90, 90])     # 红色 HSV 下界 (H 跨 0 与 180 两端, 双区间)
UPPER = np.array([10, 255, 255])
LOWER2 = np.array([170, 90, 90])
UPPER2 = np.array([180, 255, 255])


class Detector:
    def __init__(self, **params):
        self.min_area = params.get("min_area", 30)

    def detect(self, frame):
        img = frame["img"]
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, LOWER, UPPER) | cv2.inRange(hsv, LOWER2, UPPER2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                                cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return {"found": False, "uv": None, "los_C": None,
                    "p_T_C": None, "conf": 0.0, "latency_ms": 0.0}
        c = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(c) < self.min_area:
            return {"found": False, "uv": None, "los_C": None,
                    "p_T_C": None, "conf": 0.0, "latency_ms": 0.0}
        M = cv2.moments(c)
        u, v = M["m10"] / M["m00"], M["m01"] / M["m00"]
        K = frame["K"]
        los_C = np.array([(u - K[0, 2]) / K[0, 0],
                          (v - K[1, 2]) / K[1, 1], 1.0])
        los_C /= np.linalg.norm(los_C)
        return {"found": True, "uv": (u, v), "los_C": los_C,
                "p_T_C": None, "conf": 1.0, "latency_ms": 0.0}
