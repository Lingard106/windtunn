# guidance_sim — 制导闭环全仿真 (Phase 0)

对接文档: `../docs/guidance_loop_dev_spec.md` (v0.4, 接口以文档为准)

## 快速开始

```bash
python -m venv .venv && .venv\Scripts\activate
pip install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple
python scripts\hello_pybullet.py       # 渲染链路
python scripts\selftest_simcore.py     # sim_core 自检
python eval\run_mission.py             # 单发闭环
python -c "from eval.monte_carlo import monte_carlo; monte_carlo(20)"
python scripts\record_nominal.py       # 录制 M1 测试序列
```

## Phase 0 状态 (2026-10-08)

- 全链路闭环跑通: 占位 URDF → 解析气动 → PyBullet 6DOF → 虚拟相机 → 色块识别 → 比例制导 → 脱靶量
- 标称场景脱靶 7.3m (占位气动+占位制导, 期望内; M2 用真气动表+正式制导设计后达标 100% 命中)
- 排障记录: PyBullet 内建线性阻尼失真 (changeDynamics 关闭)、applyExternalForce WORLD_FRAME
  的 position 是世界坐标 (传 [0,0,0] = 挂在世界原点, 70m 力臂乱矩)、舵面用 resetJointState
  运动学摆位 (关节马达反作用矩踢爆低惯量弹体)

## 已知占位项 (Phase 1+ 替换)

- aero/placeholder.py: 解析占位气动 → CFD 扫气表 lookup (CFD 侧交付)
- detector/color_blob.py: 红色色块 → ArUco/真实纹理
- guidance/pn_pid.py: 横向追踪+俯仰阻尼 → 双环 PN+姿态控制 (含纵向射程管理)
- 场景: 单一靶板 → 可配置环境 (§5.1.1 引擎选型)
