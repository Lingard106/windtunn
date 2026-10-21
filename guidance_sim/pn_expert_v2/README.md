# pn_expert_v2 数值专家交接

本目录只有独立参数和纯 NumPy 数值专家。图形场景、真实相机、状态推进、命中评价和回放由[prototypes/ideal_guidance_sim.py](../prototypes/ideal_guidance_sim.py)适配；[完整物理增强版](../traditional_control/README.md)也复用专家类，但另有参数包。总导航见[仿真区 README](../README.md)。盘点日期：2026-10-21；本次未运行专家或验收。

## 文件与接口

| 文件 | 责任 |
|---|---|
| [expert.py](expert.py) | `BallisticPNExpert`：输入自身估计状态与估计目标，输出受限俯仰/偏航比力请求及诊断；含更新保持、超时、状态捕获/恢复 |
| [config.json](config.json) | 供理想导引入口读取的 `reduced_order`、`expert`、`controller` 及来源说明 |

专家文件不导入 MuJoCo、场景或图像识别器；构造输入包含质量/密度/面积/Cd、重力、靶面法向、镖尖偏移及可选风估计。更新输入为时间、自身位置/速度、实际过载状态、目标估计和估计年龄。`wind_W_mps` 是可选估计，默认零；未知真实风不应作为默认视觉实验的特权输入。

## 现有运行方式

从工作区根目录 PowerShell 执行，或使用 `启动器/` 中的[运行专家策略仿真.cmd](../../启动器/运行专家策略仿真.cmd)。`expert.py` 没有独立 CLI。

```powershell
.\guidance_sim\.venv\Scripts\python.exe guidance_sim/prototypes/ideal_guidance_sim.py --flight-config guidance_sim/pn_expert_v2/config.json --guidance-source vision --auto-run

# 现有无窗口单工况接口；本次未执行
.\guidance_sim\.venv\Scripts\python.exe -B guidance_sim/prototypes/ideal_guidance_sim.py --flight-config guidance_sim/pn_expert_v2/config.json --headless --position-offset-m 0.05 -0.05 0.05 --wind-cross-mps 1 --output guidance_sim/outputs/pn_expert_v2/example
```

默认固定靶、真实视觉、2.6 s；`--guidance-law classical_pn` 是显式传统 PN 对照，`--guidance-source truth` 是显式目标真值路线。窗口和无窗口运行默认不写文件；无窗口结果打印到控制台。显式 `--headless --output` 只写目录内 `result.json`，另加 `--trace` 才写 `trace.json`；同目录再次运行会覆盖同名文件。窗口“保存结果”只写 `outputs/runs/viewer_<时间戳>.json`，不导出图片。统一规则见[输出规范](../../docs/输出规范.md)。

## 当前配置与适配边界

- `expert.law=ballistic_pn`，N=3，专家更新 20 ms，预测步长 25 ms，目标视觉滤波时间常数 80 ms。此处为弹道预测 ZEM/PN 修正，不能仅以传统 `N×接近速度×视线角速度` 解释。
- 理想执行上限为合成 0.25 g、响应 0.10 s、变化率 2.5 g/s；该比力不含重力和被动阻力，重力与阻力由运动模型单独处理。`controller.max_accel_g=5` 是请求限幅，不能解释为可实现 5 g。
- `use_nominal_launch=true` 使用本分区初速 16.93559338 m/s、偏航 −15.23843314°；原 2027 刚体预设初速约 16.74702836 m/s 保留。俯仰继续来自发射预设。
- 默认视觉先真实渲染、原绿灯检测，再结合相机光心、理想自身姿态/位置、静态灯 X 平面和灯到上靶面的刚性偏移产生目标估计。仅锁定且色块未被图像边缘裁切时接受新目标；超时撤销新请求，实际过载仍按响应衰减。
- 理想模型令前轴随速度、滚转为零，绕过自驾仪、PWM、舵机和真实气动舵效；命中采用镖尖几何穿面。不能把它等同完整刚体接触或实飞精度。
- 本配置顶层 `model` 字符串保留共同飞控模型名，实际算法由 `expert.law`、执行模型由 `reduced_order` 决定；识别分支应读取这些字段。

运行需保留[共同运行底座](../prototypes/README.md)、[场地与发射配置](../profiles/README.md)、资产和依赖。没有动态靶真值输入并不表示没有先验：理想自身导航、相机外参及静态 CAD 标定仍然存在，更换场地后需要核对标定与几何关系。

历史固定靶 29 工况验收见[历史运行说明](../../docs/历史运行说明_2026-10-20.md)及已有结果。该局部范围、真实相机时延/内参误差、实飞气动和移动目标能力本次均未重新验证；目录本身没有独立批量验收脚本，也没有训练程序。
