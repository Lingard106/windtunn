# prototypes 共同运行底座交接

这里是各仿真分支共用的运行代码。目录名含 `prototypes`，但本次整理的六个既有入口都直接或间接依赖它，另有并行新增的[四发演示入口](../four_shot_demo/README.md)；迁移时不能只复制单个入口。总导航见[仿真区 README](../README.md)。盘点日期：2026-10-21；本次为静态检查，未运行仿真。

## 文件职责

| 文件 | 当前责任 |
|---|---|
| [dart_demo.py](dart_demo.py) | `DemoSimulation`、MuJoCo XML 建模、刚体积分、载荷加载、接触/过面结果、状态快照与无窗口短跑入口 |
| [dart_viewer.py](dart_viewer.py) | `DartViewer`，Tk 主窗口、独立相机窗口、实时调度、诊断侧栏、回放及状态/配置保存 |
| [ideal_guidance_sim.py](ideal_guidance_sim.py) | `IdealSimulation`/`IdealViewer`，有限过载质点响应、传统 PN/v2 专家切换、视觉上靶面估计、v3 参考适配与无窗口导出 |
| [simulation_setup.py](simulation_setup.py) | 解析场地、镖体、发射预设、数字配置和显式覆盖项，不计算运动 |
| [dart_model.py](dart_model.py) | 镖体 manifest、质量惯量、相机安装、发射与靶面清单加载/校验 |
| [field_scene.py](field_scene.py) | 场地 manifest、网格与活动靶外观装配；默认优先 2027 轻量场地 |
| [target_motion.py](target_motion.py) | `fixed`、`random_fixed`、`random_moving`、`end_moving` 四模式，事件驱动运动与可复现随机状态 |
| [aerodynamics.py](aerodynamics.py) | 估计被动气动力、力矩、默认气动参数和关闭气动的对照后端 |
| [guidedemo_control.py](guidedemo_control.py) | 飞控配置校验、视线 PN、原三回路自驾仪、可选过载 PID、滚转与四舵混控 |
| [control_actuation.py](control_actuation.py) | PWM、舵机限位/速度/一阶响应、估计舵效载荷 |
| [flight_sensors.py](flight_sensors.py) | 状态到 IMU、检测像素到机体系视线、统一真实相机采样接口 |
| [imu_noise.py](imu_noise.py) | 可开关、可缩放、带随机状态的 IMU 噪声与回放恢复 |
| [openmv_tracker.py](openmv_tracker.py) | 原绿灯阈值/色块/ROI/确认丢失状态机和显示标注 |
| [vision_settings.py](vision_settings.py) | 320×240 桌面视觉参数、曝光近似、红十字与镖参考像素校验 |
| [camera_optics.py](camera_optics.py) | 渲染图绿色光晕近似；不是实测相机 ISP |
| [playback.py](playback.py) | `SampleClock` 与状态回放缓存；回放不应重新推进识别或随机过程 |
| [flight_debug.py](flight_debug.py) | 诊断缓存、三轴/过载/舵面曲线与显式 CSV 导出 |
| [tuning_panel.py](tuning_panel.py) | 交互飞控参数草稿、校验、应用/重置与配置导出 |
| [dart_demo_config.json](dart_demo_config.json) | 低层通用演示数值默认值；正常 2027 启动会叠加 CAD 与发射预设 |

## 现有运行方式

在工作区根目录的 PowerShell 中执行，使用带 Tkinter 的本地 Python 3.11 虚拟环境。窗口快捷入口位于[运行基线物理仿真.cmd](../../启动器/运行基线物理仿真.cmd)和[运行标准导引仿真.cmd](../../启动器/运行标准导引仿真.cmd)。

```powershell
# 原 v1 完整物理模型，等同“运行基线物理仿真.cmd”的核心参数
.\guidance_sim\.venv\Scripts\python.exe guidance_sim/prototypes/dart_viewer.py --season 2027 --flight-control on --flight-config guidance_sim/profiles/flight_control_v1_direct_2027.json --auto-run

# 原传统 PN 的有限过载理想导引窗口
.\guidance_sim\.venv\Scripts\python.exe guidance_sim/prototypes/ideal_guidance_sim.py --guidance-source vision --auto-run

# 低层完整物理短跑，显式保存数值结果到指定目录
.\guidance_sim\.venv\Scripts\python.exe -B guidance_sim/prototypes/dart_demo.py --duration 0.6 --output guidance_sim/outputs/dart_demo_handoff_example
```

直接运行 `dart_viewer.py` 或 `dart_demo.py` 时 `--flight-control` 默认是 `off`。`ideal_guidance_sim.py` 默认是 `on`、固定靶、真实视觉；其 `--guidance-source truth` 是显式真值对照，`--flight-control off --no-drag` 是现有无控制无阻力参考。上列示例本次均未执行。

## 输入、加载顺序与输出

运行依赖 [requirements.txt](../requirements.txt)、[assets](../assets/)、[profiles](../profiles/README.md)，原飞控元信息还会读取 [source_manifest.json](../reference/guidedemo/source_manifest.json)。`simulation_setup.resolve_startup_paths()` 默认选择现存 2027 轻量/完整场地、`assets/dart_shenda/manifest.json` 和 `profiles/field_2027_fixed_target.json`；自定义场地需要显式核对 `--launch`。

数字参数先由气动默认值和 `dart_demo_config.json` 组合，再受模型/发射预设影响。2027 发射预设引用 `field_2027_fixed_target_config.json` 重放标定物性；调用方最后应用 `explicit_values`。显式 `--config` 中实际出现的字段优先，不能仅看本目录默认 JSON 判断最终质量和初速。飞控由独立 `--flight-config` 控制。

| 操作 | 输出 |
|---|---|
| `dart_demo.py` | 默认只打印；显式 `--output` 后仅写目录内 `result.json`，另加 `--trace` 才写 `trace.json` |
| `ideal_guidance_sim.py --headless` | 默认只打印；显式 `--output` 后仅写目录内 `result.json`，另加 `--trace` 才写 `trace.json`；两开关都要求无窗口模式 |
| 窗口“保存结果” | `outputs/runs/viewer_<时间戳>.json`，单文件内含当前状态及完整数值/飞控/视觉配置，不自动写 CSV 或轨迹 |
| 诊断窗口“导出CSV” | 仅主动点击时写 `outputs/exports/` 下 CSV |
| 窗口“应用视觉并重置” | 会写回共享的 `profiles/vision_desktop.json`，并重置实验 |

当前入口不导出图片，详细轨迹默认不累计。统一开关与保存规则见[输出规范](../../docs/输出规范.md)。`--headless` 仍可能渲染真实相机图用于识别，需要可用的图形驱动。窗口实时采样与无窗口采样均使用 90 Hz 相机时钟；物理步长和飞控周期来自各自配置。

## 状态边界与接手注意

- 完整物理路径保留刚体姿态、被动气动、PWM 与舵机；理想路径绕过这些执行环节，用受限比力推进，姿态前轴沿速度、滚转为零，仅做几何过面评价。两者的过载与命中结果不能直接当作等价证据。
- 坐标约定为世界 Z 向上，机体 X 前/Y 左/Z 上，四元数 `w,x,y,z`；视线及比力的正号另见源码。`flight_sensors.py` 使用 OpenGL 像素中心 159.5/119.5，修改相机投影前需保留该约定。
- 原 v1 的三项增益并非普通 PID 的 Kp/Ki/Kd。`flight_control_default.json` 与基线物理仿真启动器指定参数不同，记录实验必须保存完整配置。
- 原 `OpenMVTracker` 的未确认 ROI 丢失分支没有增强版的重捕获修补；增强版使用独立包装器，见[传统控制 README](../traditional_control/README.md)。
- 回放快照与配置有一致性要求；不能把某一分支或某份配置的状态直接恢复到另一分支。`__pycache__` 是可再生缓存，不是来源代码。
- `target_motion.py` 只实现本地运动假设和事件时序。其规则来源文字不等于本次重新核对了现行赛事规则；几何接触也不是官方裁判判定。
- 当前没有自动化回归测试目录；本次未验证图形窗口、模型稳定性、历史验收数字或导出内容的实际生成。
