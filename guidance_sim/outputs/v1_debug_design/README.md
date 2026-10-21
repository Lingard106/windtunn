# v1三轴控制与调试显示交接

交接盘点：2026-10-21；原有135个文件、13,876,359字节。下述验收为2026-10-20阶段的既有记录，本轮未重新运行桌面窗口、控制或回放测试。返回[输出总索引](../README.md)。

## 目录职责

本目录承接v1俯仰/偏航独立参数、可选滚转角度/角速率环、三轴侧栏、独立相机窗口、透明g HUD及只读调试缓存/导出的设计与验收。后续UI性能和波形旁调参优化另见[v1_ui_refinement](../v1_ui_refinement/README.md)。

| 入口 | 含义与状态 |
|---|---|
| [design.md](design.md)、[implementation_plan.md](implementation_plan.md) | 原设计、约束和已完成记录；design末尾说明最新侧栏/相机布局，前期布局也被保留 |
| `inspection.json`、`control_probes.py/.json`、`embedded_tuning_probe.py/.json` | 改前通道检查及新增控制/传感器/嵌入调参检查 |
| `readonly_debug_review.py/.json/.csv` | 诊断读取不推进物理/控制/噪声状态的审查工具与结果 |
| `baseline/manifest.json`、`noise_off/on.npz`及状态JSON | 改动前真实视觉v1基线：有/无IMU噪声各1151行、2.3s，带源码/配置哈希、控制器和噪声快照；作为历史比较基准保留 |
| `source_before/` | 改动前README、部分Python与配置快照，路径层级保留原样；不是完整仓库，也不是当前可运行分支 |
| `preview.png` | **早期设计图**，不能作为当前侧栏/独立相机布局验收图 |
| `acceptance/` | 离屏、数值、真实Tk、回放、截图导出的主要证据和检查脚本 |

## 验收证据索引

- `acceptance/integration_report.json`：历史记录两种噪声模式与冻结基线各1151行逐项一致，差值0，诊断读取只读。
- `acceptance/desktop_report.json`：v1/v2/v3真实Tk窗口、独立相机、运行/暂停、回放截止、重置与只读显示的结果。对应`desktop_v1_live.png`、`desktop_v1_compact.png`、`desktop_v1_camera.png`及v2/v3截图。
- `acceptance/capture_report.json`：带HUD显示画面与识别原图分开、HUD关闭还原、固定320×240识别源图、保存不改状态；其`folder`指向`capture_smoke/outputs/dart_viewer_20261020_142415_178515/`。
- `acceptance/debug_module_report.json`、`actuator_diagnostic_report.json`、`roll_plant_report.json`：缓存/因果回放/图表、执行器诊断、滚转扰动数值检查。
- `acceptance/debug_*_final.*`：后期因果截止和导出证据；同名不带`final`、小尺寸/早期排布图保留作中间过程，不能仅凭文件名认为全部代表最新UI。
- `acceptance/noise_off/on_cutoff.csv`、`ui_v1/v2/v3_replay.csv/.png`：对照及回放截止导出；数据量较大，按报告对应时间阅读。
- `acceptance/capture_smoke/outputs/`：四次保存画面与配置的历史运行目录，含状态、配置、CSV、源图和显示图；由`capture_report.json`定位最终被引用的一次。
- `acceptance/legacy_v2/`：旧简化专家入口兼容结果与大型轨迹，不是v1主验收。

报告描述的是各自测试时刻；后续UI修复再次使用过部分本目录报告/CSV，完整时间关系参见[v1_ui_refinement/plan.md](../v1_ui_refinement/plan.md)。

## 对应源码与保留建议

当前源码为[prototypes/flight_debug.py](../../prototypes/flight_debug.py)、[dart_viewer.py](../../prototypes/dart_viewer.py)、[tuning_panel.py](../../prototypes/tuning_panel.py)、[guidedemo_control.py](../../prototypes/guidedemo_control.py)、[flight_sensors.py](../../prototypes/flight_sensors.py)和[imu_noise.py](../../prototypes/imu_noise.py)。基线采用的视觉配置/哈希见`baseline/manifest.json`及后续`v1_ui_refinement/legacy_regression.json`，不能直接用用户后来保存的视觉参数代替。

保留`source_before/`、基线、manifest/状态、主验收报告、探针和代表截图；它们用于说明新显示未改变旧轨迹。`check_desktop.py`等会创建GUI或生成文件，本轮未运行，重验时应先检查参数与输出路径，使用新目录，避免覆盖既有证据。不要把`source_before`复制覆盖`prototypes`。
