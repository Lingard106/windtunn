# profiles 配置交接

本目录保存运行参数，不包含可执行入口。各 JSON 的作用层不同；启动命令、解析器与最终生效配置共同决定实验，不能用某一默认文件代替完整方案。总导航见[仿真区 README](../README.md)。盘点日期：2026-10-21；本次仅读取文件，未更改数值或运行验证。

## 文件说明

| 文件 | 消费方与当前用途 |
|---|---|
| [aero_default.json](aero_default.json) | `prototypes/aerodynamics.py` 加载的估计被动气动默认值：Cd=0.35、normal=2.5、stability=0.18、angular damping=2.0、密度1.225 |
| [field_2027_fixed_target.json](field_2027_fixed_target.json) | 2027 发射/靶面/绿灯几何、接触参数、标定来源；`simulation_setup.py` 默认选用 |
| [field_2027_fixed_target_config.json](field_2027_fixed_target_config.json) | 上述发射预设引用的数字物性快照：质量0.25 kg、初速约16.74702836 m/s、俯仰35.617°、步长0.002 s等 |
| [flight_control_default.json](flight_control_default.json) | `guidedemo_control.py` 未显式指定飞控文件时的默认方案；N15、Kr0.8、Kα0.5、KI0.01，`guidedemo_v1` 模式 |
| [flight_control_v1_direct_2027.json](flight_control_v1_direct_2027.json) | [运行基线物理仿真.cmd](../../启动器/运行基线物理仿真.cmd)显式加载的原候选；N10、Kr0.05、Kα0.5、KI1.7，滚转 P0.1，`guidedemo_v1` 模式 |
| [ideal_guidance.json](ideal_guidance.json) | 理想导引默认的传统 PN 对照：N10、原发射预设、0.25 g合成上限/0.1 s响应/2.5 g/s变化率，额外瞄准增益为0 |
| [vision_desktop.json](vision_desktop.json) | 共享桌面视觉设置：LAB、2500 μs曝光近似、光晕半径4/增益2、红十字 `[160,125]`、镖参考 `[130,120]` |

完整物理增强版使用独立的 [traditional_control/config.json](../traditional_control/config.json)（含内层飞控与固定标称预测模型）；理想 v2 使用 [pn_expert_v2/config.json](../pn_expert_v2/config.json)；v3 与采样配置位于 [learning](../learning/README.md)。它们不在本目录，也不应混作同一种 `--flight-config`。

## 默认加载与覆盖

1. 数字参数由估计气动默认值与 `prototypes/dart_demo_config.json` 合并；后者是通用演示起点，并非2027运行的最终值。
2. `simulation_setup.resolve_startup_paths()` 优先选择现存的2027轻量场地、默认镖体清单和本目录固定靶发射预设。该预设 `calibration.config` 指向旁边的 `field_2027_fixed_target_config.json`。
3. 模型/发射方案建立几何和发射基线；保存的数字快照中非发射量随后覆盖，保持标定重放。显式 `--config` 中实际出现的字段最终优先。
4. 飞控文件由 `--flight-config` 单独选取。直接运行原窗口时飞控默认关闭；“运行基线物理仿真.cmd”显式开启并选 `flight_control_v1_direct_2027.json`。
5. 理想 v2 的 `use_nominal_launch=true` 另设初速16.93559338 m/s、偏航−15.23843314°，因此它不沿用原刚体初速。CLI 偏移在该基线上叠加。

相对引用通常按所在 manifest/预设目录解析，CLI 显式相对路径按当前工作目录解析。发射 JSON 中保留了来源报告的本机绝对路径文字；当前自动加载依据是相对的 `calibration.config`，迁移时要区分来源记录与实际依赖。

## 现有使用示例

以下在工作区根目录 PowerShell 执行；命令会运行程序，本次未执行。

```powershell
# 原 v1 指定飞控配置
.\guidance_sim\.venv\Scripts\python.exe guidance_sim/prototypes/dart_viewer.py --season 2027 --flight-control on --flight-config guidance_sim/profiles/flight_control_v1_direct_2027.json --auto-run

# 理想传统 PN 显式指定独立配置
.\guidance_sim\.venv\Scripts\python.exe guidance_sim/prototypes/ideal_guidance_sim.py --flight-config guidance_sim/profiles/ideal_guidance.json --guidance-source vision --auto-run
```

`--config` 用于数字物性覆盖，`--launch` 用于发射与靶几何，`--scene` 用于场地清单。完整物理增强版与理想入口还支持 `--vision-config`；原 `dart_viewer.py` 固定读取共享 `vision_desktop.json`。

## 会被写回的配置与交接边界

- 共同窗口“应用视觉并重置”会写回 `vision_desktop.json`，所以它是当前操作状态，不能只当冻结参考。运行截图/结果应保留同刻视觉快照。
- v3 的参考标定零点为 `[160,180]`，当前共享红十字 `[160,125]` 在参与微调模式会产生明显非零靶面偏移；详见[learning README](../learning/README.md)。数据采样器按自己的标定点±2像素产生回合参考，不直接沿用该共享红十字。
- `flight_control_default.json` 和 `flight_control_v1_direct_2027.json` 名称相近但参数不同；不显式带配置路径的运行不能默认叫作“v1实跑参数”。
- 部分飞控缺省字段会由加载器补齐，JSON原文并不总是完整生效配置；共同输出会保存解析后配置，交接对照以该快照为准。
- 气动文件明确 `engineering_estimate`、`unvalidated`、未用CFD；舵效/碰撞软硬/有限过载响应同样未作实物标定。靶半径为0.03 m，有限尺寸镖头接触和镖尖过面误差的评价含义不同。
- 本次没有核对来源中的赛事规则是否更新，也没有重做标定或历史验收；读取及 JSON 结构检查不等同运行正确性验证。
