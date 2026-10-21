# learning 参考微调与专家数据交接

本目录提供 v3 靶面参考转换，以及基于理想 v2 的专家示范数据采样。当前没有神经网络模型、优化器或训练入口。总导航见[仿真区 README](../README.md)。盘点日期：2026-10-21；本次未采样、训练或重跑历史数据。

## 文件职责

| 文件 | 用途 |
|---|---|
| [aim_reference.py](aim_reference.py) | `AimReference`，把标定视角下期望绿灯像素差转换为固定靶面偏移，支持显示/参与两模式及快照 |
| [aim_reference.json](aim_reference.json) | 320×240 标定、零偏移像素 `[160,180]`、明确的 1 m 角度到距离换算尺度，当前模式 `image_reference` |
| [dataset.py](dataset.py) | 配置校验、独立回合种子与工况计划、决策前特征采集、同刻动作标签、质量掩码及 manifest 内的版本/哈希/汇总 |
| [config.json](config.json) | 默认分区数量、90 Hz 相机、2.6 s 时长、目标误差阈值和随机范围，引用 v2/视觉/参考配置 |

运行依赖[理想导引适配器](../prototypes/README.md)、[v2 专家](../pn_expert_v2/README.md)、[profiles](../profiles/README.md)、运行资产和 NumPy/MuJoCo/OpenCV/Pillow；本目录不能脱离共同底座独立交接。

## 现有命令

均在工作区根目录 PowerShell 中运行。`启动器/` 中的[运行参考点微调仿真.cmd](../../启动器/运行参考点微调仿真.cmd)为第一条的窗口入口；根目录的[采集专家示范数据.cmd](../../采集专家示范数据.cmd)调用采样器。

```powershell
.\guidance_sim\.venv\Scripts\python.exe -B guidance_sim/prototypes/ideal_guidance_sim.py --flight-config guidance_sim/pn_expert_v2/config.json --aim-reference-config guidance_sim/learning/aim_reference.json --guidance-source vision --auto-run

# 仅校验配置并打印计划，不渲染、不写采样数据
.\guidance_sim\.venv\Scripts\python.exe -B -X utf8 guidance_sim/learning/dataset.py --dry-run

# 会执行采样；本次未运行
.\guidance_sim\.venv\Scripts\python.exe -B -X utf8 guidance_sim/learning/dataset.py --counts 48 12 12
```

采样器支持 `--config`、`--counts TRAIN VALIDATION TEST`、`--seed`、`--output`、`--dry-run`。训练回合至少一个，其余可为零；默认输出为 `outputs/expert_dataset/<UTC时间戳>_<随机后缀>/`。指定输出目录必须不存在或为空，非空目录会被拒绝。

## v3 参考语义与当前配置差异

`display_only` 只显示红十字；`image_reference` 把红十字相对标定点的偏移映射到固定靶面。下移让所选目标沿靶面向上，右移让所选目标向左；1 m 是声明的换算尺度，不是实时相机测距。固定偏移不追求整段高抛运动中绿灯停在同一像素。原靶面与原命中判定保留，超出原靶半径只加标志、不夹紧目标。

**当前共享视觉配置的 `aim_point_px=[160,125]`，与本目录的 `[160,180]` 标定零点不同。** 理想仿真的 `submit_visual_measurement()` 会把识别详情中的参考像素传入 `AimReference.update()`；窗口启动未见强制恢复零点。按当前 55°视场与 1 m 尺度静态换算，55 像素差约为 238.6 mm 靶面偏移，明显大于原 30 mm 半径。该数值是配置/公式推导，未启动窗口核对；接手时应先确认这是否为上一轮刻意保留的试验状态，不能把当前 v3 默认当作零偏移演示。本次未改参数。

采样计划不直接沿用共享视觉红十字位置：`episode_plan()` 使用本目录标定点加独立整数随机扰动，`collect_episode()` 以每回合设置替换像素。窗口“应用视觉并重置”会写回共享 `profiles/vision_desktop.json`；采样器不改该文件。

## 数据契约与默认范围

默认 72 个完整飞行回合按训练/验证/测试 48/12/12 分区；种子由主种子 2027101901、分区和回合索引派生。初始 XYZ 各±0.05 m、恒定侧风±1 m/s、偏航±0.2°、俯仰±0.1°、初速±0.05 m/s、参考像素各轴±2。目标仍为固定靶，误差阈值 0.01 m。

每次专家决策前采集 31 维 `X`：估计目标相对机体位置3、自身世界速度3、姿态矩阵9、实际控制2、上一请求2、最近检测视线3、当前有效/锁定2、检测年龄1、目标估计有效/年龄2、时间1、声明目标偏移3。标签 `y_command_g` 为同刻更新后的俯仰/偏航两维非重力比力请求，单位 g、下/右为正，既不是舵角也不是实际已达到过载。

输入含理想自身位置/姿态、速度和实际控制状态，目标来自真实图像与静态 CAD 先验；真实风、真实目标和未来状态不进入 `X`。特权真值只在工况元信息与 `eval_*` 数组中供评价。缺失目标/视线以零填充，年龄截到 1 s，并另有有效性标志。

| 产物 | 内容与使用约束 |
|---|---|
| `train/`、`validation/`、`test/` 每回合 NPZ | `X`、`y_command_g`、`t_s`、`sample_valid`、`trusted_training_mask`、专家事件/状态，以及单独 `eval_*` 数组 |
| `manifest.json` | 计划/逐回合参数、完成状态、特征名/单位、假设、运行版本、完整配置和采样前后源码哈希；完成后 `summary` 内含分区质量、命中/目标误差、可信样本数量 |

新采集只写每回合 NPZ 和 `manifest.json`，不再重复写三份配置 JSON 或 `report.json`，不导出图片。窗口与无窗口仿真的显式保存规则见[输出规范](../../docs/输出规范.md)。

`sample_valid` 反映新鲜有效专家更新；可信掩码还要求原靶命中、所选目标误差≤10 mm、执行约束通过。失败回合保留，可信掩码置零。完整回合各自分区，后续标准化只能在训练集拟合。采样前后哈希变化会把运行标记为无效；中断/失败会保留已有回合与 manifest，当前没有续跑入口。

历史说明提到首批结果中的 `sl_ready/` 已筛选数据，但当前 `dataset.py` 中没有创建该目录或自动汇总三份筛选 NPZ 的逻辑。它应作为已有结果单独交接，不能承诺重新运行当前采样器会自动生成同样的导出。历史 72 发统计和失败回合分析见[历史运行说明](../../docs/历史运行说明_2026-10-20.md)；本次未重新核验结果内容。
