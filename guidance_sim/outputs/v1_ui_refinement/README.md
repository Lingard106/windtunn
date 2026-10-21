# UI响应性能与紧凑调参交接

交接盘点：2026-10-21；原有21个文件、438,780字节。本目录记录2026-10-20阶段的后续界面优化；本轮未测性能、启动GUI或复跑回归。返回[输出总索引](../README.md)。

## 最终结果与过程文件

先读[plan.md](plan.md)与[acceptance.json](acceptance.json)。既有记录覆盖85个调参字段、紧凑布局、波形旁俯仰/偏航/滚转编辑、共享草稿、局部应用与显示响应；`errors=[]`。性能结果依赖当时机器和窗口负载，不能当作当前机器实时性能保证。

| 文件 | 阶段与用途 |
|---|---|
| `acceptance.json` | 交付汇总：字段可达性、就地调参、观测复用与真实Tk心跳/视觉槽位记录 |
| `pure_checks.json` | 因果窗口、完整导出、复制隔离、数值语义、阶梯/尖峰保留与调度让出检查 |
| `legacy_regression.json` | 冻结视觉SHA下有/无噪声各1151行基线一致；引用兄弟`v1_debug_design/acceptance/`的CSV |
| `before_timing.json`、`after_timing.json`、`before_charts_profile.txt`、`after_charts_profile.txt` | 同阶段优化前后计时/剖析记录，属于历史对比；改变机器、分辨率或负载会影响数值 |
| `profile_debug.json` | 调试绘图性能探针的详细输出 |
| `tuning_layout.md`、`tuning_layout_report.json` | 调参字段、响应式布局与保留业务方法语义的核对 |
| `viewer_audit.md` | 查看器热路径/调度独立审查；与最终汇总一并理解 |
| `tuning_compact.png`、`tuning_large.png`、`waveform_tuning.png` | 紧凑调参页、大窗口及波形旁编辑截图 |
| `tuning_panel_before.py` | 优化前调参页源码快照，**不作为当前运行模块** |
| `check_pure.py`、`check_responsiveness.py`、`profile_debug_probe.py`、`profile_ui.py`、`tuning_layout_probe.py` | 当时的验证/剖析脚本；部分会创建GUI或写结果，本轮未执行 |

## 代码与输入

对应当前[prototypes/dart_viewer.py](../../prototypes/dart_viewer.py)、[flight_debug.py](../../prototypes/flight_debug.py)、[tuning_panel.py](../../prototypes/tuning_panel.py)。优化包含有界调度工作、字体/显示缓存、图表快照/像素包络及布局，原约束为保留物理/控制/90Hz视觉/诊断采样语义。与原三轴显示设计/验收的关系参见[v1_debug_design/README.md](../v1_debug_design/README.md)。

回归使用`expert_dataset/initial_sl_v1/vision_config.json`冻结视觉配置，其SHA保存在`legacy_regression.json`；当时用户当前视觉设置与历史基线设置分开保留。不要只用当前默认配置重跑后直接宣称原1151行基线回归失败或通过，应先确认输入一致。

## 保留或重建

保留前后性能数据、脚本、优化前快照、验收JSON和截图；此目录较小，适合整体打包。旧快照用于差异分析，不能覆盖当前源码。重验应记录机器/窗口尺寸、相机显隐、仿真时长及冻结输入，并写到新目录。关联的真实桌面、回放和截图报告在兄弟目录中，单独传递本目录会丢失部分证据。
