# 传统控制增强版结果交接

交接盘点：2026-10-21；原有87个文件、16,553,444字节。原主验收说明日期为2026-10-20，本轮只整理索引，没有重新仿真或实测。返回[输出总索引](../README.md)。

## 接手阅读顺序

1. [验收说明.md](验收说明.md)：最终结果、边界、生效参数、复现命令与当前截图。
2. [final_audit.json](final_audit.json)：64个留出种子对账、同工况配对、固定标称预测模型与公开CLI核对的历史汇总。
3. `final_selection.json`、`vision_frozen.json`、`prediction_model_nominal.json`、`tolerance_holdout_spec.json`：冻结参数/哈希、视觉、标称预测模型和主验收种子/公差。
4. `tolerance_holdout.json`：大型逐发主验收结果；先看汇总，有问题再读逐发数据。
5. `product_acceptance.json`、`randomization_audit.json`、`integration_checks.json`：GUI/回放、真实公差注入与模型接入的历史核查。

原主验收使用质量±1%、速度±0.1m/s、roll±1°、Cd±5%的联合独立均匀误差，无风，64个种子5101–5164。预测PN+物理过载PID为64/64，同一修复后视觉的原v1为3/64。所有候选成功为原镖头接触命中；接触点距靶心和镖尖过面误差是不同量，不能混作同一精度指标。64/64只描述该样本与该模型工况，不说明任意工况或实物100%。

## 哪些是中间或旧版证据

| 文件/子目录 | 分类与处理 |
|---|---|
| `plan.md`、`control_audit.md`、`target_audit.md`、`physical_candidate_design.md`、`roll_capture_audit.md`、`control_doc_review.md` | 设计与审查过程；最终生效配置和结论优先看验收说明/冻结文件 |
| `baseline_pilot.json`、`coarse_search.json`、`development*`、`predictive_pilot*`、`predictive_development_validation*`、`search_candidates/` | 试探、参数粗搜与开发验证；不是64种子小公差主留出 |
| `holdout.json`、`holdout_spec.json` | 更早的标称无公差对照，与主公差留出种子/范围不同 |
| `holdout_harness_error.json` | **旧验证脚本异常中断记录**，含缺少`target_position_W_m`属性的错误，不能当作完整有效留出结果 |
| `tolerance_development*`、`tolerance_reacquisition_dev*`、`roll_capture_*` | 公差开发及确认前视觉失捕修复的过程/消融证据 |
| `late_motion*`、`wind_stress*`、`stress*` | 边界压力试验；晚期移动32例仅14/32的记录不纳入最终小公差主验收，也不能据主验收宣称连续运动最优 |
| `prediction_model_before/` | 预测模型修改前源码/配置快照；仅供历史对照，不能当运行源码 |
| `cli_tolerance_smoke/`、`product_smoke/` | CLI与产品单例冒烟结果 |
| `firmware_doc/` | 移植说明的数值/OpenMV检查、C++主机检查及`.exe/.o`产物；源码对应根`firmware_examples/`，这些产物不是固件烧录包 |
| `hit_rate.png/.svg`、`physical_control.png`、`predictor_controls.png` | 结果图、真实窗口与预测参数页截图 |
| `__pycache__/` | Python缓存，可重建，本次未删除 |

`batch.py`、`benchmark.py`、`tolerance_batch.py`、`search_gains.py`、`report.py`、`*_probe.py`、`*_audit.py`和`*_checks.py`是当时试验/报告工具；`physical_candidate.py`为候选算法实现。它们位于ignored结果目录，交接源码时不能以为Git已包含这些历史工具。

## 产品代码、输入与复现

当前产品入口为[traditional_control/run.py](../../traditional_control/run.py)，实现位于同分区`simulation.py`、`viewer.py`、`vision.py`和[config.json](../../traditional_control/config.json)，通过根[运行传统控制仿真.cmd](../../../运行传统控制仿真.cmd)打开。原刚体/舵机/气动来自`prototypes/`。完整参数bundle和新分区的视觉重搜修复是方案组成部分，只将PID数字填回旧v1不能复现结果。

主验收的固定预测m/Cd不能跟随每发实际公差回填；自身导航仍是理想测量，真实气动/舵效未实物标定。`frozen_manifest.json`和`audited_source_hashes.json`记录早期冻结/核查来源，最终选择按`final_selection.json`及主验收文件核对，勿混用不同阶段哈希。

保留最终证据、原始开发/异常结果及其spec，便于说明选择过程。需要复现使用[验收说明中的公开命令](验收说明.md)，写到新的`traditional_control_recheck`等目录并记录版本；本轮未执行。要清理时可以重建缓存与主机编译产物，但先备份现有证据，本轮完整保留全部文件。
