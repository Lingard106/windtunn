# 四发连续演示

双击根目录的「启动四发赛场演示.cmd」。直接复用单发时期 `PhysicalTraditionalViewer`：原赛场、原镖体、原镜头预设、鼠标旋转缩放、三通道波形、波形旁调参及独立镖载相机。只增加一行四发计分和自动换发。

本目录只保留 `run.py` 和本说明。已撤掉新造的回放窗口、展示场景副本和后台预计算；删除本次生成的逐帧图片、重复回合及临时检查文件。

- 开始后实时运行四发，共享同一个目标和比赛时钟；每发物理、视觉和飞控重新起步。
- 默认间隔2.6秒，默认沿用match公差与传统增强控制；原10秒恢复游走和命中冷却规则保留。
- 暂停、倍率、相机和调参使用原窗口操作；应用参数/重置会重开整轮。
- 时间轴只回放当前发的内存记录；换发清空上一发曲线和回放，四发计分保留。
- 默认不写文件。主动使用“保存结果”只写 `outputs/runs/viewer_<时间戳>.json`，含当前状态与完整数值/飞控/视觉配置；曲线“导出CSV”只写 `outputs/exports/` 下 CSV，不导出图片；关闭窗口释放内存。

可选命令（工作区根目录PowerShell）：

```powershell
# 不自动开始，便于先调镜头
.\guidance_sim\.venv\Scripts\python.exe -B guidance_sim/four_shot_demo/run.py

# 固定标称镖体、3秒间隔
.\guidance_sim\.venv\Scripts\python.exe -B guidance_sim/four_shot_demo/run.py --auto-run --randomize none --interval 3

# 不开窗口验证，结果只打印，不生成文件
.\guidance_sim\.venv\Scripts\python.exe -B guidance_sim/four_shot_demo/run.py --headless
```

`--seed`/`--target-seed`设置随机种子，`--target-mode`设置原目标模式，`--interval`范围2.6–9秒，`--randomize`为match或none。仅显式使用`--headless --output 路径.json`时保存一份汇总JSON；该参数是文件路径。统一文件契约见[输出规范](../../docs/输出规范.md)。仿真命中仍为原接触/过面判定，不是官方裁判硬件。
