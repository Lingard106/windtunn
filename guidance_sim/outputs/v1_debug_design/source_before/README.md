# 飞镖仿真：极简运行版

原三个入口分区保留；新参考接口和数据采集放在learning分区。仿真窗口加载2027轻量场地，可暂停、重置、调参和回放。

| 启动器 | 验证内容 | 配置 |
|---|---|---|
| 启动v1实跑参数.cmd | 原MuJoCo刚体＋传统PNG＋三回路自驾仪＋舵机/气动 | profiles/flight_control_v1_direct_2027.json |
| 启动理想导引仿真.cmd | 原N10传统PN对照，原发射初值，有限过载质点 | profiles/ideal_guidance.json |
| 启动PN专家v2.cmd | 弹道预测改进PN，真实视觉估计上方靶面 | pn_expert_v2/config.json |
| 启动参考微调v3.cmd | 红十字参与靶面微调，可切回仅显示 | learning/aim_reference.json＋v2专家 |
| 采集PN专家数据.cmd | 批量随机工况专家采样，生成SL数据 | learning/config.json |

原入口和控制参数保留；新专家的初速与导引不会覆盖原方案。共用传感器修复：OpenGL像素中心使用159.5/119.5进行反投影，避免半像素定位偏差。

首次安装（在本目录打开PowerShell，使用带Tkinter的Python 3.11）：

```powershell
py -3.11 -m venv guidance_sim/.venv
.\guidance_sim\.venv\Scripts\python.exe -m pip install -r guidance_sim/requirements.txt
```

原物理版PNG N=10，俯仰/偏航共用Kr=0.05、Kα=0.5、KI=1.7，滚转P=0.1；相机参数读取profiles/vision_desktop.json。几何评价目标为绿灯上方装甲平面。默认视觉识别对象仍是绿灯，单纯跟踪绿灯不能自动保证击中上方平面。

## 运行代码

共用运行及传统验证代码位于 `guidance_sim/prototypes/`：

| 文件 | 功能 |
|---|---|
| dart_viewer.py | 主窗口、双画面、交互与采样调度 |
| dart_demo.py | MuJoCo建模、积分、接触和状态输出 |
| ideal_guidance_sim.py | 重力/阻力质点运动、有限过载响应、同场景和真实视觉的独立入口 |
| simulation_setup.py | 选择当前场地、镖体和发射配置 |
| dart_model.py | 加载镖体、物性、相机和靶面 |
| field_scene.py | 加载场地与活动靶头网格 |
| target_motion.py | 固定及移动目标模式 |
| aerodynamics.py | 被动气动力和力矩 |
| control_actuation.py | PWM、舵机响应和舵效载荷 |
| guidedemo_control.py | PNG、三回路自驾仪和滚转控制 |
| flight_sensors.py | IMU与图像视线测量 |
| imu_noise.py | IMU噪声 |
| openmv_tracker.py | 灯块检测与追踪 |
| vision_settings.py | 阈值、曝光和瞄准点配置 |
| camera_optics.py | 图像光晕近似 |
| playback.py | 采样时钟与状态回放 |
| tuning_panel.py | 窗口内飞控调参 |

`profiles/`保存当前物理与理想仿真的运行配置；`assets/`保存必需STL/OBJ与清单；`reference/`只留运行元数据需要的来源哈希JSON。测试、离线工具及历史结果均已删除。

新专家分区为 `guidance_sim/pn_expert_v2/`：expert.py只负责弹道预测和过载决策，config.json只保存v2参数；共用ideal_guidance_sim.py提供场景、视觉、运动、窗口与回放适配。v2窗口截图保存在outputs/pn_expert_v2/；批量验收在outputs/pn_expert_acceptance/，均被Git忽略。

原始CAD和机械图纸按此前约定留在 `local_originals/`；它与本机 `.venv/`、运行生成的 `outputs/` 均被Git忽略。运行不依赖原始CAD。气动和舵效为工程估计，几何接触不等同官方裁判识别。

## 飞控位置与诊断结论

- 导引与自驾仪：guidedemo_control.py。PNG由视线速率产生过载，_autopilot_update把过载变成舵角；v1三项增益不是普通PID的Kp/Ki/Kd。
- 舵机与舵效：control_actuation.py；IMU与图像适配：flight_sensors.py；力加载及积分：dart_demo.py。
- 物理配置：profiles/flight_control_v1_direct_2027.json。controller、servo、actuation各节分别控制自驾仪、舵机、舵效。现有增益是既有几何候选，未通过过载跟踪调参。

2026-10-19诊断未发现千倍单位换算错误。默认初速下1°偏航舵产生约0.008215g直接横向比力；固定0.1g指令下，现有v1在0.1s只得到0.000114g，2s约0.003459g。主要问题是自驾仪带宽及伪攻角估计与当前估计气动不匹配。滤波器以姿态θ为输入，持续转弯时会与误差积分竞争；不能因为有积分就断言无静差。

角度以抬头/左转为正，过载以向下/向右为正；正舵与过载反号本身不是错误。+1g指令伴随−2°/s角速率时，角速率项可暂时压过积分，产生约−0.000752g反向瞬态。不要据此把全链路符号一起翻转。默认发射方向已预对准固定靶，偏航变化小也不能证明PNG有效或无效，须加偏航扰动对照。

已修正：v1原抗积分饱和条件方向；相邻视觉光线使用自身完整姿态传输，消除离轴/滚转造成的假视线速率。原分轴陀螺补偿只作旧接口兼容。实际过载显示到六位小数，区分机体偏航与航迹偏航。修正确定问题不等于解决了当前v1的弱过载跟踪。

## 独立简化导引仿真

入口仍为启动理想导引仿真.cmd。默认真实渲染→识别→视线测量→PNG→合成过载限幅→一阶响应/变化率限制→运动积分，绕过自驾仪、PWM、舵机和气动舵效。独立配置profiles/ideal_guidance.json；默认关闭额外红点瞄准修正以观察原PNG。

reduced_order节可直接修改，窗口“导引 / 飞控调参”也可修改并重置：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| max_control_g | 0.25 | 俯仰、偏航合成控制比力上限，不含重力及被动阻力 |
| response_tau_s | 0.10 s | 一阶过载响应时间常数；未触发变化率限制时约0.23 s达到阶跃90% |
| max_slew_g_s | 2.5 g/s | 合成控制过载的最大变化率 |
| drag_enabled | true | 使用当前质量、面积、密度、Cd及横风计算二次阻力 |

这些是可调工程假设，尚未标定实物性能。0.25g参考当前初速16.747m/s、30°等效舵偏下约0.246g的直接舵致比力；它不是包含机体气动力的总能力预测，也未随空速自动缩放。原controller.max_accel_g=5仅限制PNG单轴请求，不能理解为能做到5g。窗口分别显示PNG请求、限幅后的执行目标和实际响应。

按当前m=0.25kg、S=0.0033587m²、ρ=1.225，18m/s时动压约198.45Pa。5g横向比力需要12.26N，按该面积折算CN≈18.39；当前线性舵效30°仅约0.285g。即使不合理地外推到90°也仅约0.854g，且大偏角无法用此线性公式预测。[气动力公式](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/lift-equation/)、[大迎角影响](https://www.grc.nasa.gov/WWW/k-12/VirtualAero/BottleRocket/airplane/incline.html)。

运动方程为a_W=R(v)[0,−g·n_yaw,−g·n_pitch]+a_drag+[0,0,−g]，其中n使用实际响应状态。重力始终开启，只叠加一次；没有自动抵消重力。关闭PNG与阻力时为解析抛物线，保留阻力时是有阻力抛体。PNG主动修正后轨迹本来就不必是抛物线；旧版首次有效指令直接达到俯仰5g，是轨迹剧变的主要原因。

PNG不输出姿态角。简化版仍令机体前轴沿速度、滚转为零，不单独模拟姿态响应；航迹转向受实际过载限制。仅作几何过面检查，无刚体碰撞响应。真实视觉N10在当前高抛初始条件下仍可能长期请求超出能力的过载，不能据此声称命中或增益最佳。

显式使用--guidance-source truth可检查真值视线对照；默认vision不以目标坐标生成导引测量。加入--yaw-offset-deg 1观察偏航扰动。无控制、无阻力参考可直接运行：

```powershell
.\guidance_sim\.venv\Scripts\python.exe -B guidance_sim/prototypes/ideal_guidance_sim.py --flight-control off --no-drag --auto-run
```

## PN专家v2

这是重力/阻力感知的弹道预测ZEM/PN修正，不是传统N·Vc·视线角速度公式。先预测零控制时镖头与靶平面的交点，再用包含过载滞后的末端灵敏度修正偏靶。若原弹道已能命中，专家基本不干预，不再把高抛弹道正常的视线变化当成需要消除的误差。

新分区仅对简化质点模型标定初速16.93559338m/s、偏航−15.23843314°，俯仰仍35.617°；原刚体方案16.74702836m/s保持不变。v2比例N=3，专家20ms更新、预测积分25ms、视觉目标滤波时间常数80ms。执行仍受0.25g合成限幅、100ms响应、2.5g/s变化率限制，重力与当前Cd阻力始终进入运动。

默认视觉路线：真实320×240渲染→原绿灯检测→正确像素光线及自身相机光心→静态测绘灯X平面交点→已知灯到上方靶面的刚性偏移→目标估计。目标识别已锁定且色块未被画面边缘裁切时才更新；视觉超时撤销新控制请求，实际过载按原响应衰减。这里明确使用场地静态几何标定和理想自身位姿，控制不读取动态靶坐标或真实风速；改变场地后需要重新标定。truth模式是单独对照。

验收范围为固定目标，命中使用原CAD镖头首次穿过原30mm半径靶面，不修改判定半径、不吸附镖体。移动目标、真实相机内参/时延和实飞气动尚未验收；这些工程假设不等同实物性能。本阶段不包含NN或训练。

2026-10-19冻结参数验收：29组真实视觉工况全部命中（标称1组、组合边界8组、固定种子20271019随机留出20组）；相同初值关闭控制的对照为3/29。扰动范围为初始XYZ各±5cm、常值侧风±1m/s、偏航±0.2°、俯仰±0.1°、速度±0.05m/s。最大偏靶1.516mm，P95偏靶1.436mm；最大实际控制过载0.113571g，向量变化率未超2.5g/s（仅浮点舍入误差）。报告及逐步轨迹见outputs/pn_expert_acceptance/acceptance_report.json。这是该局部工况范围的仿真验收，不是任意风况或实飞保证。

命令行可复现任意工况，例如XYZ各偏移5cm、1m/s侧风：

```powershell
.\guidance_sim\.venv\Scripts\python.exe -B guidance_sim/prototypes/ideal_guidance_sim.py --flight-config guidance_sim/pn_expert_v2/config.json --headless --position-offset-m 0.05 -0.05 0.05 --wind-cross-mps 1 --output guidance_sim/outputs/pn_expert_v2/example
```

## 参考微调v3与初始专家数据

传统v1、传统PN和PN专家v2继续使用原控制方式。v2入口的红十字明确标为仅显示；使用启动参考微调v3.cmd，在相机窗口选择“参与靶面微调”，修改红十字后点击“应用视觉并重置实验”。“仅显示参考”模式保持原靶心。

红十字含义为标定视角下绿灯的期望图像位置：[160,180]是当前零偏移，下移让实际目标沿靶面向上，右移让目标向左。learning/aim_reference.json的reference_distance_m=1是明确的角度到距离换算尺度，不是实时测距；当前55°视场、240像素高对应4.338mm/像素。偏移固定在靶面坐标中，高抛轨迹仍保留正常的视线变化。移动2像素对应沿靶面约8.676mm；倾斜靶面的“向上”不等同世界竖直方向。

不暗中限制目标偏移或移动物理靶板。窗口显示实际毫米偏移及是否超出原30mm半径；到所选目标的误差和对原靶面的物理命中分别记录。移动10像素会请求约43.38mm偏移，可能准确到达请求点但仍判为偏出原靶，不能把它计作原靶命中。

learning/目录中：aim_reference.py负责目标参考转换，aim_reference.json保存参考标定；dataset.py负责采样，config.json保存随机范围、种子和数据划分。本阶段只生成专家数据，不训练NN。

默认72架次按完整飞行分为训练48、验证12、测试12，随机初始XYZ各±5cm、常值侧风±1m/s、偏航±0.2°、俯仰±0.1°、速度±0.05m/s及标定参考各轴±2像素。真实视觉为90Hz；每次专家新决策前记录31维自身状态/视觉/所选目标估计，随后记录同刻俯仰、偏航过载目标作为两维标签。风真值、目标真值、未来轨迹不进入输入X；理想自身位姿和静态地图先验仍明确保留。

每架次保存独立NPZ，含X、y_command_g、t_s、sample_valid、trusted_training_mask及单独eval_*评价数据。可信掩码要求专家新决策有效、原靶命中、所选目标误差≤10mm、执行约束通过；失败回合保留，可信掩码置零。数据集按回合和独立种子划分，后续标准化只使用训练集。manifest.json保存逐回合参数、参考位置、版本、配置、哈希；report.json汇总质量。outputs/expert_dataset/由Git忽略，数据需要单独共享。

运行一次新采样（自动创建新目录，不覆盖已有结果）：

```powershell
.\guidance_sim\.venv\Scripts\python.exe -B -X utf8 guidance_sim/learning/dataset.py --counts 48 12 12
```

仅查看计划可加--dry-run；--seed修改主种子，--output指定一个新的空目录。

首批已实际采集至outputs/expert_dataset/initial_sl_v1/：72架次、6959条原始记录，原物理靶命中71/72。训练/验证/测试分别包含2806/720/721条可信专家决策；sl_ready/内三份NPZ已按可信掩码筛选，可直接用于后续SL。X为31维状态/视觉特征，y_command_g为俯仰、偏航两维非重力控制比力请求，单位g，并非舵角。未进行标准化或NN训练。

唯一失败train_00037有172帧相机采样，仅检测到1帧、0帧被接受，专家未取得目标；96条原始记录完整保留，有效与可信掩码全零，未进入SL导出。这说明此前29例通过不覆盖所有视觉捕获工况。未来NN接管仍需显式处理目标不可用状态，不能由当前有效决策子集推断其已学会失锁处置。

本批71个成功回合到所选目标的最大误差为1.682mm；全部72回合执行约束通过，最大实际控制比力0.106434g。原始NPZ、采集源码/配置快照和SL导出均已独立核验，详见本批README.md、report.json和independent_audit.json。该批仅覆盖上述固定目标局部扰动范围，使用理想自身状态和静态地图先验。
