# 经典控制验证仿真 — 对接文档

> 适用分支: `classical-control` ｜ 更新: 2026-10-20 ｜ 读者: 视觉/算法侧、电控/固件侧
>
> **结论先行**: 当前推荐入口是**「传统控制增强版」**(`启动传统控制增强版.cmd` +
> `guidance_sim/traditional_control/config.json`)。其余入口是对照组与算法实验分支,
> **参数与成绩不可跨入口混用**。

## 0. 与仓库其它文档的关系

| 文档 | 管什么 | 什么时候读 |
|---|---|---|
| 本文 | 仿真交付物、接口契约、数据契约、验收边界 | 要接这套仿真的数据或控制律时先读这里 |
| [`docs/control_strategy_and_firmware_porting.md`](control_strategy_and_firmware_porting.md) | 固件现状逐项核对、OpenMV/STM32 改法、示例代码 | **要上板移植时以该文档为准** |
| [`guidance_sim/README_lite.md`](../guidance_sim/README_lite.md) | 全部入口的运行手册、逐项参数与诊断结论 | 要跑、要调参时 |

本文只定义**边界与契约**, 不复述调参细节; 端口细节一律引用移植说明, 避免两处漂移。

## 1. 交付物: 六个入口

| 启动器 | 验证内容 | 配置文件 | 定位 |
|---|---|---|---|
| `启动v1实跑参数.cmd` | 原 MuJoCo 刚体 + 视线角速率 PNG(N=10) + 三回路自驾 + 舵机/气动 | `profiles/flight_control_v1_direct_2027.json` | 原方案对照 |
| **`启动传统控制增强版.cmd`** | 完整物理模型 + 视觉上靶面估计 + 弹道预测 ZEM/PN + 俯仰/偏航过载 PID + 滚转 P | `traditional_control/config.json` | **当前推荐主线** |
| `启动理想导引仿真.cmd` | 传统 N10 对照, 有限过载质点, 绕过自驾/舵机/气动 | `profiles/ideal_guidance.json` | 几何过面检查 |
| `启动PN专家v2.cmd` | 弹道预测改进 PN, 真实视觉估计上方靶面 (质点) | `pn_expert_v2/config.json` | 导引律实验 |
| `启动参考微调v3.cmd` | 红十字参与靶面微调, 可切回仅显示 | `learning/aim_reference.json` + v2 专家 | 标定实验 |
| `采集PN专家数据.cmd` | 批量随机工况专家采样, 生成 SL 数据集 | `learning/config.json` | 数据生产 (不训练) |

六者共用同一 2027 轻量场地与页面框架; 新专家的初速与导引**不会**覆盖原 v1 方案。
`firmware_examples/` 是上板参考代码, 原固件工程未被修改。

## 2. 接口契约

### 2.1 坐标、符号与单位 (最容易接错的地方)

- 机体系: `+X 向前 / +Y 向左 / +Z 向上`。
- 俯仰角与角速率**抬头为正**; 偏航**左转为正**; 过载请求**向下 / 向右为正**。
- 加速度计测的是**比力** `f = a − g`, 过载反馈直接用变换后的比力, **不要在 AP 输入处再扣一次重力**;
  静止向上加计约 `+1g`, 不是"静止过载必须为 0"。

```text
omega_pitch_deg_s = -omega_B.y * 180/pi
omega_yaw_deg_s   = +omega_B.z * 180/pi
n_pitch_measured  = -f_B.z / 9.80665
n_yaw_measured    = -f_B.y / 9.80665
```

- 正舵与过载反号**本身不是错误** (见混控与舵效符号); 不要凭曲线反号就全链路翻转。
- 简化入口 (理想/PN专家v2) 的过载是**非重力控制比力**, 重力独立叠加一次, 无自动重力补偿。

### 2.2 时间与频率预算

| 环节 | 当前值 | 说明 |
|---|---|---|
| 控制周期 (AP) | 5 ms / 200 Hz | 固件需改为截止时间调度, 记录实测抖动 |
| PWM | 20 ms / 50 Hz | 保持 1500 µs 中位, 10 µs/°|
| 视觉 | 90 Hz (320×240) | 每张图一次检测 |
| 诊断 | ~500 Hz | 纯读取, 最多 10000 条 |
| 预测决策 | 20 ms 更新 | 预测积分步长 25 ms, **不是 IMU 采样周期** |
| 目标滤波 / 可信超时 | 80 ms / 150 ms | 超时即撤销新控制请求 |
| 视觉延迟建模 | 66 ms (spec §3) | 移植前必须保留该预算 |

### 2.3 观测量契约

视觉帧**必须**提供: 原始像素中心 `u,v`、采样时刻 (`capture_time_us`, 相机时钟)、
bbox 宽高 (判裁切)、曝光、序号, 以及四个布尔语义:

| 标志 | 含义 | 可信判据 |
|---|---|---|
| `detected` | 本帧检出目标 | |
| `locked` | 状态机已确认锁定 | |
| `clipped` | bbox 触到任一画面边缘 | 触边即**不**作为可信上靶面测量 |
| `predicted` | 盲推 (COAST) 帧, 非实测 | 盲推帧**不得**刷新 150 ms 有效期 |
| `trusted` | `detected && locked && !clipped && !predicted` | 只有 trusted 更新目标 EMA |

**未检测也必须发包** (发无效状态); 不能用 `err=0` 表示"目标已居中"。
像素中心约定: 仿真用 OpenGL 像素索引 `159.5/119.5` 反投影; 硬件须用实测 `fx,fy,cx,cy`,
不要给所有 OpenMV 坐标机械 +0.5, 也不能继续把占位 `FOCAL_PX=277` 当标定结果。

IMU 需提供 7 通道: 俯仰/偏航/滚转角、俯仰/偏航角速率、俯仰/偏航比力 (g)。

### 2.4 导引 → 控制的输出边界

推荐链路 (与 `traditional_control/simulation.py` 一致):

```text
像素射线 l_C → (R_WB·R_BC) → l_W → 与测绘灯平面求交 → 加灯到上靶面刚性偏移
  → 目标位置 EMA (τ=80ms) → 弹道 ZEM/PN (预测 5 条弹道: 零请求 + 俯仰/偏航 ±0.02g)
  → 2×2 末端灵敏度 J → u = -(N/2)·solve(J,e), N=3
  → 合成圆盘限幅 0.2 g → 按实际姿态转机体系 → 两轴过载 PID → 四舵混控
```

- 目标偏移是**仿真标定值** `[+89.740, +0.004, +79.049] mm`, 实物必须按实际相机/镖体/场地重测。
- `N=3` 属于**这条预测律**, 不能替换原 `PngGuidance(4,20,5)` 里的 4 就宣称移植完成。
- 只接受"真实检测 + 已锁定 + 未触边裁切 + 交点在前方"的测量; 当前**只估计目标位置, 不估计目标速度**,
  也不读取换位程序。

过载 PID 离散式 (`traditional_control` 生效值):

```text
e = n_cmd - n_measured                          # g
I_trial = clamp(I_old + e*dt, -10, +10)         # g·s
D = (n_measured - n_prev)/dt                    # g/s, 首帧为 0
delta_raw = -(Kp*e + Ki*I_trial - Kd*D + Kr*omega_deg_s)
# |delta_raw| > 30° 且本次积分使输出更深入饱和 → 撤回该次积分
```

| 参数 | 俯仰 | 偏航 | 单位 |
|---|---:|---:|---|
| Kp / Ki / Kd / Kr | 80 / 100 / 0 / 0.1 | 80 / 100 / 0 / 0.1 | °/g、°/(g·s)、°·s/g、s |
| 积分限幅 / 轴限幅 | ±10 / ±30 | ±10 / ±30 | g·s / ° |

滚转默认仍为 `roll_cmd = -0.1 × roll_angle_deg` 且随视觉门控 (`legacy_p`);
可选角度外环 + 角速率内环已实现但**不在 64/64 验收内**, 切换须重新验收。

三轴混控 (轴分别限 30° 后, 单片仍可能超 30°, 需逐片限幅):

```text
servo[0] = +pitch +yaw +roll     servo[1] = +pitch -yaw -roll
servo[2] = -pitch +yaw -roll     servo[3] = -pitch -yaw +roll
```

执行机构上限保持原设置: 机械行程 30°、速率 300°/s、时间常数 10 ms、PWM 20 ms。

## 3. OpenMV → STM32 通信协议 (v2 测量格式)

现有固件侧是 **7 字节** (`FF + x_hi x_lo y_hi y_lo w h`), OpenMV v5/v6 脚本发 **10 字节**
(`AA55 + err_x + err_y + area + status + XOR`), 同样 115200 也**不能通信**;
且 `area` 不能代替宽高判断裁切。建议两端一起升级到显式版本协议, 旧协议保留为人工兼容模式,
**不要靠猜字节长度自动混解析**。

| 偏移 | 字段 | 约定 |
|---|---|---|
| 0–1 | `AA 55` | 帧头 |
| 2 | `version` | 2 |
| 3 | `length` | 24 (含帧头与 CRC) |
| 4–5 | `frame_seq` | uint16 小端, 每张图递增 |
| 6–9 | `capture_time_us` | uint32 小端, **相机侧**测量时刻, 需与 STM32 时钟对齐 |
| 10–13 | `u_q4` / `v_q4` | uint16 小端, 原始中心 ×16 (1/16 像素), 不是红十字误差 |
| 14–17 | `bbox_w` / `bbox_h` | uint16 小端 |
| 18 | `flags` | bit0 detected / bit1 locked / bit2 clipped / bit3 predicted |
| 19 | `reserved` | 0 |
| 20–21 | `exposure_us` | uint16 小端 |
| 22–23 | `CRC16` | 小端, 覆盖字节 0–21 |

CRC 固定 **CRC-16/CCITT-FALSE**: poly `0x1021`, init `0xFFFF`, RefIn/RefOut=false, XorOut=0,
标准向量 `"123456789"` → `0x29B1`。字段逐字节打包, 避免结构体填充/大小端/未对齐读。

`capture_time_us` 不能直接写 UART 发送时刻; 两块 MCU 时钟不同源, 至少测量并补偿固定延迟。
接收侧须校验帧头/版本/长度/CRC/序号/新鲜度/像素范围/flags, 坏包**不刷新**可信测量时间,
重复序号不当新图。READY、发射通知与相机配置握手需另有明确命令/应答 (当前缺失)。

## 4. 数据契约 (SL 数据集)

`guidance_sim/learning/dataset.py` 生成, 输出目录 `guidance_sim/outputs/` (**不入库**, 需单独共享)。

每个架次一份 NPZ: `X`、`y_command_g`、`t_s`、`sample_valid`、`trusted_training_mask`,
另附 `eval_*` 评价数据; 目录另有 `manifest.json`(逐回合参数/参考位置/版本/配置/哈希) 与 `report.json`。

- `X`: **31 维** float32, 只在专家新决策前采一次; 分组见下。
- `y_command_g`: 2 维 (俯仰、偏航) **非重力控制比力请求, 单位 g, 不是舵角**。
- 可信掩码: 专家决策有效 + 原靶命中 + 所选目标误差 ≤ 10 mm + 执行约束通过。
  失败回合保留, 掩码置零。
- 划分按回合 + 独立种子; 标准化只允许用训练集。
- 风真值、目标真值、未来轨迹**不进入** X; 理想自身位姿与静态地图先验**明确保留**。

| 组 | 维度 | 单位 |
|---|---:|---|
| `estimated_goal_relative_L` | 3 | m |
| `own_velocity_W` | 3 | m/s |
| `rotation_WL` | 9 | 1 |
| `actual_control` (pitch,yaw) | 2 | g |
| `previous_command` (pitch,yaw) | 2 | g |
| `latest_detected_ray_L` | 3 | 1 |
| `latest_frame_valid` / `_locked` | 1 / 1 | bool |
| `detection_age` / `estimated_goal_age` | 1 / 1 | s (clip 1) |
| `estimated_goal_valid` | 1 | bool |
| `elapsed_time` | 1 | s |
| `declared_goal_offset_W` | 3 | m |

首批实测: 72 架次 (48/12/12)、6959 条原始记录、原物理靶命中 71/72;
可信决策 2806/720/721。唯一失败回合 `train_00037` 有 172 帧相机采样、只检测到 1 帧且 0 帧被接受 —
**说明"目标不可用/失锁处置"没有被这批数据覆盖**, 后续 NN 接管必须显式处理该状态。

## 5. 配置契约

- `profiles/` 是**共用运行配置** (场地、镖体、相机、视觉阈值、v1 飞控、理想导引); 入口用
  `--flight-config` 可覆盖。
- `traditional_control/config.json`、`pn_expert_v2/config.json`、`learning/*.json` 是**各入口独立方案**,
  结构各自演进, 不要跨入口复制字段。
- 权威性: 某入口的成绩只对应它自己的 config; 例如把增强版的 PID 数字填进 v1 入口**不能**复现 64/64。
- 交互调参只改草稿并重置本轮; **只有显式导出 JSON 才写参数文件**。

## 6. 验收成绩与适用边界

| 对象 | 结果 | 边界 |
|---|---|---|
| 传统控制增强版, 联合公差 64 发 (seeds 5101–5164, randomization seed 91003) | **64/64 命中**; 同视觉原 v1 为 3/64 (4.69%); 95% Wilson 94.34%–100%; 接触点到靶心均值 20.31 mm / P95 23.12 mm; 异常 0 | 质量 ±1% / 初速 ±0.1 m/s / 离架 roll ±1° / Cd ±5%, 横风 0; 随机移动靶在 0–0.6 s 换位, 镖约 1.95 s 到靶 → 属**换位后静止命中**, 不等于持续移动拦截 |
| PN专家v2, 29 组真实视觉工况 | 29/29 命中; 同初值关控对照 3/29; 最大偏靶 1.516 mm / P95 1.436 mm; 最大控制过载 0.113571 g | 固定目标; XYZ ±5 cm、侧风 ±1 m/s、偏航 ±0.2°、俯仰 ±0.1°、速度 ±0.05 m/s |

**明确不覆盖**: 实机性能、真实相机内参与曝光时延、IMU 安装/标度/零偏、AHRS 飞行精度、
自身导航 (`p/v`)、末端持续移动 (此前仅 14/32)、HIL、真实舵效与质量/Cd 误差。
气动与舵效全部为**工程估计**; 几何接触命中不等同官方裁判识别。成绩是当前模型与公差分布下的
经验结果, 不证明全局最优或实机 100%。

## 7. 上板对接路线 (摘要, 详细见移植说明 §7)

| 顺序 | 做什么 | 验收看什么 |
|---|---|---|
| 1 | 确认实际烧录脚本版本; 统一 OpenMV/STM32 协议与握手 | 原始 u/v、状态、序号、时间正确; 半包/粘包/坏 CRC/回卷不重放旧帧 |
| 2 | 补 ROI 重捕获, 区分实测与 COAST; 标定内外参 | 移出小 ROI 后能重锁; 改红十字不改变传出的原始像素 |
| 3 | 标定 IMU 轴/标度/时基与舵机通道, 接通 PWM | 单轴转动、单轴舵请求与记录符号一致; 脉宽/行程符合配置 |
| 4 | 新增两套过载 PID, 先用受控输入做单轴验证 | 与 Python 参考逐位对照; 持续限幅不朝错误方向累积积分 |
| 5 | 离架测速 + 短时自身导航 + 图像时间对齐 | 静止目标在相机转动/roll 时世界射线仍稳定 |
| 6 | 移植目标观察器与预测 PN, 保留旧模式 | 同输入下目标估计/ZEM/J/g 请求/舵角可与 Python 对照; 满足 20 ms 决策 + 5 ms AP |
| 7 | 用真实硬件误差更新仿真并做闭环回放/HIL | 导航/相机/时延/舵效误差进入验收后, 再评价整链命中率 |

参考代码在 `firmware_examples/` (`openmv_guidance_example.py`、`stm32_control_example.h/.cpp`、
`stm32_task_integration_example.cpp`), 三个必须由工程实现的平台接口:
`Platform_PublishVisionFromIsr` / `Platform_ReadControlInput` / `Platform_WriteServoPwm`。
已完成的主机侧检查: C++/Python 过载 PID 600 步最大舵角差约 `3.74e-6°`、积分差 `1.60e-8 g·s`;
OpenMV 打包 → C++ 解析 80 帧交叉验证 (含坏 CRC), 25 个拆包位置与回卷通过。
**这些只证明数值与协议可对照, 不代表相机/USART DMA/实时性/导航/舵机闭环已验收。**

## 8. 快速复现

```powershell
py -3.11 -m venv guidance_sim/.venv
.\guidance_sim\.venv\Scripts\python.exe -m pip install -r guidance_sim/requirements.txt

# 64 发联合公差对照 (增强版 vs 原 v1)
.\guidance_sim\.venv\Scripts\python.exe guidance_sim/traditional_control/run.py --headless `
  --target-mode random_moving --randomize match --randomization-seed 91003 `
  --seeds (5101..5164) --compare-v1 --output guidance_sim/outputs/traditional_control_recheck

# PN专家v2 单工况
.\guidance_sim\.venv\Scripts\python.exe -B guidance_sim/prototypes/ideal_guidance_sim.py `
  --flight-config guidance_sim/pn_expert_v2/config.json --headless `
  --position-offset-m 0.05 -0.05 0.05 --wind-cross-mps 1 `
  --output guidance_sim/outputs/pn_expert_v2/example

# 采集 48/12/12 专家数据
.\guidance_sim\.venv\Scripts\python.exe -B -X utf8 guidance_sim/learning/dataset.py --counts 48 12 12
```

依赖: Python 3.11 (需 Tkinter) + `guidance_sim/requirements.txt` (MuJoCo 3.16.0 等)。
结果 JSON 含预定种子、逐发真实参数与标称模型、命中事件、接触/过面误差统计、异常计数与 Wilson 区间;
异常计失败, **`--trace` 默认关闭以免产生大文件**。

## 9. Git 边界

入库: 源码、配置、文档、`guidance_sim/assets/` 的 STL/OBJ 运行资产、`firmware_examples/`。
不入库 (`guidance_sim/outputs/`、`.venv/`、`__pycache__/`、`local_originals/` 共约 1.1 GB 原始 CAD):
验收报告、截图、NPZ 数据集、逐步轨迹需**单独共享**——
尤其数据集与 `outputs/traditional_limit/` 下的证据不在仓库里, 复现时需重新生成。
资产目录整体入库 (`guidance_sim/assets/**` 不受忽略规则约束), 因为它们是**运行输入**
(CAD 管线导出) 而非可再生产物; `.gitattributes` 对其按原始字节存储, 保证 SHA256 登记有效。

## 10. 变更记录

- 2026-10-20: 初版。汇总六个入口、接口/数据/配置契约、验收边界与上板路线;
  同期移植说明入库为 `docs/control_strategy_and_firmware_porting.md`。
- 2026-10-21: 本分支精简为**制导线独立仓** — 移除风洞 CFD 线、Phase 0 占位骨架及协作方文档,
  根 `README.md` 重写为本仓总览, `requirements.txt` 只保留实际使用的 mujoco/numpy/opencv/Pillow;
  本文的接口与数据契约内容不变。
