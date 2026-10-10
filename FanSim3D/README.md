# FanSim3D — 抽风式离心风机仿真项目 使用手册

并行于 WindTunnel3D 的姊妹项目: 自研**抽风式离心风机** (吸入口轴向进气 → 叶轮 → 蜗壳 →
轴向出口方孔 → 出口法兰 → 风洞前置扩散段)。为桌面风洞 (试验段 300×200, 设计风速 25 m/s)
供风。全流程 MATLAB 为核心, GPU 动态求解复用 WindTunnel3D 的 FluidX3D 管线
(动态渲染口径一致: 体素化 → RAW → C++ 生成 → OpenCL 求解 → VTK → MATLAB 交互浏览)。

**参考图构型**: 前弯多叶离心风机 (蜗牛壳 + 钟形吸入口 + 前盘/叶片/后盘)。
默认参数 = `fan_sweep` 前弯家族最优点 (2026-09-27 扫描)。

---

## 一、最快上手 (60 秒)

```matlab
cd D:\FanSim3D
fan_performance            % 1D 性能: 曲线+工况点+稳定性指标 (秒级)
fan_sweep                  % 叶片角度×长度 扫描 → 选型图谱 fan_sweep.png
fan_build_model            % 3D 建模 + impeller.stl / volute.stl (打印件) + fan_model.png
fan_report                 % 一页设计报告 fan_report.png

% GPU 动态求解 (三步, 与 WindTunnel3D 同款流程)
export_fan_fluidx3d        % ① 体素化 + setup_fan.cpp 生成 (WT_QUICK=1 冒烟)
build_fan.bat              % ② 双击编译 (MSVC + /DMOVING_BOUNDARIES)
fluidx3d\windfan.exe       % ③ GPU 求解 → fluidx3d\vtk\ + fan_diag.csv
explore_fan                % ④ 回 MATLAB 交互浏览切片 + 流量时程
fan_mov                    % ⑤ 延时动画 fan_timelapse.mp4 (双切片+壁面轮廓+叶片位置+Q(t) 小图)
```

## 二、设计结论 (2026-09-27 扫描)

| 项 | 值 | 说明 |
|---|---|---|
| 叶轮 | D2=460, D1=276, b2=240 mm, Z=32 | 叶片径向长 92 mm, 厚 3 mm |
| 叶片出口角 β2A | **130° (前弯)** | 前弯家族最优; 全域最优为后弯 60° (η +4%) |
| 转速 | 960 rpm (6 极直联) | 变频 ~790-1180 rpm 覆盖 20-30 m/s |
| 工况点 | Q=1.46 m³/s → **试验段 24.3 m/s** | 风洞阻力 K=148, 设计点压降 315 Pa |
| 轴功率 | 1.12 kW | η静=41%, 喘振裕度 68%, 功率预算 <2.2 kW ✓ |
| 蜗壳 | 外廓 Ø1051 mm, 出口 300×270 | 轴向出口方孔 = 风洞前置扩散段入口 |

风机链路: 钟形口 → 吸力眼 (Ø375) → 叶轮 (轴向 0.09~0.34m) → 蜗壳 → 出口短管 →
**法兰 = WindTunnel3D `tunnel_params` 的 `P.fd.W_in/H_in`** — 两边参数已对齐。

## 三、可调参数 (`fan_params.m`)

| 参数 | 含义 | 默认 |
|---|---|---|
| `P.imp.beta2A` | 叶片出口安装角 (>90 前弯) ← 核心变量 | 130 |
| `P.imp.D1` | 叶片进口直径 (叶片长 = (D2−D1)/2) ← 核心变量 | 0.276 |
| `P.imp.D2 / Z / b2` | 外径 / 叶片数 / 叶宽 | 0.46 / 32 / 0.24 |
| `P.imp.rpm` | 转速 | 960 |
| `P.imp.beta1A` | 进口角 ([]=按设计风量无冲击自动+4°冲角) | [] |
| `P.vol.*` | 蜗壳型线/宽度/舌隙/出口/回收效率 | 见文件 |
| `P.fd.W_in/H_in` | 出口法兰 (须与 tunnel_params.P.fd 一致) | 0.30/0.27 |
| `P.gpu.*` | dx/步数/旋转开关 | 0.003 / 30000 / false |
| 预设 | `fan_params('backward'/'compact'/'hiStatic')` | balanced |

## 四、模型方法与诚实声明

- **1D 平均流面模型** (`fan_performance`): 欧拉方程 + Wiesner 滑移 + 损失族。
  给趋势与设计点, 绝对效率 ±15~20% — **GPU 求解为最终裁判**。
- **GPU 求解** (`windfan.exe`): FluidX3D LBM + SUBGRID LES。叶片 = TYPE_S|TYPE_X
  动壁 (u=ω×r); 默认**冻结转子**; 出口自适应背压按风洞阻力 K·Q² 自寻优;
  每 100 步输出 Q/转矩/功率 → fan_diag.csv; 20 快照 + 时均/湍流度 VTK。
- **瞬态旋转叶轮** (`P.gpu.rotate=true`): 实验性 (重体素化路径待调), 默认冻结转子。
- STL 为双面面片, **切片打印前请用 CAD/网格修复工具闭合**。
- 1D 模型绝对效率 ±15~20% (经验系数), 扫描图谱用于**相对选型**。

## 五、文件清单

| 文件 | 说明 |
|---|---|
| `fan_params.m` | ★ 全部参数与预设 |
| `tunnel_load.m` | 风洞系统阻力曲线 (读 WindTunnel3D 参数, K 分解前/后段) |
| `fan_performance.m` | ★ 1D 平均流面性能模型 (曲线/工况点/稳定性) |
| `fan_blade_arc.m` | 圆弧叶片构造 (单一真源: 弧长+全形) |
| `fan_blade_mask.m` | 叶片掩码共享帮手 (与 WindTunnel3D 耦合导出同源) |
| `fan_sweep.m` | β2A × D1/D2 扫描 → 选型图谱与推荐 |
| `fan_geometry.m` | 3D 布局尺寸表 + 蜗壳型线 |
| `fan_build_model.m` | 3D 渲染 + 设计检查图 + impeller/volute STL |
| `fan_voxelize.m` | CFD 域体素化 (静止壁/旋转叶片/旋转区清单) |
| `export_fan_fluidx3d.m` | ★ MATLAB → FluidX3D 衔接器 (动壁 C++ 生成) |
| `build_fan.bat` / `fluidx3d\windfan.exe` | GPU 求解编译/执行 |
| `explore_fan.m` | GPU 结果交互浏览器 (切片 + Q 时程) |
| `fan_mov.m` | 延时动画导出 (fan_timelapse.mp4, 帧帧自含信息) |
| `fan_report.m` | 一页设计报告 (含系统损失分解条形图) |

## 六、已知边界

- **GPU 工况点偏低**: 粗网格下叶片保真度不足, Q 远低于 1D 设计点 (见七的量化数据)。
  **改善路径**: WT_DX=0.002 + 叶片加厚 (P.imp.tB=0.005) + 更长步数;
  绝对性能以 1D 模型为准, GPU 用于流场结构/相对比较。
- **旋转叶轮瞬态重体素化** (P.gpu.rotate=true) 实验性, 默认冻结转子 (已验证稳定)。
- 蜗壳轴向出口的 90° 转向损失按 ζ=0.20 计入系统阻力。
- 前置扩散段面积比 4.4:1 依赖整流网+纱网镇流, 用 WindTunnel3D 稳态管线复核。

## 七、与 WindTunnel3D 的耦合管线 (风机 = 风洞的真实风源)

风机与风洞拼成**一个 CFD 域**: 增压箱→钟形口→叶轮(冻结转子动壁)→蜗壳→短管→
前置扩散段→稳定段→收缩段→试验段→自适应背压出口。入口在 WindTunnel3D:

```matlab
export_fluidx3d_fan         % 耦合域体素化 + setup 生成 (读 tunnel_params + fan_params)
build_windtunnel_fan.bat    % 编译 (4mm 全档 ~46M 格 / QUICK 6mm 冒烟)
fluidx3d\windtunnel_fan.exe % 求解 → fan_diag.csv(Q/转矩/功率) + forces.csv + VTK
explore_gpu                 % 自动识别耦合模式 (叶轮红色渲染 / 测试物 raw=3 / x0 扩展)
```

**环境开关** (export_fluidx3d_fan):

| 开关 | 作用 | 建议 |
|---|---|---|
| `WT_QUICK=1` | 6mm/4000 步冒烟 (~10 min) | 首跑必用 |
| `WT_DX=0.004` | 全档 4mm (~46M 格, ~35 min) | 定性流场 |
| `WT_DX=0.002` | 2mm (~93M 格, ~1h+, 8GB 可容) | **定量泵送评估** |
| `WT_STEPS=8000` | 步数覆盖 (系统充压 ~0.3s, 步数够才有真工况点) | 定量时 ≥15000 |
| `WT_SPIN=-1` | 反转叶轮 (抽/排风方向切换) | 校验旋向 |
| `WT_OBJ=cylinder/sphere` | 试验段放测试物 (raw=3, forces.csv 出 Cd/Cl) | 默认 none |

**实现优化记录 (2026-09-28, 全部实证)**:

- y-z 解包序修复 (`floor(n/nx)`; 原 `mod(n, ny·nz)` 是 z-最快序公式 → 旋转速度乱码) — 根因级修复
- 后盘内径 = 眼径 (吸入口穿孔; 原误用轴毂半径, 进气被堵 99.4%)
- **毂实体填死眼→出口轴向短路** (叶片内缘 r1 以内的中心柱; 短路使离心泵送无法建立)
- 蜗壳末端 60° 扇区切向出口转型段 (原仅中心方孔, 外层气流出不去)
- 叶片 CFD 加厚 ≥1.2 格 / ν 下限 0.005 (与标准管线 4mm 一致) / 动壁速度每诊断步重写
- 动量匹配出口修复后: 全涵道 4mm Q=+1.03 m³/s (设计 71%), 单机 6mm 冒烟 Q=+1.64 m³/s (自由供风, 与 1D 同量级) — 出口 E 边界 u=0 固定值是静滞风场总根因, 已改为自洽外流速度; 早期 4mm 的 Q=+0.005 是黏性主导+出口憋死的复合结果
  (4mm 下叶道 ~6.7 格的保真度极限), 定量结论以 1D 为准、2mm 复核
