# WindTunnel3D — 桌面级风洞仿真项目 使用手册

目标: 试验段 300×200×200 mm, 稳定风速 20–30 m/s (设计点 25 m/s)。
全流程以 MATLAB 为核心; 动态风场可选 GPU 加速 (FluidX3D, 已装好)。

> **2026-09-27 更新**: 风洞起点新增**前置扩散段** (风机出口法兰 300×270 → 稳定段入口
> 735×490, L=500mm, 面积比 4.4:1, 出口整流网) — 对接姊妹项目 **FanSim3D** (现为套件
> `../FanSim3D`) 的抽风式离心风机 (D2=460mm 前弯多叶, 960rpm, 1D 工况点 24.3 m/s)。
> 风洞全长 2.0 → 2.5 m, 该段折入稳定段几何 (xSec 索引不变), 前置扩散段的分离/整流用
> `visualize_3d` 的 R 判据与 FanSim3D 的 GPU 求解复核。

> **2026-09-30 更新 (风机定向修复)**: 风机耦合布局重排 — 旧版风机与风洞同轴内联 (叶轮绕
> x 转, 进气沿 -x), 本质问题: 离心风机应为侧置 (吸入口轴向进气 + 蜗壳切向排出)。新布局:
> **抽风口面朝 +y** (大气盖移到 +y 域面, 气体沿 -y 抽入) | **叶轮绕 y 轴旋转 (xoz 平面)**
> | **蜗壳 xz 螺旋, 底部切向出口沿 +x** (旋转切线方向) 喷入前置扩散段。旋向配对: 螺旋逆
> 时针生长 ↔ ω<0 (绕 +y), `WT_SPIN=-1` 整机 z 镜像。叶轮截面 (D2=460 前弯多叶) 与 1D
> 性能模型原样复用 FanSim3D; 耦合域 738×227×296 = 49.6M 格 (dx=4mm); 后处理 meta 新增
> y0/z0 域原点 (风机耦合域 y/z 不再对称), explore_gpu/gpu_report/whole_duct_mov/probe_adj
> 已适配。旧朝向的 43GB VTK 快照移入 `fluidx3d/vtk_old_inline/` 存档。
>
> **[P0] 快照镜像修复 + 套件化 (2026-09-30)**: FluidX3D 的 `update_fields()` 在本配置下
> 重构的 rho/u 镜像损坏 (实测 rho 0.61~2.02、u 流体区低 ~8 倍) — 修复前所有瞬时 u/rho
> 快照均不可信 (mean_u.vtk 时均与 fan_diag.csv 诊断不受影响)。三套生成器已在快照前移除
> 该调用; 新增 `WT_VTK` 档位 (默认只写 u 场省 ~2/3 磁盘, `WT_VTK=all` 附加 rho+F)。
> 项目与 FanSim3D 并列迁入套件 (根见 `../README.md`), 跨项目路径全部改为 `mfilename`
> 相对定位 + `%~dp0` bat, 整个套件文件夹可任意搬移。

---

## 一、最快上手 (30 秒)

```matlab
cd D:\WindTunnel3D
visualize_3d      % 稳态仿真 + 自动出 5 张图 (关掉图窗即结束)
explore_3d        % 交互浏览器: 拖滑块从任意方向切片看流场
```

想看**动态风场** (测试物绕流、涡脱落、气动力), 三选一:

```matlab
lbm_run           % MATLAB 版: 1~15 分钟, 演示级 Re (教学/快速验证)

% GPU 版 (真实 Re, 推荐): 三步
export_fluidx3d   % ① 生成体素网格 + C++ 源码 (MATLAB 里跑)
                  % ② 双击 build_windtunnel.bat 编译 (换参数后需重跑①②)
                  % ③ 双击 fluidx3d\windtunnel.exe GPU 求解 (~6 分钟, 20 快照 + 力时程)
explore_gpu       % ④ 回 MATLAB 交互浏览 GPU 结果 + 气动力面板
```

放入你自己的测试物: 把 CAD 模型导出为**二进制 STL**, 存为
`D:\WindTunnel3D\test_object.stl` — 之后所有求解器自动使用它
(包围盒中心自动对齐试验段中心; 无此文件时回退演示球)。

---

## 二、六条命令速查 (在 MATLAB 命令行输入名字即执行)

| 命令 | 干什么 | 什么时候用 |
|---|---|---|
| `build_tunnel_model` | 生成风洞 3D 模型, 导出 `tunnel_body.stl` + 侧视/壁面角检查图 | 改了尺寸后; 或要把模型导入 CAD 与你的 STEP 装配 |
| `contraction_study` | 收缩段长度×收缩比参数扫描, 给出推荐设计点 | 设计选型阶段 |
| `boundary_layer_study` | 腔体长度→边界层/核心加速扫描 + PLA 粗糙度对比 | 评估摩擦分层、定试验段长度 |
| `visualize_3d` | 稳态流场求解 + 自动出 4 张分析图 + 体渲染主图 + GIF | 快速看稳态全貌 (默认档约 2 分钟) |
| `explore_3d` | 稳态结果交互浏览器 (不重算) | 细看稳态场, 拖滑块任意切片 |
| `lbm_run` | MATLAB 版动态仿真 (瞬态 LBM) | 看涡脱落/气动力时程 + 涵道内壁3D渲染, 快速验证 |

前三条**不求解流场** (设计与建模); 后三条各属一条求解管线 (见下表)。

---

## 三、三条求解管线 (按精度/速度选)

| | ① 稳态管线 | ② MATLAB 动态 | ③ GPU 动态 (FluidX3D) ★ | ④ 风机耦合 GPU ★★ |
|---|---|---|---|---|
| 求解器 | `flow3d_solve` (定常重建+边界层模型) | `lbm3d_solve` (D3Q19 TRT) | FluidX3D 3.7 (OpenCL, 已编译) | FluidX3D + 旋转叶轮动壁 |
| 入口命令 | `visualize_3d` | `lbm_run` | `export_fluidx3d` → bat → exe | `export_fluidx3d_fan` → `build_windtunnel_fan.bat` → `windtunnel_fan.exe` |
| 结果查看 | `explore_3d` | 自带 3 张图 | `explore_gpu` (同款交互) | `explore_gpu`/`gpu_mov`/`gpu_report` 自动兼容 |
| 风源 | 理想来流 | 理想来流 | 理想来流 | **FanSim3D 旋转叶轮实际激发** |
| 适用 | 设计迭代/快速全貌 | 教学演示/趋势验证 | **正式结果/交付** | 风机-风洞一体化动态验证 |

### ③ GPU 管线细节

试验段入口 25 m/s 精确匹配, 出口运动水库贯通; 测试物 TYPE_S|TYPE_X 单独标记,
动量交换法输出 `forces.csv` (每 100 步采样, 满足涡脱 Nyquist) → 浏览器面板看
Cd/Cl 时程、后半段均值与 St=f·d/U (Cl 频谱峰)。被测物几何自动测算: 导出时从体素
mask 量出包围盒/**真实迎风投影面积** 与**面积堵塞比** — >10% 提示 Cd 偏大, >25%
提示 LBM 裕度下降。**入口合成湍流**: 流函数法横向涡旋脉动 (y-z 面严格无散, 每分量
6 模式, 波长 20~80 格, rms 归一至 Tu 链设计值 0.46%, 每 100 步更新相位) — 实测
试验段入口 Tu=0.31~0.42% ✓。**逐步时均/湍流度**: 后半段每 100 步累加 Σu/Σu²,
落盘 `vtk/mean_u.vtk` + `vtk/tu.vtk` — explore_gpu 直接读取。2mm 晶格 600 万体素,
每 1500 步一个 VTK 快照共 20 个, 实测求解约 7 分钟 (RX 6650 XT)。**高频切片时序
流**: 每 100 步把 y=0/z=mid 切片 u 三分量流式写入 `fluidx3d/u_slice_y0/zm.raw`
(~310MB) — 供 gpu_slices 出时空图与切片电影。3D VTK 快照相对涡脱周期是时间欠采样
(动画结构跳变是采样问题), 连续动力学证据看 gpu_slices 时空图斜条纹。名义晶格
Re=1200 + SUBGRID LES (Smagorinsky) 补足高 Re 湍流行为。几何/测试物变更后重跑
export 自动把旧一代结果归档到 `fluidx3d\vtk_prev\`。

### ④ 风机耦合管线 (风源=真实叶轮, 2026-09-27 新增)

`export_fluidx3d_fan` 把姊妹项目 **D:\FanSim3D** 的抽风式离心风机 (D2=460mm 前弯多叶,
960rpm) 与风洞拼成**一个 CFD 域** (2026-09-30 侧置定向: 风洞工作方向=x | 抽风口面朝 +y,
外部气体沿 -y 被抽入 | 叶轮绕 y 轴在 xoz 平面旋转 | 蜗壳 xz 阿基米德螺旋收集, 自底部
切向出口沿旋转切线 +x 方向喷入风洞): 大气(+y 域面 E) → 增压箱 → 钟形口 → 吸力眼 →
旋转叶轮(冻结转子动壁, 绕 y 轴) → 蜗壳(切向出口) → 渐扩短管 → 前置扩散段(整流网=2格
棋盘穿孔板) → 稳定段 (蜂窝器/纱网折入背压) → 收缩段 → 试验段 → 自适应背压出口(后扩散段+排空)。

- 流程: `export_fluidx3d_fan` → `build_windtunnel_fan.bat` → `fluidx3d\windtunnel_fan.exe`
  → `explore_gpu` (自动识别耦合模式: x0/涵道壁扩展, 叶轮渲染为红色, 测试物 raw=3)
- 标记: raw=2 叶轮(TYPE_S\|TYPE_X 动壁, 输出 fan_diag.csv: Q/转矩/功率), raw=3 测试物
  (TYPE_S\|TYPE_Y, forces.csv 照常输出 Cd/Cl)
- 档位: WT_QUICK=1 → 6mm/4000 步冒烟; 默认 4mm/15000 步; WT_DX 覆盖
- **使用办法**: ① `export_fluidx3d_fan`(WT_QUICK=1 首跑) ② `build_windtunnel_fan.bat`
  ③ `fluidx3d\windtunnel_fan.exe` ④ `explore_gpu` — 与标准管线互斥使用
  (meta/raw 同名, 切换管线后必须重跑对应 export!)
- 开关: `WT_STEPS=步数` 覆盖 (系统充压 ~0.3s, 步数不足无真工况点) |
  `WT_SPIN=-1` 整机 z 镜像反转 (螺旋/出口/旋向同步配对) | `WT_DX=0.002` 定量档
- 物理预期: 叶轮激发实际风场 — 蜗壳旋流、叶轮尾迹、整流网/收缩段的旋流抑制全部真实求解,
  试验段来流品质(旋流残留/湍流度)可直接从 tu.vtk 读出 — 这是 ①②③ 给不了的
- **整体视频流**: 渲 whole_duct_mov → gpu_whole_fan.mp4 (纵切 |U| + 涡量扰流面板, 成对插值流畅化)。2026-09-30 侧置定向首跑收敛: Q=+1.32 m³/s → 试验段 ~21.5 m/s (1D 设计 1.46/-24.3, 峰值 1.45; 差值为真实蜗壳/纱网损失 4mm 分辨率落点), 快照见 vtk/、流场核验图 fan_flow_check.png、几何切片图 slice_check.png — 叶道仅 ~6.7 格的分辨率保真度
  极限; 定性旋流/可视化有效, 定量以 FanSim3D 的 1D 模型为准、2mm 档复核

## 四、所有可调参数 (`tunnel_params.m`)

### 构型预设 (一键换整体方案)
```matlab
P = tunnel_params('balanced')   % 均衡 (默认): R=6, L=600mm, 矩形试验段
P = tunnel_params('lowTurb')    % 低湍流: R=8, L=420mm (五次型线)
P = tunnel_params('compact')    % 紧凑: R=5, L=220mm (三次型线)
P = tunnel_params('aniso')      % 矩形试验段 200×120mm
```
命令行切换: `setenv('WT_CONFIG','lowTurb')` 后跑任意脚本。

### 常用单项参数 (改数字即生效)
| 参数 | 含义 | 默认 |
|---|---|---|
| `P.V_test` | 试验段设计风速 (m/s) | 25 |
| `P.ct.R` / `P.ct.L` | 收缩比 / 收缩段长 (m) | 6 / 0.60 (入口 735×490) |
| `P.ct.curve` | 收缩型线: quintic/cubic/sine/linear/twostage | quintic |
| `P.ts.W/H/L` | 试验段宽/高/长 (m) | 0.30/0.20/0.20 |
| `P.sc.L` | 稳定段长度 (m) | 0.45 |
| `P.fd.W_in/H_in/L` | **前置扩散段** 风机法兰宽/高/长 (m) — 对接 FanSim3D 风机出口 | 0.30/0.27/0.50 |
| `P.df.W_out/H_out/L` | 扩散段出口宽/高/长 (m) | 0.40/0.26/0.75 (面积比 1.73, 2θ≈5.6°) |
| `P.wall.ks` | PLA 壁面粗糙度 (m) | 0.0002 (打磨后可 0.00005) |
| `P.flow.dx_cell` | 稳态网格目标单元尺寸 (m), 网格数由此自动派生 | 0.003 |
| `P.lbm.object` | 测试物: 'stl'(你的模型)/sphere/cylinder/plate/naca | 'stl' |
| `P.lbm.objFile` | STL 路径 | 'test_object.stl' |

### 环境变量开关 (MATLAB 里 setenv, 不改代码)
| 变量 | 作用 |
|---|---|
| `WT_QUICK=1` | 快速低分辨率档 (冒烟验证) |
| `WT_NOGIF=1` | 跳过 GIF 生成 (分析图秒出) |
| `WT_RESOLVE=1` | 强制重新求解 (忽略已有存档) |
| `WT_CONFIG=<名>` | 切换构型预设 |
| `WT_T=<时间步>` | explore_gpu 直接打开指定时刻快照 |
| `WT_FIELD=<场>` | explore_gpu 初始场 (spd/vort/q/rho/mean/turb, 无头快照导出用) |
| `WT_OBJ=<类型>` | 覆盖测试物 (export_fluidx3d / lbm_run: sphere/cylinder/plate/naca) |
| `WT_NOOBJ=1` | 空风洞基线模式: 被测物几何仍测算但不写入流场 (→ mean_u_base/tu_base, gpu_report 自动对照) |
| `WT_DX=<m>` | GPU 晶格覆盖 (如 0.001=1mm 档, 格数/耗时按 dx 三~四次方涨) |
| `WT_MOV=[N]` | explore_gpu 导出时序动画 gpu_timelapse.mp4 (=1 全部快照, =N 每 N 个取 1) |
| `WT_CP=1` | 无头快照/动画中强制开启表面载荷着色 |
| `WT_MID=1` | 稳态中档 (单元 ×2, 求解 ~25 s 快速查看) |

### 稳态精度档位 (flow3d_solve, 网格随几何自动派生)
| 档位 | 触发 | 体素 (300×200 试验段) | 单元尺寸 |
|---|---|---|---|
| 快速 | WT_QUICK=1 | 111×43×32 | ~18 mm |
| 中档 | WT_MID=1 | 334×130×87 (380 万) | ≈6 mm |
| **默认 (最高)** | 直接运行 | **667×260×173 (3000 万)** | **≈3.0 mm** (y/z 独立适配矩形截面) |
网格由 `P.flow.dx_cell` (默认 3mm) 按全长/入口宽/入口高自动派生 — 改试验段尺寸后档位自动跟随, 不再失配。
存档 flow3d_result.mat 大场字段为单精度, 内存需求 ~3.5 GB。

> GPU 说明: `flow3d_solve` 的批量泊松投影仅在检测到 **NVIDIA CUDA** 设备时用 gpuArray 加速;
> AMD/Intel 显卡无 CUDA (MATLAB gpuArray 不适用, 装了 PCT 也一样), 稳态路线始终 CPU 运行 —
> 本机 (RX 6650 XT) 即此情况, GPU 加速一律走 FluidX3D (OpenCL) 管线, 控制台会明确提示原因。

---

## 五、交互浏览器 (explore_3d / explore_gpu) — 双窗口布局

- **主视图 (3D)**: 涵道内壁 + 体填充 + 测试物/边界层, 截面位置只画**青/品红虚线轮廓标尺**,
  不遮挡模型; 拖滑块时标尺与游标实时移动
- **GPU 主视图渲染**: 按场类型自适应等值面 — 速度/时均场画 0.90V 尾流/边界层壳
  (均匀核心完全透明, 一眼看穿洞内), 涡量/**Q 准则**/湍流度画尾流结构嵌套壳;
  + 薄雾层补体感 + 头灯光照
- **Q 准则涡结构** (仅 explore_gpu): Q = ½(‖Ω‖²−‖S‖²) 逐快照由速度场解算 —
  涡旋转强于剪切变形的连通结构, 涡环/展向涡比涡量幅值干净一个量级
- **表面载荷着色** (仅 explore_gpu): 快照同步落盘 `F_*.vtk` (体素级气动力矢量),
  测试物表面按 |F|/q∞ turbo 着色 (勾选"表面载荷") — 相当于数值压力敏感漆,
  迎风面高压/肩部峰值/尾流低压一眼可辨
- **时序动画导出**: MATLAB 里 `setenv('WT_MOV','1')` 后跑 explore_gpu →
  gpu_timelapse.mp4 (涡脱演化/尾流发展全程回放)
- **截面分析窗 (独立窗口, 自动弹出)**: 竖直截面 (x×z) 与水平截面 (x×y) 两幅场图,
  带涵道壁黑线与测试物蓝线轮廓, 拖动滑块实时刷新
- **鼠标左键拖拽**: 旋转主视图; 滚轮: 缩放
- **竖直截面 y 滑块**: 流向×高度纵剖; **水平截面 z 滑块**: 俯视风场
- **x 裁剪滑块**: 隐藏下游段, "看进洞内"
- **显示场切换**: 流速 / 涡量 / 涡粘比 (稳态) 或 流速/涡量/密度/**时均流速/湍流度 Tu%** (GPU)
- **时均/湍流度** (仅 explore_gpu): 优先直读 GPU 逐步统计 `mean_u.vtk`/`tu.vtk`
  (求解器每 100 步累加, 无欠采样, 秒开); 旧存档自动回退后半段快照平均 (Tu 为下界)
  — 时均场 |Ū| 与湍流度 Tu(%), 真实风洞测的就是这两个量
- **气动力面板** (仅 explore_gpu): forces.csv → Cd/Cl 时程曲线 + 后半段均值 +
  Cl 频谱峰涡脱频率与 St 数标注 (动量交换法, 标注参考面积与名义晶格 Re)
- **时间快照滑块** (仅 explore_gpu): 拖动回放 GPU 瞬态演化
- **显隐复选框**: 体填充 / 等值面 / 涵道壁 / PLA 边界层 / 截面标尺 / 截面窗子图
- explore_3d / explore_gpu 只关闭自己的旧窗口, 不会动 visualize_3d 的分析图

---

## 六、完整工作流示例

### 场景 1: 设计一款新风洞
```matlab
cd D:\WindTunnel3D
build_tunnel_model                 % 看侧视图, 检查壁面角 <35°/<7°
contraction_study                  % 选收缩段 L 和 R
boundary_layer_study               % 定试验段长度 (分层 vs 核心加速)
% 满意后改 tunnel_params.m 里的默认值
```

### 场景 2: 稳态流场检查
```matlab
visualize_3d                       % 4 张分析图秒出 (GIF 最后, 可 WT_NOGIF=1 跳过)
explore_3d                         % 切片细看任意截面
```

### 场景 3: 用你的模型测气动力 (GPU, 正式流程)
1. CAD 里把模型导出为二进制 STL → 存为 `D:\WindTunnel3D\test_object.stl`
   (无 STL 时可先用参数化测试物: MATLAB 里 `setenv('WT_OBJ','cylinder')` 后跑 export;
   模型最大可到试验段 300×200×200mm, 导出时自动打印包围盒/迎风面积/堵塞比诊断)
2. (可选, 推荐) 空风洞基线: MATLAB `setenv('WT_NOOBJ','1')` → export → bat → exe,
   完成后 `setenv('WT_NOOBJ','')` — gpu_report 会自动加载基线做对照并给堵塞修正 Cd
3. MATLAB: `export_fluidx3d` (生成网格与源码; 默认 2mm 全档, `WT_QUICK=1` 冒烟)
4. 资源管理器双击 `build_windtunnel.bat` (编译, 换模型后需重跑 3-4 步)
5. 双击 `fluidx3d\windtunnel.exe` (GPU 求解, 完成提示 "wind tunnel done")
6. MATLAB: `explore_gpu` — 切片看绕流, Q 准则看涡结构, 勾选"表面载荷"看模型表面
   气动载荷分布, 面板读 Cd/Cl/St
7. MATLAB: `gpu_mov` — 一键延时动画 `gpu_timelapse.mp4` (默认 Q 准则涡脱演化;
   `gpu_mov('spd')` 换场, `gpu_mov('turb',2)` 加速渲染; mp4 失败自动回退 GIF)
   + 切片分析窗同步留档 `gpu_timelapse_slices.mp4`
8. MATLAB: `gpu_slices` — 连续性档案: 时空图 `gpu_xt_y0.png` (斜条纹=涡连续输运)
   + 高频切片电影 `gpu_slice_movie_y0.mp4` (|u| 与涡量 ω_y 梯度双面板)
   + `gpu_slice_movie_zm.mp4` (水平切片) + 控制台相邻帧相关系数 (连续性定量检验)
9. MATLAB: `gpu_report` — 一页双重测算报告 `gpu_report.png`
   (流场质量/边界层发展/气动力/涡脱频谱/基线对照/堵塞修正汇总)

### 场景 4: 与你的 STEP 装配模型核对
`tunnel_body.stl` 流向为 STL +Z 轴, 任意 CAD (SolidWorks/Fusion/FreeCAD)
可直接导入并与 STEP 模型装配比对。

### 场景 5: MATLAB 动态风场 (快速演示)
`lbm_run` — 自动完成瞬态求解并输出:
- 图1: 测试物 Cd/Cl 时程 + 客观性文献基准线 (自动偏差诊断)
- 图2: 水平/竖直涡量切面 (对称涡街可见)
- lbm_p0_slices.png: **总压切片时序蒙太奇** (jet 彩虹渲染, 视频同款形式)
- lbm_fig3.png: 3D 涵道内壁半透明壳 + 测试物 + 涡量等值面
- 控制台: 客观性验证块 (Cd/St 偏差、分辨率与堵塞比诊断)

---

## 七、文件清单

| 文件 | 说明 |
|---|---|
| `tunnel_params.m` | ★ 全部参数与构型预设 (一切改动从这里开始) |
| `smoke_check.m` | 改参数后的几何/网格/域覆盖快速体检 (秒级, 不求解) |
| `tunnel_geometry.m` | 沿程几何/速度/壁面角 (a/b 双型线 + 5 种收缩型线库) |
| `build_tunnel_model.m` | 建模 + STL 导出 + 设计检查图 |
| `contraction_study.m` / `boundary_layer_study.m` | 设计研究脚本 |
| `flow3d_solve.m` | 稳态体素求解器 |
| `normalShock.m` | 正激波关系模块 (高速工况) |
| `visualize_3d.m` | 稳态求解 + 4 张分析图 + GIF |
| `explore_3d.m` | 稳态交互浏览器 (参数可就地改并重解) |
| `lbm3d_solve.m` / `lbm_run.m` | MATLAB 动态 LBM 求解器与驱动 |
| `tunnel_voxelize.m` | 动态域体素化 (涵道壁 + 测试物 STL) |
| `export_fluidx3d.m` | ★ MATLAB → FluidX3D GPU 衔接器 |
| `read_fluidx3d_vtk.m` | FluidX3D VTK 结果读回 MATLAB |
| `explore_gpu.m` | GPU 结果交互浏览器 (时间回放 + 气动力) |
| `gpu_mov.m` | 一键延时动画导出 (gpu_timelapse.mp4, 重运算标准后处理) |
| `gpu_slices.m` | 高频切片时序留档: 时空图 + 切片电影 + 连续性定量检验 |
| `gpu_report.m` | 风洞+被测物 双重测算一页仪表盘 (gpu_report.png) |
| `build_windtunnel.bat` / `fluidx3d\windtunnel.exe` | GPU 求解编译/执行 |
| `选型与安装清单.md` | 软件选型与安装记录 (FluidX3D 已装) |

结果存档: `flow3d_result.mat` / `lbm_result.mat` / `gpu_meta.mat` +
`fluidx3d\vtk\`(快照流场 u_/rho_/F_ + `mean_u.vtk` 时均场 + `tu.vtk` 湍流度%
+ 空风洞基线 `mean_u_base.vtk`/`tu_base.vtk`) + `forces.csv`(气动力时程) +
`fluidx3d\u_slice_y0/zm.raw`(高频切片时序流) + 视频档案
`gpu_timelapse.mp4`(3D) / `gpu_timelapse_slices.mp4`(切片窗) /
`gpu_slice_movie_y0.mp4` / `gpu_slice_movie_zm.mp4`(高频切片电影) +
`gpu_xt_y0.png`(时空图) + `gpu_report.png`(双重测算) +
`fluidx3d\vtk_prev\`(上一代归档)。

> **2026-09 修复**: `read_fluidx3d_vtk` 原按 SoA 解包矢量 VTK, 而 FluidX3D
> 实际写 AoS (每格点 x,y,z 相连) — 修复前所有矢量场 (流速/涡量) 均带 3 周期
> 分量交错的细微伪影, 修复后分量均值/剖面与理论一致。

### 逆压梯度与边界层分离 (两款求解器均覆盖)
| 求解器 | 方法 | 输出 |
|---|---|---|
| 稳态 `flow3d_solve` | 一维湍流边界层判据: R = (dp/dx·δ*)/τw (壁面摩擦对抗逆压的能力比, R→1 分离) | 逐站 R(x) + 自动判定 + R>0.9 站位自动嵌入近壁反向带 (流场中直接可见分离趋势) |
| `boundary_layer_study` | 同源 R(x) 曲线 + **速度剖面耙图** (扩散段轮廓+各站剖面, 参考图形式) + 陡扩散对比 (角度翻倍→R=1.68 必分离) | 分离风险曲线图 |
| 动态 `lbm3d_solve` | **真实求解**: 贴壁 2 格内逆流 (u_x<0) 体素占比逐站检测 | revFrac(x) + 自动判定 (当前附着流 0%) |

当前设计 (试验段 300×200 → 扩散段出口 400×260, L=0.75 m, 面积比 1.73, 全张角 2θ≈5.6°):
稳态求解判 "逆压显著但余量充足" — 扩散段逆压比 R_max=0.62 (默认 667×260×173 档实测,
R≥1 才分离, 余量 38%), 无近壁反向带嵌入。对比旧 150×150 设计 (面积比 4, 2θ≈12.2°,
R_max≈2.0 已入分离风险区): 大试验段使扩散段张角大幅收敛, 全程附着。
经验判据 2θ≤7° 留有余量; `boundary_layer_study` 的无粘一维口径更保守, 可复查陡扩散对比。

### 壁面摩擦对风速与边界层的影响 (大试验段 300×200 实测)

新设计下大截面使边界层相对更薄 (δ/半宽比旧 150 方形段小一倍), 摩擦增速与
核心均匀性同步改善; PLA 粗糙壁 (ks=0.2mm) 仍使边界层全程湍流 (cf 较光滑壁 +243%)。
逐站数值跑 `visualize_3d` / `explore_3d` 看控制台诊断与切片 (贴壁低速带直接可见),
设计研究用 `boundary_layer_study`。

物理链条: 收缩段强顺压把边界层压在 2mm 内 → 进入试验段后重新生长 → 排挤效应
使核心风速比无粘值略高 → 扩散段逆压使 δ 增厚 (当前张角 2θ≈5.6°, 增厚温和)。
试验段中部分层度 (核心速度不均匀度) <1% — 大截面的直接红利。

---

## 八、动态解算的客观性验证 (每次 lbm_run 自动输出)

lbm_run 结束时会打印"客观性验证"块，把 LBM 结果与文献基准对表：

| 指标 | LBM (快速档) | 文献基准 | 说明 |
|---|---|---|---|
| 圆球 Cd (演示 Re=250) | ~1.5 | 0.70 (Clift-Grace-Weber 相关式) | 偏差 +116%：3.8 格直径的体素阶梯 + 低分辨率 |
| 圆柱 Cd (演示 Re=250) | ~2.3 | 0.70 | 同上，横向圆柱 |
| 圆柱 St (涡脱) | 0.06~0.14 | 0.212 (Williamson) | 方向正确、量级可信；受尾流发展时间与分辨率限制 |
| 出口风速 | 25.0 m/s | 设计值 25 | 精确命中 (通量匹配) |
| 质量守恒 | 各站通量偏差 <3% | 0 | 连续性自检 |

**分辨率与堵塞比诊断**自动打印：直径晶格数 (<10 警告)、面积堵塞比。
**客观性结论**：演示档 Cd 半定量 (趋势与量级正确、数值偏高 1~2 倍)、St 定性正确、
速度场定量可信 (风速/通量精确)。正式定量请走 GPU 2mm 档 + 光滑 STL +
更小测试物 (直径≥15 格)，或转 OpenFOAM。
**文献来源**：Cd — Clift, Grace & Weber (1978)；St — Williamson (1997)。

### GPU 管线实测校验 (圆柱 d=30mm, 15 格直径, 名义晶格 Re=1200 + LES)

| 量 | GPU 实测 (forces.csv 频谱/均值) | 文献 (自由圆柱, Re~10^3) | 评注 |
|---|---|---|---|
| Cl_rms (升力脉动) | 0.35 | 0.3~0.5 | ✓ 量级吻合, 涡脱强而相干 |
| 涡脱 St | 0.41 | 0.21 | 偏高 ~2×: 15% 面积堵塞 + 圆柱贯通壁面 + 体素阶梯, 受限工况 St 上移 |
| 时均尾流 Tu | 尾流核心 60~78% | 同量级 | tu.vtk 逐步统计, 无欠采样 |
| 入口/外流 Tu | 入口 0.66%, 试验段 0.31~0.42% | 设计值 0.46% | 与 Tu 链 (蜂窝器/纱网/收缩衰减) 闭环 ✓ |

面板中 St 由 Cl 后半段 FFT 峰自动标注 (f·d/U); 升力取 Fz (竖直向)。
Cd 在受限+体素阶梯下偏高 (墙面贯通圆柱), 定性趋势与相对比较可用, 绝对值谨慎。

## 九、已知边界 (诚实声明)

- 稳态管线的边界层为子网格模型 (体素是单元平均值); 解析边界层结构需
  `P.flow.dx_cell` 降到 2mm 以下 (格数按 3 次方涨, 注意内存)
- MATLAB 动态 LBM 受晶体稳定上限约束, 演示 Re≤~900; Cd 为半定量
- GPU 管线名义晶格 Re=1200 (以试验段宽度计), SUBGRID LES 补足高 Re 湍流行为;
  2mm 晶格下体素球 Cd 偏高 (阶梯效应), 光滑 STL 模型 + 1mm 档可继续收敛
- 入口合成湍流为流函数法 2D 横向脉动 (y-z 面无散, 无流向分量 — 理想化的 2C 网格湍流),
  相位每 100 步才更新一次; 沿程衰减使试验段入口 Tu≈0.3~0.4% (设计 0.46%)
- Tu 湍流度链为一维经验式 (风扇/蜂窝器/纱网/收缩段衰减), 给数量级与设计趋势,
  非逐站 CFD 结果; GPU 的 tu.vtk 则是脉动直接解算的统计真值 (受晶格分辨率限制)
- 激波模块为 1D 关系式+逐体素马赫标记 (当前 Ma≈0.07 无激波, 教学用途)
- 验证环境: MATLAB R2026a (基础版, 无工具箱) + Windows 11 + RX 6650 XT

﻿# WindTunnel3D — 桌面级风洞仿真项目 使用手册

目标: 试验段 300×200×200 mm, 稳定风速 20–30 m/s (设计点 25 m/s)。
全流程以 MATLAB 为核心; 动态风场可选 GPU 加速 (FluidX3D, 已装好)。

> **2026-09-27 更新**: 风洞起点新增**前置扩散段** (风机出口法兰 300×270 → 稳定段入口
> 735×490, L=500mm, 面积比 4.4:1, 出口整流网) — 对接姊妹项目 **FanSim3D** (现为套件
> `../FanSim3D`) 的抽风式离心风机 (D2=460mm 前弯多叶, 960rpm, 1D 工况点 24.3 m/s)。
> 风洞全长 2.0 → 2.5 m, 该段折入稳定段几何 (xSec 索引不变), 前置扩散段的分离/整流用
> `visualize_3d` 的 R 判据与 FanSim3D 的 GPU 求解复核。

> **2026-09-30 更新 (风机定向修复)**: 风机耦合布局重排 — 旧版风机与风洞同轴内联 (叶轮绕
> x 转, 进气沿 -x), 本质问题: 离心风机应为侧置 (吸入口轴向进气 + 蜗壳切向排出)。新布局:
> **抽风口面朝 +y** (大气盖移到 +y 域面, 气体沿 -y 抽入) | **叶轮绕 y 轴旋转 (xoz 平面)**
> | **蜗壳 xz 螺旋, 底部切向出口沿 +x** (旋转切线方向) 喷入前置扩散段。旋向配对: 螺旋逆
> 时针生长 ↔ ω<0 (绕 +y), `WT_SPIN=-1` 整机 z 镜像。叶轮截面 (D2=460 前弯多叶) 与 1D
> 性能模型原样复用 FanSim3D; 耦合域 738×227×296 = 49.6M 格 (dx=4mm); 后处理 meta 新增
> y0/z0 域原点 (风机耦合域 y/z 不再对称), explore_gpu/gpu_report/whole_duct_mov/probe_adj
> 已适配。旧朝向的 43GB VTK 快照移入 `fluidx3d/vtk_old_inline/` 存档。
>
> **[P0] 快照镜像修复 + 套件化 (2026-09-30)**: FluidX3D 的 `update_fields()` 在本配置下
> 重构的 rho/u 镜像损坏 (实测 rho 0.61~2.02、u 流体区低 ~8 倍) — 修复前所有瞬时 u/rho
> 快照均不可信 (mean_u.vtk 时均与 fan_diag.csv 诊断不受影响)。三套生成器已在快照前移除
> 该调用; 新增 `WT_VTK` 档位 (默认只写 u 场省 ~2/3 磁盘, `WT_VTK=all` 附加 rho+F)。
> 项目与 FanSim3D 并列迁入套件 (根见 `../README.md`), 跨项目路径全部改为 `mfilename`
> 相对定位 + `%~dp0` bat, 整个套件文件夹可任意搬移。

---

## 一、最快上手 (30 秒)

```matlab
cd D:\WindTunnel3D
visualize_3d      % 稳态仿真 + 自动出 5 张图 (关掉图窗即结束)
explore_3d        % 交互浏览器: 拖滑块从任意方向切片看流场
```

想看**动态风场** (测试物绕流、涡脱落、气动力), 三选一:

```matlab
lbm_run           % MATLAB 版: 1~15 分钟, 演示级 Re (教学/快速验证)

% GPU 版 (真实 Re, 推荐): 三步
export_fluidx3d   % ① 生成体素网格 + C++ 源码 (MATLAB 里跑)
                  % ② 双击 build_windtunnel.bat 编译 (换参数后需重跑①②)
                  % ③ 双击 fluidx3d\windtunnel.exe GPU 求解 (~6 分钟, 20 快照 + 力时程)
explore_gpu       % ④ 回 MATLAB 交互浏览 GPU 结果 + 气动力面板
```

放入你自己的测试物: 把 CAD 模型导出为**二进制 STL**, 存为
`D:\WindTunnel3D\test_object.stl` — 之后所有求解器自动使用它
(包围盒中心自动对齐试验段中心; 无此文件时回退演示球)。

---

## 二、六条命令速查 (在 MATLAB 命令行输入名字即执行)

| 命令 | 干什么 | 什么时候用 |
|---|---|---|
| `build_tunnel_model` | 生成风洞 3D 模型, 导出 `tunnel_body.stl` + 侧视/壁面角检查图 | 改了尺寸后; 或要把模型导入 CAD 与你的 STEP 装配 |
| `contraction_study` | 收缩段长度×收缩比参数扫描, 给出推荐设计点 | 设计选型阶段 |
| `boundary_layer_study` | 腔体长度→边界层/核心加速扫描 + PLA 粗糙度对比 | 评估摩擦分层、定试验段长度 |
| `visualize_3d` | 稳态流场求解 + 自动出 4 张分析图 + 体渲染主图 + GIF | 快速看稳态全貌 (默认档约 2 分钟) |
| `explore_3d` | 稳态结果交互浏览器 (不重算) | 细看稳态场, 拖滑块任意切片 |
| `lbm_run` | MATLAB 版动态仿真 (瞬态 LBM) | 看涡脱落/气动力时程 + 涵道内壁3D渲染, 快速验证 |

前三条**不求解流场** (设计与建模); 后三条各属一条求解管线 (见下表)。

---

## 三、三条求解管线 (按精度/速度选)

| | ① 稳态管线 | ② MATLAB 动态 | ③ GPU 动态 (FluidX3D) ★ | ④ 风机耦合 GPU ★★ |
|---|---|---|---|---|
| 求解器 | `flow3d_solve` (定常重建+边界层模型) | `lbm3d_solve` (D3Q19 TRT) | FluidX3D 3.7 (OpenCL, 已编译) | FluidX3D + 旋转叶轮动壁 |
| 入口命令 | `visualize_3d` | `lbm_run` | `export_fluidx3d` → bat → exe | `export_fluidx3d_fan` → `build_windtunnel_fan.bat` → `windtunnel_fan.exe` |
| 结果查看 | `explore_3d` | 自带 3 张图 | `explore_gpu` (同款交互) | `explore_gpu`/`gpu_mov`/`gpu_report` 自动兼容 |
| 风源 | 理想来流 | 理想来流 | 理想来流 | **FanSim3D 旋转叶轮实际激发** |
| 适用 | 设计迭代/快速全貌 | 教学演示/趋势验证 | **正式结果/交付** | 风机-风洞一体化动态验证 |

### ③ GPU 管线细节

试验段入口 25 m/s 精确匹配, 出口运动水库贯通; 测试物 TYPE_S|TYPE_X 单独标记,
动量交换法输出 `forces.csv` (每 100 步采样, 满足涡脱 Nyquist) → 浏览器面板看
Cd/Cl 时程、后半段均值与 St=f·d/U (Cl 频谱峰)。被测物几何自动测算: 导出时从体素
mask 量出包围盒/**真实迎风投影面积** 与**面积堵塞比** — >10% 提示 Cd 偏大, >25%
提示 LBM 裕度下降。**入口合成湍流**: 流函数法横向涡旋脉动 (y-z 面严格无散, 每分量
6 模式, 波长 20~80 格, rms 归一至 Tu 链设计值 0.46%, 每 100 步更新相位) — 实测
试验段入口 Tu=0.31~0.42% ✓。**逐步时均/湍流度**: 后半段每 100 步累加 Σu/Σu²,
落盘 `vtk/mean_u.vtk` + `vtk/tu.vtk` — explore_gpu 直接读取。2mm 晶格 600 万体素,
每 1500 步一个 VTK 快照共 20 个, 实测求解约 7 分钟 (RX 6650 XT)。**高频切片时序
流**: 每 100 步把 y=0/z=mid 切片 u 三分量流式写入 `fluidx3d/u_slice_y0/zm.raw`
(~310MB) — 供 gpu_slices 出时空图与切片电影。3D VTK 快照相对涡脱周期是时间欠采样
(动画结构跳变是采样问题), 连续动力学证据看 gpu_slices 时空图斜条纹。名义晶格
Re=1200 + SUBGRID LES (Smagorinsky) 补足高 Re 湍流行为。几何/测试物变更后重跑
export 自动把旧一代结果归档到 `fluidx3d\vtk_prev\`。

### ④ 风机耦合管线 (风源=真实叶轮, 2026-09-27 新增)

`export_fluidx3d_fan` 把姊妹项目 **D:\FanSim3D** 的抽风式离心风机 (D2=460mm 前弯多叶,
960rpm) 与风洞拼成**一个 CFD 域** (2026-09-30 侧置定向: 风洞工作方向=x | 抽风口面朝 +y,
外部气体沿 -y 被抽入 | 叶轮绕 y 轴在 xoz 平面旋转 | 蜗壳 xz 阿基米德螺旋收集, 自底部
切向出口沿旋转切线 +x 方向喷入风洞): 大气(+y 域面 E) → 增压箱 → 钟形口 → 吸力眼 →
旋转叶轮(冻结转子动壁, 绕 y 轴) → 蜗壳(切向出口) → 渐扩短管 → 前置扩散段(整流网=2格
棋盘穿孔板) → 稳定段 (蜂窝器/纱网折入背压) → 收缩段 → 试验段 → 自适应背压出口(后扩散段+排空)。

- 流程: `export_fluidx3d_fan` → `build_windtunnel_fan.bat` → `fluidx3d\windtunnel_fan.exe`
  → `explore_gpu` (自动识别耦合模式: x0/涵道壁扩展, 叶轮渲染为红色, 测试物 raw=3)
- 标记: raw=2 叶轮(TYPE_S\|TYPE_X 动壁, 输出 fan_diag.csv: Q/转矩/功率), raw=3 测试物
  (TYPE_S\|TYPE_Y, forces.csv 照常输出 Cd/Cl)
- 档位: WT_QUICK=1 → 6mm/4000 步冒烟; 默认 4mm/15000 步; WT_DX 覆盖
- **使用办法**: ① `export_fluidx3d_fan`(WT_QUICK=1 首跑) ② `build_windtunnel_fan.bat`
  ③ `fluidx3d\windtunnel_fan.exe` ④ `explore_gpu` — 与标准管线互斥使用
  (meta/raw 同名, 切换管线后必须重跑对应 export!)
- 开关: `WT_STEPS=步数` 覆盖 (系统充压 ~0.3s, 步数不足无真工况点) |
  `WT_SPIN=-1` 整机 z 镜像反转 (螺旋/出口/旋向同步配对) | `WT_DX=0.002` 定量档
- 物理预期: 叶轮激发实际风场 — 蜗壳旋流、叶轮尾迹、整流网/收缩段的旋流抑制全部真实求解,
  试验段来流品质(旋流残留/湍流度)可直接从 tu.vtk 读出 — 这是 ①②③ 给不了的

 (1D 设计 1.46) — 叶道仅 ~6.7 格的分辨率保真度
  极限; 定性旋流/可视化有效, 定量以 FanSim3D 的 1D 模型为准、2mm 档复核

## 四、所有可调参数 (`tunnel_params.m`)

### 构型预设 (一键换整体方案)
```matlab
P = tunnel_params('balanced')   % 均衡 (默认): R=6, L=600mm, 矩形试验段
P = tunnel_params('lowTurb')    % 低湍流: R=8, L=420mm (五次型线)
P = tunnel_params('compact')    % 紧凑: R=5, L=220mm (三次型线)
P = tunnel_params('aniso')      % 矩形试验段 200×120mm
```
命令行切换: `setenv('WT_CONFIG','lowTurb')` 后跑任意脚本。

### 常用单项参数 (改数字即生效)
| 参数 | 含义 | 默认 |
|---|---|---|
| `P.V_test` | 试验段设计风速 (m/s) | 25 |
| `P.ct.R` / `P.ct.L` | 收缩比 / 收缩段长 (m) | 6 / 0.60 (入口 735×490) |
| `P.ct.curve` | 收缩型线: quintic/cubic/sine/linear/twostage | quintic |
| `P.ts.W/H/L` | 试验段宽/高/长 (m) | 0.30/0.20/0.20 |
| `P.sc.L` | 稳定段长度 (m) | 0.45 |
| `P.fd.W_in/H_in/L` | **前置扩散段** 风机法兰宽/高/长 (m) — 对接 FanSim3D 风机出口 | 0.30/0.27/0.50 |
| `P.df.W_out/H_out/L` | 扩散段出口宽/高/长 (m) | 0.40/0.26/0.75 (面积比 1.73, 2θ≈5.6°) |
| `P.wall.ks` | PLA 壁面粗糙度 (m) | 0.0002 (打磨后可 0.00005) |
| `P.flow.dx_cell` | 稳态网格目标单元尺寸 (m), 网格数由此自动派生 | 0.003 |
| `P.lbm.object` | 测试物: 'stl'(你的模型)/sphere/cylinder/plate/naca | 'stl' |
| `P.lbm.objFile` | STL 路径 | 'test_object.stl' |

### 环境变量开关 (MATLAB 里 setenv, 不改代码)
| 变量 | 作用 |
|---|---|
| `WT_QUICK=1` | 快速低分辨率档 (冒烟验证) |
| `WT_NOGIF=1` | 跳过 GIF 生成 (分析图秒出) |
| `WT_RESOLVE=1` | 强制重新求解 (忽略已有存档) |
| `WT_CONFIG=<名>` | 切换构型预设 |
| `WT_T=<时间步>` | explore_gpu 直接打开指定时刻快照 |
| `WT_FIELD=<场>` | explore_gpu 初始场 (spd/vort/q/rho/mean/turb, 无头快照导出用) |
| `WT_OBJ=<类型>` | 覆盖测试物 (export_fluidx3d / lbm_run: sphere/cylinder/plate/naca) |
| `WT_NOOBJ=1` | 空风洞基线模式: 被测物几何仍测算但不写入流场 (→ mean_u_base/tu_base, gpu_report 自动对照) |
| `WT_DX=<m>` | GPU 晶格覆盖 (如 0.001=1mm 档, 格数/耗时按 dx 三~四次方涨) |
| `WT_MOV=[N]` | explore_gpu 导出时序动画 gpu_timelapse.mp4 (=1 全部快照, =N 每 N 个取 1) |
| `WT_CP=1` | 无头快照/动画中强制开启表面载荷着色 |
| `WT_MID=1` | 稳态中档 (单元 ×2, 求解 ~25 s 快速查看) |

### 稳态精度档位 (flow3d_solve, 网格随几何自动派生)
| 档位 | 触发 | 体素 (300×200 试验段) | 单元尺寸 |
|---|---|---|---|
| 快速 | WT_QUICK=1 | 111×43×32 | ~18 mm |
| 中档 | WT_MID=1 | 334×130×87 (380 万) | ≈6 mm |
| **默认 (最高)** | 直接运行 | **667×260×173 (3000 万)** | **≈3.0 mm** (y/z 独立适配矩形截面) |
网格由 `P.flow.dx_cell` (默认 3mm) 按全长/入口宽/入口高自动派生 — 改试验段尺寸后档位自动跟随, 不再失配。
存档 flow3d_result.mat 大场字段为单精度, 内存需求 ~3.5 GB。

> GPU 说明: `flow3d_solve` 的批量泊松投影仅在检测到 **NVIDIA CUDA** 设备时用 gpuArray 加速;
> AMD/Intel 显卡无 CUDA (MATLAB gpuArray 不适用, 装了 PCT 也一样), 稳态路线始终 CPU 运行 —
> 本机 (RX 6650 XT) 即此情况, GPU 加速一律走 FluidX3D (OpenCL) 管线, 控制台会明确提示原因。

---

## 五、交互浏览器 (explore_3d / explore_gpu) — 双窗口布局

- **主视图 (3D)**: 涵道内壁 + 体填充 + 测试物/边界层, 截面位置只画**青/品红虚线轮廓标尺**,
  不遮挡模型; 拖滑块时标尺与游标实时移动
- **GPU 主视图渲染**: 按场类型自适应等值面 — 速度/时均场画 0.90V 尾流/边界层壳
  (均匀核心完全透明, 一眼看穿洞内), 涡量/**Q 准则**/湍流度画尾流结构嵌套壳;
  + 薄雾层补体感 + 头灯光照
- **Q 准则涡结构** (仅 explore_gpu): Q = ½(‖Ω‖²−‖S‖²) 逐快照由速度场解算 —
  涡旋转强于剪切变形的连通结构, 涡环/展向涡比涡量幅值干净一个量级
- **表面载荷着色** (仅 explore_gpu): 快照同步落盘 `F_*.vtk` (体素级气动力矢量),
  测试物表面按 |F|/q∞ turbo 着色 (勾选"表面载荷") — 相当于数值压力敏感漆,
  迎风面高压/肩部峰值/尾流低压一眼可辨
- **时序动画导出**: MATLAB 里 `setenv('WT_MOV','1')` 后跑 explore_gpu →
  gpu_timelapse.mp4 (涡脱演化/尾流发展全程回放)
- **截面分析窗 (独立窗口, 自动弹出)**: 竖直截面 (x×z) 与水平截面 (x×y) 两幅场图,
  带涵道壁黑线与测试物蓝线轮廓, 拖动滑块实时刷新
- **鼠标左键拖拽**: 旋转主视图; 滚轮: 缩放
- **竖直截面 y 滑块**: 流向×高度纵剖; **水平截面 z 滑块**: 俯视风场
- **x 裁剪滑块**: 隐藏下游段, "看进洞内"
- **显示场切换**: 流速 / 涡量 / 涡粘比 (稳态) 或 流速/涡量/密度/**时均流速/湍流度 Tu%** (GPU)
- **时均/湍流度** (仅 explore_gpu): 优先直读 GPU 逐步统计 `mean_u.vtk`/`tu.vtk`
  (求解器每 100 步累加, 无欠采样, 秒开); 旧存档自动回退后半段快照平均 (Tu 为下界)
  — 时均场 |Ū| 与湍流度 Tu(%), 真实风洞测的就是这两个量
- **气动力面板** (仅 explore_gpu): forces.csv → Cd/Cl 时程曲线 + 后半段均值 +
  Cl 频谱峰涡脱频率与 St 数标注 (动量交换法, 标注参考面积与名义晶格 Re)
- **时间快照滑块** (仅 explore_gpu): 拖动回放 GPU 瞬态演化
- **显隐复选框**: 体填充 / 等值面 / 涵道壁 / PLA 边界层 / 截面标尺 / 截面窗子图
- explore_3d / explore_gpu 只关闭自己的旧窗口, 不会动 visualize_3d 的分析图

---

## 六、完整工作流示例

### 场景 1: 设计一款新风洞
```matlab
cd D:\WindTunnel3D
build_tunnel_model                 % 看侧视图, 检查壁面角 <35°/<7°
contraction_study                  % 选收缩段 L 和 R
boundary_layer_study               % 定试验段长度 (分层 vs 核心加速)
% 满意后改 tunnel_params.m 里的默认值
```

### 场景 2: 稳态流场检查
```matlab
visualize_3d                       % 4 张分析图秒出 (GIF 最后, 可 WT_NOGIF=1 跳过)
explore_3d                         % 切片细看任意截面
```

### 场景 3: 用你的模型测气动力 (GPU, 正式流程)
1. CAD 里把模型导出为二进制 STL → 存为 `D:\WindTunnel3D\test_object.stl`
   (无 STL 时可先用参数化测试物: MATLAB 里 `setenv('WT_OBJ','cylinder')` 后跑 export;
   模型最大可到试验段 300×200×200mm, 导出时自动打印包围盒/迎风面积/堵塞比诊断)
2. (可选, 推荐) 空风洞基线: MATLAB `setenv('WT_NOOBJ','1')` → export → bat → exe,
   完成后 `setenv('WT_NOOBJ','')` — gpu_report 会自动加载基线做对照并给堵塞修正 Cd
3. MATLAB: `export_fluidx3d` (生成网格与源码; 默认 2mm 全档, `WT_QUICK=1` 冒烟)
4. 资源管理器双击 `build_windtunnel.bat` (编译, 换模型后需重跑 3-4 步)
5. 双击 `fluidx3d\windtunnel.exe` (GPU 求解, 完成提示 "wind tunnel done")
6. MATLAB: `explore_gpu` — 切片看绕流, Q 准则看涡结构, 勾选"表面载荷"看模型表面
   气动载荷分布, 面板读 Cd/Cl/St
7. MATLAB: `gpu_mov` — 一键延时动画 `gpu_timelapse.mp4` (默认 Q 准则涡脱演化;
   `gpu_mov('spd')` 换场, `gpu_mov('turb',2)` 加速渲染; mp4 失败自动回退 GIF)
   + 切片分析窗同步留档 `gpu_timelapse_slices.mp4`
8. MATLAB: `gpu_slices` — 连续性档案: 时空图 `gpu_xt_y0.png` (斜条纹=涡连续输运)
   + 高频切片电影 `gpu_slice_movie_y0.mp4` (|u| 与涡量 ω_y 梯度双面板)
   + `gpu_slice_movie_zm.mp4` (水平切片) + 控制台相邻帧相关系数 (连续性定量检验)
9. MATLAB: `gpu_report` — 一页双重测算报告 `gpu_report.png`
   (流场质量/边界层发展/气动力/涡脱频谱/基线对照/堵塞修正汇总)

### 场景 4: 与你的 STEP 装配模型核对
`tunnel_body.stl` 流向为 STL +Z 轴, 任意 CAD (SolidWorks/Fusion/FreeCAD)
可直接导入并与 STEP 模型装配比对。

### 场景 5: MATLAB 动态风场 (快速演示)
`lbm_run` — 自动完成瞬态求解并输出:
- 图1: 测试物 Cd/Cl 时程 + 客观性文献基准线 (自动偏差诊断)
- 图2: 水平/竖直涡量切面 (对称涡街可见)
- lbm_p0_slices.png: **总压切片时序蒙太奇** (jet 彩虹渲染, 视频同款形式)
- lbm_fig3.png: 3D 涵道内壁半透明壳 + 测试物 + 涡量等值面
- 控制台: 客观性验证块 (Cd/St 偏差、分辨率与堵塞比诊断)

---

## 七、文件清单

| 文件 | 说明 |
|---|---|
| `tunnel_params.m` | ★ 全部参数与构型预设 (一切改动从这里开始) |
| `smoke_check.m` | 改参数后的几何/网格/域覆盖快速体检 (秒级, 不求解) |
| `tunnel_geometry.m` | 沿程几何/速度/壁面角 (a/b 双型线 + 5 种收缩型线库) |
| `build_tunnel_model.m` | 建模 + STL 导出 + 设计检查图 |
| `contraction_study.m` / `boundary_layer_study.m` | 设计研究脚本 |
| `flow3d_solve.m` | 稳态体素求解器 |
| `normalShock.m` | 正激波关系模块 (高速工况) |
| `visualize_3d.m` | 稳态求解 + 4 张分析图 + GIF |
| `explore_3d.m` | 稳态交互浏览器 (参数可就地改并重解) |
| `lbm3d_solve.m` / `lbm_run.m` | MATLAB 动态 LBM 求解器与驱动 |
| `tunnel_voxelize.m` | 动态域体素化 (涵道壁 + 测试物 STL) |
| `export_fluidx3d.m` | ★ MATLAB → FluidX3D GPU 衔接器 |
| `read_fluidx3d_vtk.m` | FluidX3D VTK 结果读回 MATLAB |
| `explore_gpu.m` | GPU 结果交互浏览器 (时间回放 + 气动力) |
| `gpu_mov.m` | 一键延时动画导出 (gpu_timelapse.mp4, 重运算标准后处理) |
| `gpu_slices.m` | 高频切片时序留档: 时空图 + 切片电影 + 连续性定量检验 |
| `gpu_report.m` | 风洞+被测物 双重测算一页仪表盘 (gpu_report.png) |
| `build_windtunnel.bat` / `fluidx3d\windtunnel.exe` | GPU 求解编译/执行 |
| `选型与安装清单.md` | 软件选型与安装记录 (FluidX3D 已装) |

结果存档: `flow3d_result.mat` / `lbm_result.mat` / `gpu_meta.mat` +
`fluidx3d\vtk\`(快照流场 u_/rho_/F_ + `mean_u.vtk` 时均场 + `tu.vtk` 湍流度%
+ 空风洞基线 `mean_u_base.vtk`/`tu_base.vtk`) + `forces.csv`(气动力时程) +
`fluidx3d\u_slice_y0/zm.raw`(高频切片时序流) + 视频档案
`gpu_timelapse.mp4`(3D) / `gpu_timelapse_slices.mp4`(切片窗) /
`gpu_slice_movie_y0.mp4` / `gpu_slice_movie_zm.mp4`(高频切片电影) +
`gpu_xt_y0.png`(时空图) + `gpu_report.png`(双重测算) +
`fluidx3d\vtk_prev\`(上一代归档)。

> **2026-09 修复**: `read_fluidx3d_vtk` 原按 SoA 解包矢量 VTK, 而 FluidX3D
> 实际写 AoS (每格点 x,y,z 相连) — 修复前所有矢量场 (流速/涡量) 均带 3 周期
> 分量交错的细微伪影, 修复后分量均值/剖面与理论一致。

### 逆压梯度与边界层分离 (两款求解器均覆盖)
| 求解器 | 方法 | 输出 |
|---|---|---|
| 稳态 `flow3d_solve` | 一维湍流边界层判据: R = (dp/dx·δ*)/τw (壁面摩擦对抗逆压的能力比, R→1 分离) | 逐站 R(x) + 自动判定 + R>0.9 站位自动嵌入近壁反向带 (流场中直接可见分离趋势) |
| `boundary_layer_study` | 同源 R(x) 曲线 + **速度剖面耙图** (扩散段轮廓+各站剖面, 参考图形式) + 陡扩散对比 (角度翻倍→R=1.68 必分离) | 分离风险曲线图 |
| 动态 `lbm3d_solve` | **真实求解**: 贴壁 2 格内逆流 (u_x<0) 体素占比逐站检测 | revFrac(x) + 自动判定 (当前附着流 0%) |

当前设计 (试验段 300×200 → 扩散段出口 400×260, L=0.75 m, 面积比 1.73, 全张角 2θ≈5.6°):
稳态求解判 "逆压显著但余量充足" — 扩散段逆压比 R_max=0.62 (默认 667×260×173 档实测,
R≥1 才分离, 余量 38%), 无近壁反向带嵌入。对比旧 150×150 设计 (面积比 4, 2θ≈12.2°,
R_max≈2.0 已入分离风险区): 大试验段使扩散段张角大幅收敛, 全程附着。
经验判据 2θ≤7° 留有余量; `boundary_layer_study` 的无粘一维口径更保守, 可复查陡扩散对比。

### 壁面摩擦对风速与边界层的影响 (大试验段 300×200 实测)

新设计下大截面使边界层相对更薄 (δ/半宽比旧 150 方形段小一倍), 摩擦增速与
核心均匀性同步改善; PLA 粗糙壁 (ks=0.2mm) 仍使边界层全程湍流 (cf 较光滑壁 +243%)。
逐站数值跑 `visualize_3d` / `explore_3d` 看控制台诊断与切片 (贴壁低速带直接可见),
设计研究用 `boundary_layer_study`。

物理链条: 收缩段强顺压把边界层压在 2mm 内 → 进入试验段后重新生长 → 排挤效应
使核心风速比无粘值略高 → 扩散段逆压使 δ 增厚 (当前张角 2θ≈5.6°, 增厚温和)。
试验段中部分层度 (核心速度不均匀度) <1% — 大截面的直接红利。

---

## 八、动态解算的客观性验证 (每次 lbm_run 自动输出)

lbm_run 结束时会打印"客观性验证"块，把 LBM 结果与文献基准对表：

| 指标 | LBM (快速档) | 文献基准 | 说明 |
|---|---|---|---|
| 圆球 Cd (演示 Re=250) | ~1.5 | 0.70 (Clift-Grace-Weber 相关式) | 偏差 +116%：3.8 格直径的体素阶梯 + 低分辨率 |
| 圆柱 Cd (演示 Re=250) | ~2.3 | 0.70 | 同上，横向圆柱 |
| 圆柱 St (涡脱) | 0.06~0.14 | 0.212 (Williamson) | 方向正确、量级可信；受尾流发展时间与分辨率限制 |
| 出口风速 | 25.0 m/s | 设计值 25 | 精确命中 (通量匹配) |
| 质量守恒 | 各站通量偏差 <3% | 0 | 连续性自检 |

**分辨率与堵塞比诊断**自动打印：直径晶格数 (<10 警告)、面积堵塞比。
**客观性结论**：演示档 Cd 半定量 (趋势与量级正确、数值偏高 1~2 倍)、St 定性正确、
速度场定量可信 (风速/通量精确)。正式定量请走 GPU 2mm 档 + 光滑 STL +
更小测试物 (直径≥15 格)，或转 OpenFOAM。
**文献来源**：Cd — Clift, Grace & Weber (1978)；St — Williamson (1997)。

### GPU 管线实测校验 (圆柱 d=30mm, 15 格直径, 名义晶格 Re=1200 + LES)

| 量 | GPU 实测 (forces.csv 频谱/均值) | 文献 (自由圆柱, Re~10^3) | 评注 |
|---|---|---|---|
| Cl_rms (升力脉动) | 0.35 | 0.3~0.5 | ✓ 量级吻合, 涡脱强而相干 |
| 涡脱 St | 0.41 | 0.21 | 偏高 ~2×: 15% 面积堵塞 + 圆柱贯通壁面 + 体素阶梯, 受限工况 St 上移 |
| 时均尾流 Tu | 尾流核心 60~78% | 同量级 | tu.vtk 逐步统计, 无欠采样 |
| 入口/外流 Tu | 入口 0.66%, 试验段 0.31~0.42% | 设计值 0.46% | 与 Tu 链 (蜂窝器/纱网/收缩衰减) 闭环 ✓ |

面板中 St 由 Cl 后半段 FFT 峰自动标注 (f·d/U); 升力取 Fz (竖直向)。
Cd 在受限+体素阶梯下偏高 (墙面贯通圆柱), 定性趋势与相对比较可用, 绝对值谨慎。

## 九、已知边界 (诚实声明)

- 稳态管线的边界层为子网格模型 (体素是单元平均值); 解析边界层结构需
  `P.flow.dx_cell` 降到 2mm 以下 (格数按 3 次方涨, 注意内存)
- MATLAB 动态 LBM 受晶体稳定上限约束, 演示 Re≤~900; Cd 为半定量
- GPU 管线名义晶格 Re=1200 (以试验段宽度计), SUBGRID LES 补足高 Re 湍流行为;
  2mm 晶格下体素球 Cd 偏高 (阶梯效应), 光滑 STL 模型 + 1mm 档可继续收敛
- 入口合成湍流为流函数法 2D 横向脉动 (y-z 面无散, 无流向分量 — 理想化的 2C 网格湍流),
  相位每 100 步才更新一次; 沿程衰减使试验段入口 Tu≈0.3~0.4% (设计 0.46%)
- Tu 湍流度链为一维经验式 (风扇/蜂窝器/纱网/收缩段衰减), 给数量级与设计趋势,
  非逐站 CFD 结果; GPU 的 tu.vtk 则是脉动直接解算的统计真值 (受晶格分辨率限制)
- 激波模块为 1D 关系式+逐体素马赫标记 (当前 Ma≈0.07 无激波, 教学用途)
- 验证环境: MATLAB R2026a (基础版, 无工具箱) + Windows 11 + RX 6650 XT

﻿# WindTunnel3D — 桌面级风洞仿真项目 使用手册

目标: 试验段 300×200×200 mm, 稳定风速 20–30 m/s (设计点 25 m/s)。
全流程以 MATLAB 为核心; 动态风场可选 GPU 加速 (FluidX3D, 已装好)。

> **2026-09-27 更新**: 风洞起点新增**前置扩散段** (风机出口法兰 300×270 → 稳定段入口
> 735×490, L=500mm, 面积比 4.4:1, 出口整流网) — 对接姊妹项目 **FanSim3D** (现为套件
> `../FanSim3D`) 的抽风式离心风机 (D2=460mm 前弯多叶, 960rpm, 1D 工况点 24.3 m/s)。
> 风洞全长 2.0 → 2.5 m, 该段折入稳定段几何 (xSec 索引不变), 前置扩散段的分离/整流用
> `visualize_3d` 的 R 判据与 FanSim3D 的 GPU 求解复核。

> **2026-09-30 更新 (风机定向修复)**: 风机耦合布局重排 — 旧版风机与风洞同轴内联 (叶轮绕
> x 转, 进气沿 -x), 本质问题: 离心风机应为侧置 (吸入口轴向进气 + 蜗壳切向排出)。新布局:
> **抽风口面朝 +y** (大气盖移到 +y 域面, 气体沿 -y 抽入) | **叶轮绕 y 轴旋转 (xoz 平面)**
> | **蜗壳 xz 螺旋, 底部切向出口沿 +x** (旋转切线方向) 喷入前置扩散段。旋向配对: 螺旋逆
> 时针生长 ↔ ω<0 (绕 +y), `WT_SPIN=-1` 整机 z 镜像。叶轮截面 (D2=460 前弯多叶) 与 1D
> 性能模型原样复用 FanSim3D; 耦合域 738×227×296 = 49.6M 格 (dx=4mm); 后处理 meta 新增
> y0/z0 域原点 (风机耦合域 y/z 不再对称), explore_gpu/gpu_report/whole_duct_mov/probe_adj
> 已适配。旧朝向的 43GB VTK 快照移入 `fluidx3d/vtk_old_inline/` 存档。
>
> **[P0] 快照镜像修复 + 套件化 (2026-09-30)**: FluidX3D 的 `update_fields()` 在本配置下
> 重构的 rho/u 镜像损坏 (实测 rho 0.61~2.02、u 流体区低 ~8 倍) — 修复前所有瞬时 u/rho
> 快照均不可信 (mean_u.vtk 时均与 fan_diag.csv 诊断不受影响)。三套生成器已在快照前移除
> 该调用; 新增 `WT_VTK` 档位 (默认只写 u 场省 ~2/3 磁盘, `WT_VTK=all` 附加 rho+F)。
> 项目与 FanSim3D 并列迁入套件 (根见 `../README.md`), 跨项目路径全部改为 `mfilename`
> 相对定位 + `%~dp0` bat, 整个套件文件夹可任意搬移。

---

## 一、最快上手 (30 秒)

```matlab
cd D:\WindTunnel3D
visualize_3d      % 稳态仿真 + 自动出 5 张图 (关掉图窗即结束)
explore_3d        % 交互浏览器: 拖滑块从任意方向切片看流场
```

想看**动态风场** (测试物绕流、涡脱落、气动力), 三选一:

```matlab
lbm_run           % MATLAB 版: 1~15 分钟, 演示级 Re (教学/快速验证)

% GPU 版 (真实 Re, 推荐): 三步
export_fluidx3d   % ① 生成体素网格 + C++ 源码 (MATLAB 里跑)
                  % ② 双击 build_windtunnel.bat 编译 (换参数后需重跑①②)
                  % ③ 双击 fluidx3d\windtunnel.exe GPU 求解 (~6 分钟, 20 快照 + 力时程)
explore_gpu       % ④ 回 MATLAB 交互浏览 GPU 结果 + 气动力面板
```

放入你自己的测试物: 把 CAD 模型导出为**二进制 STL**, 存为
`D:\WindTunnel3D\test_object.stl` — 之后所有求解器自动使用它
(包围盒中心自动对齐试验段中心; 无此文件时回退演示球)。

---

## 二、六条命令速查 (在 MATLAB 命令行输入名字即执行)

| 命令 | 干什么 | 什么时候用 |
|---|---|---|
| `build_tunnel_model` | 生成风洞 3D 模型, 导出 `tunnel_body.stl` + 侧视/壁面角检查图 | 改了尺寸后; 或要把模型导入 CAD 与你的 STEP 装配 |
| `contraction_study` | 收缩段长度×收缩比参数扫描, 给出推荐设计点 | 设计选型阶段 |
| `boundary_layer_study` | 腔体长度→边界层/核心加速扫描 + PLA 粗糙度对比 | 评估摩擦分层、定试验段长度 |
| `visualize_3d` | 稳态流场求解 + 自动出 4 张分析图 + 体渲染主图 + GIF | 快速看稳态全貌 (默认档约 2 分钟) |
| `explore_3d` | 稳态结果交互浏览器 (不重算) | 细看稳态场, 拖滑块任意切片 |
| `lbm_run` | MATLAB 版动态仿真 (瞬态 LBM) | 看涡脱落/气动力时程 + 涵道内壁3D渲染, 快速验证 |

前三条**不求解流场** (设计与建模); 后三条各属一条求解管线 (见下表)。

---

## 三、三条求解管线 (按精度/速度选)

| | ① 稳态管线 | ② MATLAB 动态 | ③ GPU 动态 (FluidX3D) ★ | ④ 风机耦合 GPU ★★ |
|---|---|---|---|---|
| 求解器 | `flow3d_solve` (定常重建+边界层模型) | `lbm3d_solve` (D3Q19 TRT) | FluidX3D 3.7 (OpenCL, 已编译) | FluidX3D + 旋转叶轮动壁 |
| 入口命令 | `visualize_3d` | `lbm_run` | `export_fluidx3d` → bat → exe | `export_fluidx3d_fan` → `build_windtunnel_fan.bat` → `windtunnel_fan.exe` |
| 结果查看 | `explore_3d` | 自带 3 张图 | `explore_gpu` (同款交互) | `explore_gpu`/`gpu_mov`/`gpu_report` 自动兼容 |
| 风源 | 理想来流 | 理想来流 | 理想来流 | **FanSim3D 旋转叶轮实际激发** |
| 适用 | 设计迭代/快速全貌 | 教学演示/趋势验证 | **正式结果/交付** | 风机-风洞一体化动态验证 |

### ③ GPU 管线细节

试验段入口 25 m/s 精确匹配, 出口运动水库贯通; 测试物 TYPE_S|TYPE_X 单独标记,
动量交换法输出 `forces.csv` (每 100 步采样, 满足涡脱 Nyquist) → 浏览器面板看
Cd/Cl 时程、后半段均值与 St=f·d/U (Cl 频谱峰)。被测物几何自动测算: 导出时从体素
mask 量出包围盒/**真实迎风投影面积** 与**面积堵塞比** — >10% 提示 Cd 偏大, >25%
提示 LBM 裕度下降。**入口合成湍流**: 流函数法横向涡旋脉动 (y-z 面严格无散, 每分量
6 模式, 波长 20~80 格, rms 归一至 Tu 链设计值 0.46%, 每 100 步更新相位) — 实测
试验段入口 Tu=0.31~0.42% ✓。**逐步时均/湍流度**: 后半段每 100 步累加 Σu/Σu²,
落盘 `vtk/mean_u.vtk` + `vtk/tu.vtk` — explore_gpu 直接读取。2mm 晶格 600 万体素,
每 1500 步一个 VTK 快照共 20 个, 实测求解约 7 分钟 (RX 6650 XT)。**高频切片时序
流**: 每 100 步把 y=0/z=mid 切片 u 三分量流式写入 `fluidx3d/u_slice_y0/zm.raw`
(~310MB) — 供 gpu_slices 出时空图与切片电影。3D VTK 快照相对涡脱周期是时间欠采样
(动画结构跳变是采样问题), 连续动力学证据看 gpu_slices 时空图斜条纹。名义晶格
Re=1200 + SUBGRID LES (Smagorinsky) 补足高 Re 湍流行为。几何/测试物变更后重跑
export 自动把旧一代结果归档到 `fluidx3d\vtk_prev\`。

### ④ 风机耦合管线 (风源=真实叶轮, 2026-09-27 新增)

`export_fluidx3d_fan` 把姊妹项目 **D:\FanSim3D** 的抽风式离心风机 (D2=460mm 前弯多叶,
960rpm) 与风洞拼成**一个 CFD 域** (2026-09-30 侧置定向: 风洞工作方向=x | 抽风口面朝 +y,
外部气体沿 -y 被抽入 | 叶轮绕 y 轴在 xoz 平面旋转 | 蜗壳 xz 阿基米德螺旋收集, 自底部
切向出口沿旋转切线 +x 方向喷入风洞): 大气(+y 域面 E) → 增压箱 → 钟形口 → 吸力眼 →
旋转叶轮(冻结转子动壁, 绕 y 轴) → 蜗壳(切向出口) → 渐扩短管 → 前置扩散段(整流网=2格
棋盘穿孔板) → 稳定段 (蜂窝器/纱网折入背压) → 收缩段 → 试验段 → 自适应背压出口(后扩散段+排空)。

- 流程: `export_fluidx3d_fan` → `build_windtunnel_fan.bat` → `fluidx3d\windtunnel_fan.exe`
  → `explore_gpu` (自动识别耦合模式: x0/涵道壁扩展, 叶轮渲染为红色, 测试物 raw=3)
- 标记: raw=2 叶轮(TYPE_S\|TYPE_X 动壁, 输出 fan_diag.csv: Q/转矩/功率), raw=3 测试物
  (TYPE_S\|TYPE_Y, forces.csv 照常输出 Cd/Cl)
- 档位: WT_QUICK=1 → 6mm/4000 步冒烟; 默认 4mm/15000 步; WT_DX 覆盖
- **使用办法**: ① `export_fluidx3d_fan`(WT_QUICK=1 首跑) ② `build_windtunnel_fan.bat`
  ③ `fluidx3d\windtunnel_fan.exe` ④ `explore_gpu` — 与标准管线互斥使用
  (meta/raw 同名, 切换管线后必须重跑对应 export!)
- 开关: `WT_STEPS=步数` 覆盖 (系统充压 ~0.3s, 步数不足无真工况点) |
  `WT_SPIN=-1` 整机 z 镜像反转 (螺旋/出口/旋向同步配对) | `WT_DX=0.002` 定量档
- 物理预期: 叶轮激发实际风场 — 蜗壳旋流、叶轮尾迹、整流网/收缩段的旋流抑制全部真实求解,
  试验段来流品质(旋流残留/湍流度)可直接从 tu.vtk 读出 — 这是 ①②③ 给不了的
- 诚实状态: 4mm 下泵送流量 Q≈0.005 m³/s (1D 设计 1.46) — 叶道仅 ~6.7 格的分辨率保真度
  极限; 定性旋流/可视化有效, 定量以 FanSim3D 的 1D 模型为准、2mm 档复核

## 四、所有可调参数 (`tunnel_params.m`)

### 构型预设 (一键换整体方案)
```matlab
P = tunnel_params('balanced')   % 均衡 (默认): R=6, L=600mm, 矩形试验段
P = tunnel_params('lowTurb')    % 低湍流: R=8, L=420mm (五次型线)
P = tunnel_params('compact')    % 紧凑: R=5, L=220mm (三次型线)
P = tunnel_params('aniso')      % 矩形试验段 200×120mm
```
命令行切换: `setenv('WT_CONFIG','lowTurb')` 后跑任意脚本。

### 常用单项参数 (改数字即生效)
| 参数 | 含义 | 默认 |
|---|---|---|
| `P.V_test` | 试验段设计风速 (m/s) | 25 |
| `P.ct.R` / `P.ct.L` | 收缩比 / 收缩段长 (m) | 6 / 0.60 (入口 735×490) |
| `P.ct.curve` | 收缩型线: quintic/cubic/sine/linear/twostage | quintic |
| `P.ts.W/H/L` | 试验段宽/高/长 (m) | 0.30/0.20/0.20 |
| `P.sc.L` | 稳定段长度 (m) | 0.45 |
| `P.fd.W_in/H_in/L` | **前置扩散段** 风机法兰宽/高/长 (m) — 对接 FanSim3D 风机出口 | 0.30/0.27/0.50 |
| `P.df.W_out/H_out/L` | 扩散段出口宽/高/长 (m) | 0.40/0.26/0.75 (面积比 1.73, 2θ≈5.6°) |
| `P.wall.ks` | PLA 壁面粗糙度 (m) | 0.0002 (打磨后可 0.00005) |
| `P.flow.dx_cell` | 稳态网格目标单元尺寸 (m), 网格数由此自动派生 | 0.003 |
| `P.lbm.object` | 测试物: 'stl'(你的模型)/sphere/cylinder/plate/naca | 'stl' |
| `P.lbm.objFile` | STL 路径 | 'test_object.stl' |

### 环境变量开关 (MATLAB 里 setenv, 不改代码)
| 变量 | 作用 |
|---|---|
| `WT_QUICK=1` | 快速低分辨率档 (冒烟验证) |
| `WT_NOGIF=1` | 跳过 GIF 生成 (分析图秒出) |
| `WT_RESOLVE=1` | 强制重新求解 (忽略已有存档) |
| `WT_CONFIG=<名>` | 切换构型预设 |
| `WT_T=<时间步>` | explore_gpu 直接打开指定时刻快照 |
| `WT_FIELD=<场>` | explore_gpu 初始场 (spd/vort/q/rho/mean/turb, 无头快照导出用) |
| `WT_OBJ=<类型>` | 覆盖测试物 (export_fluidx3d / lbm_run: sphere/cylinder/plate/naca) |
| `WT_NOOBJ=1` | 空风洞基线模式: 被测物几何仍测算但不写入流场 (→ mean_u_base/tu_base, gpu_report 自动对照) |
| `WT_DX=<m>` | GPU 晶格覆盖 (如 0.001=1mm 档, 格数/耗时按 dx 三~四次方涨) |
| `WT_MOV=[N]` | explore_gpu 导出时序动画 gpu_timelapse.mp4 (=1 全部快照, =N 每 N 个取 1) |
| `WT_CP=1` | 无头快照/动画中强制开启表面载荷着色 |
| `WT_MID=1` | 稳态中档 (单元 ×2, 求解 ~25 s 快速查看) |

### 稳态精度档位 (flow3d_solve, 网格随几何自动派生)
| 档位 | 触发 | 体素 (300×200 试验段) | 单元尺寸 |
|---|---|---|---|
| 快速 | WT_QUICK=1 | 111×43×32 | ~18 mm |
| 中档 | WT_MID=1 | 334×130×87 (380 万) | ≈6 mm |
| **默认 (最高)** | 直接运行 | **667×260×173 (3000 万)** | **≈3.0 mm** (y/z 独立适配矩形截面) |
网格由 `P.flow.dx_cell` (默认 3mm) 按全长/入口宽/入口高自动派生 — 改试验段尺寸后档位自动跟随, 不再失配。
存档 flow3d_result.mat 大场字段为单精度, 内存需求 ~3.5 GB。

> GPU 说明: `flow3d_solve` 的批量泊松投影仅在检测到 **NVIDIA CUDA** 设备时用 gpuArray 加速;
> AMD/Intel 显卡无 CUDA (MATLAB gpuArray 不适用, 装了 PCT 也一样), 稳态路线始终 CPU 运行 —
> 本机 (RX 6650 XT) 即此情况, GPU 加速一律走 FluidX3D (OpenCL) 管线, 控制台会明确提示原因。

---

## 五、交互浏览器 (explore_3d / explore_gpu) — 双窗口布局

- **主视图 (3D)**: 涵道内壁 + 体填充 + 测试物/边界层, 截面位置只画**青/品红虚线轮廓标尺**,
  不遮挡模型; 拖滑块时标尺与游标实时移动
- **GPU 主视图渲染**: 按场类型自适应等值面 — 速度/时均场画 0.90V 尾流/边界层壳
  (均匀核心完全透明, 一眼看穿洞内), 涡量/**Q 准则**/湍流度画尾流结构嵌套壳;
  + 薄雾层补体感 + 头灯光照
- **Q 准则涡结构** (仅 explore_gpu): Q = ½(‖Ω‖²−‖S‖²) 逐快照由速度场解算 —
  涡旋转强于剪切变形的连通结构, 涡环/展向涡比涡量幅值干净一个量级
- **表面载荷着色** (仅 explore_gpu): 快照同步落盘 `F_*.vtk` (体素级气动力矢量),
  测试物表面按 |F|/q∞ turbo 着色 (勾选"表面载荷") — 相当于数值压力敏感漆,
  迎风面高压/肩部峰值/尾流低压一眼可辨
- **时序动画导出**: MATLAB 里 `setenv('WT_MOV','1')` 后跑 explore_gpu →
  gpu_timelapse.mp4 (涡脱演化/尾流发展全程回放)
- **截面分析窗 (独立窗口, 自动弹出)**: 竖直截面 (x×z) 与水平截面 (x×y) 两幅场图,
  带涵道壁黑线与测试物蓝线轮廓, 拖动滑块实时刷新
- **鼠标左键拖拽**: 旋转主视图; 滚轮: 缩放
- **竖直截面 y 滑块**: 流向×高度纵剖; **水平截面 z 滑块**: 俯视风场
- **x 裁剪滑块**: 隐藏下游段, "看进洞内"
- **显示场切换**: 流速 / 涡量 / 涡粘比 (稳态) 或 流速/涡量/密度/**时均流速/湍流度 Tu%** (GPU)
- **时均/湍流度** (仅 explore_gpu): 优先直读 GPU 逐步统计 `mean_u.vtk`/`tu.vtk`
  (求解器每 100 步累加, 无欠采样, 秒开); 旧存档自动回退后半段快照平均 (Tu 为下界)
  — 时均场 |Ū| 与湍流度 Tu(%), 真实风洞测的就是这两个量
- **气动力面板** (仅 explore_gpu): forces.csv → Cd/Cl 时程曲线 + 后半段均值 +
  Cl 频谱峰涡脱频率与 St 数标注 (动量交换法, 标注参考面积与名义晶格 Re)
- **时间快照滑块** (仅 explore_gpu): 拖动回放 GPU 瞬态演化
- **显隐复选框**: 体填充 / 等值面 / 涵道壁 / PLA 边界层 / 截面标尺 / 截面窗子图
- explore_3d / explore_gpu 只关闭自己的旧窗口, 不会动 visualize_3d 的分析图

---

## 六、完整工作流示例

### 场景 1: 设计一款新风洞
```matlab
cd D:\WindTunnel3D
build_tunnel_model                 % 看侧视图, 检查壁面角 <35°/<7°
contraction_study                  % 选收缩段 L 和 R
boundary_layer_study               % 定试验段长度 (分层 vs 核心加速)
% 满意后改 tunnel_params.m 里的默认值
```

### 场景 2: 稳态流场检查
```matlab
visualize_3d                       % 4 张分析图秒出 (GIF 最后, 可 WT_NOGIF=1 跳过)
explore_3d                         % 切片细看任意截面
```

### 场景 3: 用你的模型测气动力 (GPU, 正式流程)
1. CAD 里把模型导出为二进制 STL → 存为 `D:\WindTunnel3D\test_object.stl`
   (无 STL 时可先用参数化测试物: MATLAB 里 `setenv('WT_OBJ','cylinder')` 后跑 export;
   模型最大可到试验段 300×200×200mm, 导出时自动打印包围盒/迎风面积/堵塞比诊断)
2. (可选, 推荐) 空风洞基线: MATLAB `setenv('WT_NOOBJ','1')` → export → bat → exe,
   完成后 `setenv('WT_NOOBJ','')` — gpu_report 会自动加载基线做对照并给堵塞修正 Cd
3. MATLAB: `export_fluidx3d` (生成网格与源码; 默认 2mm 全档, `WT_QUICK=1` 冒烟)
4. 资源管理器双击 `build_windtunnel.bat` (编译, 换模型后需重跑 3-4 步)
5. 双击 `fluidx3d\windtunnel.exe` (GPU 求解, 完成提示 "wind tunnel done")
6. MATLAB: `explore_gpu` — 切片看绕流, Q 准则看涡结构, 勾选"表面载荷"看模型表面
   气动载荷分布, 面板读 Cd/Cl/St
7. MATLAB: `gpu_mov` — 一键延时动画 `gpu_timelapse.mp4` (默认 Q 准则涡脱演化;
   `gpu_mov('spd')` 换场, `gpu_mov('turb',2)` 加速渲染; mp4 失败自动回退 GIF)
   + 切片分析窗同步留档 `gpu_timelapse_slices.mp4`
8. MATLAB: `gpu_slices` — 连续性档案: 时空图 `gpu_xt_y0.png` (斜条纹=涡连续输运)
   + 高频切片电影 `gpu_slice_movie_y0.mp4` (|u| 与涡量 ω_y 梯度双面板)
   + `gpu_slice_movie_zm.mp4` (水平切片) + 控制台相邻帧相关系数 (连续性定量检验)
9. MATLAB: `gpu_report` — 一页双重测算报告 `gpu_report.png`
   (流场质量/边界层发展/气动力/涡脱频谱/基线对照/堵塞修正汇总)

### 场景 4: 与你的 STEP 装配模型核对
`tunnel_body.stl` 流向为 STL +Z 轴, 任意 CAD (SolidWorks/Fusion/FreeCAD)
可直接导入并与 STEP 模型装配比对。

### 场景 5: MATLAB 动态风场 (快速演示)
`lbm_run` — 自动完成瞬态求解并输出:
- 图1: 测试物 Cd/Cl 时程 + 客观性文献基准线 (自动偏差诊断)
- 图2: 水平/竖直涡量切面 (对称涡街可见)
- lbm_p0_slices.png: **总压切片时序蒙太奇** (jet 彩虹渲染, 视频同款形式)
- lbm_fig3.png: 3D 涵道内壁半透明壳 + 测试物 + 涡量等值面
- 控制台: 客观性验证块 (Cd/St 偏差、分辨率与堵塞比诊断)

---

## 七、文件清单

| 文件 | 说明 |
|---|---|
| `tunnel_params.m` | ★ 全部参数与构型预设 (一切改动从这里开始) |
| `smoke_check.m` | 改参数后的几何/网格/域覆盖快速体检 (秒级, 不求解) |
| `tunnel_geometry.m` | 沿程几何/速度/壁面角 (a/b 双型线 + 5 种收缩型线库) |
| `build_tunnel_model.m` | 建模 + STL 导出 + 设计检查图 |
| `contraction_study.m` / `boundary_layer_study.m` | 设计研究脚本 |
| `flow3d_solve.m` | 稳态体素求解器 |
| `normalShock.m` | 正激波关系模块 (高速工况) |
| `visualize_3d.m` | 稳态求解 + 4 张分析图 + GIF |
| `explore_3d.m` | 稳态交互浏览器 (参数可就地改并重解) |
| `lbm3d_solve.m` / `lbm_run.m` | MATLAB 动态 LBM 求解器与驱动 |
| `tunnel_voxelize.m` | 动态域体素化 (涵道壁 + 测试物 STL) |
| `export_fluidx3d.m` | ★ MATLAB → FluidX3D GPU 衔接器 |
| `read_fluidx3d_vtk.m` | FluidX3D VTK 结果读回 MATLAB |
| `explore_gpu.m` | GPU 结果交互浏览器 (时间回放 + 气动力) |
| `gpu_mov.m` | 一键延时动画导出 (gpu_timelapse.mp4, 重运算标准后处理) |
| `gpu_slices.m` | 高频切片时序留档: 时空图 + 切片电影 + 连续性定量检验 |
| `gpu_report.m` | 风洞+被测物 双重测算一页仪表盘 (gpu_report.png) |
| `build_windtunnel.bat` / `fluidx3d\windtunnel.exe` | GPU 求解编译/执行 |
| `选型与安装清单.md` | 软件选型与安装记录 (FluidX3D 已装) |

结果存档: `flow3d_result.mat` / `lbm_result.mat` / `gpu_meta.mat` +
`fluidx3d\vtk\`(快照流场 u_/rho_/F_ + `mean_u.vtk` 时均场 + `tu.vtk` 湍流度%
+ 空风洞基线 `mean_u_base.vtk`/`tu_base.vtk`) + `forces.csv`(气动力时程) +
`fluidx3d\u_slice_y0/zm.raw`(高频切片时序流) + 视频档案
`gpu_timelapse.mp4`(3D) / `gpu_timelapse_slices.mp4`(切片窗) /
`gpu_slice_movie_y0.mp4` / `gpu_slice_movie_zm.mp4`(高频切片电影) +
`gpu_xt_y0.png`(时空图) + `gpu_report.png`(双重测算) +
`fluidx3d\vtk_prev\`(上一代归档)。

> **2026-09 修复**: `read_fluidx3d_vtk` 原按 SoA 解包矢量 VTK, 而 FluidX3D
> 实际写 AoS (每格点 x,y,z 相连) — 修复前所有矢量场 (流速/涡量) 均带 3 周期
> 分量交错的细微伪影, 修复后分量均值/剖面与理论一致。

### 逆压梯度与边界层分离 (两款求解器均覆盖)
| 求解器 | 方法 | 输出 |
|---|---|---|
| 稳态 `flow3d_solve` | 一维湍流边界层判据: R = (dp/dx·δ*)/τw (壁面摩擦对抗逆压的能力比, R→1 分离) | 逐站 R(x) + 自动判定 + R>0.9 站位自动嵌入近壁反向带 (流场中直接可见分离趋势) |
| `boundary_layer_study` | 同源 R(x) 曲线 + **速度剖面耙图** (扩散段轮廓+各站剖面, 参考图形式) + 陡扩散对比 (角度翻倍→R=1.68 必分离) | 分离风险曲线图 |
| 动态 `lbm3d_solve` | **真实求解**: 贴壁 2 格内逆流 (u_x<0) 体素占比逐站检测 | revFrac(x) + 自动判定 (当前附着流 0%) |

当前设计 (试验段 300×200 → 扩散段出口 400×260, L=0.75 m, 面积比 1.73, 全张角 2θ≈5.6°):
稳态求解判 "逆压显著但余量充足" — 扩散段逆压比 R_max=0.62 (默认 667×260×173 档实测,
R≥1 才分离, 余量 38%), 无近壁反向带嵌入。对比旧 150×150 设计 (面积比 4, 2θ≈12.2°,
R_max≈2.0 已入分离风险区): 大试验段使扩散段张角大幅收敛, 全程附着。
经验判据 2θ≤7° 留有余量; `boundary_layer_study` 的无粘一维口径更保守, 可复查陡扩散对比。

### 壁面摩擦对风速与边界层的影响 (大试验段 300×200 实测)

新设计下大截面使边界层相对更薄 (δ/半宽比旧 150 方形段小一倍), 摩擦增速与
核心均匀性同步改善; PLA 粗糙壁 (ks=0.2mm) 仍使边界层全程湍流 (cf 较光滑壁 +243%)。
逐站数值跑 `visualize_3d` / `explore_3d` 看控制台诊断与切片 (贴壁低速带直接可见),
设计研究用 `boundary_layer_study`。

物理链条: 收缩段强顺压把边界层压在 2mm 内 → 进入试验段后重新生长 → 排挤效应
使核心风速比无粘值略高 → 扩散段逆压使 δ 增厚 (当前张角 2θ≈5.6°, 增厚温和)。
试验段中部分层度 (核心速度不均匀度) <1% — 大截面的直接红利。

---

## 八、动态解算的客观性验证 (每次 lbm_run 自动输出)

lbm_run 结束时会打印"客观性验证"块，把 LBM 结果与文献基准对表：

| 指标 | LBM (快速档) | 文献基准 | 说明 |
|---|---|---|---|
| 圆球 Cd (演示 Re=250) | ~1.5 | 0.70 (Clift-Grace-Weber 相关式) | 偏差 +116%：3.8 格直径的体素阶梯 + 低分辨率 |
| 圆柱 Cd (演示 Re=250) | ~2.3 | 0.70 | 同上，横向圆柱 |
| 圆柱 St (涡脱) | 0.06~0.14 | 0.212 (Williamson) | 方向正确、量级可信；受尾流发展时间与分辨率限制 |
| 出口风速 | 25.0 m/s | 设计值 25 | 精确命中 (通量匹配) |
| 质量守恒 | 各站通量偏差 <3% | 0 | 连续性自检 |

**分辨率与堵塞比诊断**自动打印：直径晶格数 (<10 警告)、面积堵塞比。
**客观性结论**：演示档 Cd 半定量 (趋势与量级正确、数值偏高 1~2 倍)、St 定性正确、
速度场定量可信 (风速/通量精确)。正式定量请走 GPU 2mm 档 + 光滑 STL +
更小测试物 (直径≥15 格)，或转 OpenFOAM。
**文献来源**：Cd — Clift, Grace & Weber (1978)；St — Williamson (1997)。

### GPU 管线实测校验 (圆柱 d=30mm, 15 格直径, 名义晶格 Re=1200 + LES)

| 量 | GPU 实测 (forces.csv 频谱/均值) | 文献 (自由圆柱, Re~10^3) | 评注 |
|---|---|---|---|
| Cl_rms (升力脉动) | 0.35 | 0.3~0.5 | ✓ 量级吻合, 涡脱强而相干 |
| 涡脱 St | 0.41 | 0.21 | 偏高 ~2×: 15% 面积堵塞 + 圆柱贯通壁面 + 体素阶梯, 受限工况 St 上移 |
| 时均尾流 Tu | 尾流核心 60~78% | 同量级 | tu.vtk 逐步统计, 无欠采样 |
| 入口/外流 Tu | 入口 0.66%, 试验段 0.31~0.42% | 设计值 0.46% | 与 Tu 链 (蜂窝器/纱网/收缩衰减) 闭环 ✓ |

面板中 St 由 Cl 后半段 FFT 峰自动标注 (f·d/U); 升力取 Fz (竖直向)。
Cd 在受限+体素阶梯下偏高 (墙面贯通圆柱), 定性趋势与相对比较可用, 绝对值谨慎。

## 九、已知边界 (诚实声明)

- 稳态管线的边界层为子网格模型 (体素是单元平均值); 解析边界层结构需
  `P.flow.dx_cell` 降到 2mm 以下 (格数按 3 次方涨, 注意内存)
- MATLAB 动态 LBM 受晶体稳定上限约束, 演示 Re≤~900; Cd 为半定量
- GPU 管线名义晶格 Re=1200 (以试验段宽度计), SUBGRID LES 补足高 Re 湍流行为;
  2mm 晶格下体素球 Cd 偏高 (阶梯效应), 光滑 STL 模型 + 1mm 档可继续收敛
- 入口合成湍流为流函数法 2D 横向脉动 (y-z 面无散, 无流向分量 — 理想化的 2C 网格湍流),
  相位每 100 步才更新一次; 沿程衰减使试验段入口 Tu≈0.3~0.4% (设计 0.46%)
- Tu 湍流度链为一维经验式 (风扇/蜂窝器/纱网/收缩段衰减), 给数量级与设计趋势,
  非逐站 CFD 结果; GPU 的 tu.vtk 则是脉动直接解算的统计真值 (受晶格分辨率限制)
- 激波模块为 1D 关系式+逐体素马赫标记 (当前 Ma≈0.07 无激波, 教学用途)
- 验证环境: MATLAB R2026a (基础版, 无工具箱) + Windows 11 + RX 6650 XT

