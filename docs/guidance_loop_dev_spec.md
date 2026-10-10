# 制导闭环全仿真链路 — 算法对接技术开发文档

> 版本 v0.4 (2026-10-08) |
> v0.4: 补 §4.5.1-4.5.3 (控制链路/视觉链路/咬合验证); 全文中性表述并书面化润色。
> v0.3: 补 §5.1.1 3D 环境引擎分阶段选型 (PyBullet→Unity/UE→Gazebo 路线, Isaac Sim 因 A 卡排除)。
> v0.2: 补 §5.2 环境配置从零自举、§5.3 仓库骨架、§8 责任矩阵; 修正 sim_core 状态表述 (待建, 非已建)。
> 读者: 负责目标识别与制导律的算法同学。CFD/气动侧细节见 §6 参考文档。
> 主线选型: **Python + PyBullet**(方案 C, 理由 §5)。文档中所有接口以此为准。

---

## 1. 链路总览

```
┌─────────── 离线(一次性) ───────────┐
│ 参数化几何(弹体+4舵面)              │
│   → CFD 舵偏/攻角扫气              │
│   → 气动数据库 aero_table.csv      │
└──────────────┬───────────────────┘
               ▼
┌─────────── 闭环仿真 (500 Hz) ──────────────────────────┐
│  6DOF 刚体动力学 ← 气动查表 ← 舵机模型(±20°/300°ps/10ms) │
│         ▲                                    │         │
│         │ δ=[δ1..δ4]                         ▼         │
│  ┌──────┴───────┐   误差   ┌────────────┐  视线角     │
│  │ 制导律 (算法侧)│ ◄─────── │ 目标识别(算法侧)│ ◄─── 虚拟摄像头(33Hz RGB) │
│  └──────────────┘          └────────────┘             │
└───────────────────────────────────────────────────────┘
【评估】Monte Carlo 撒初始扰动+风 → 脱靶量 CEP 统计
```

- **需求开发范围**: ④ 目标识别模块 + ⑤ 制导律模块 (§4.3-4.5 接口)。
- **CFD 侧**: 参数化几何生成器、CFD 扫气管线、**气动数据库 aero_table.csv** (含可信域标定; 声学需求如舵面风噪另立专项)。
- **算法负责**: ④ 目标识别 + ⑤ 制导律 (§4.4/4.5), 以及 **sim_core/虚拟相机的搭建** (§5.2 附录给骨架与验收单, 按 README 自举; 气动查表函数由 CFD 侧提供参考实现)。

## 2. 坐标系与约定

| 系 | 记号 | 定义 | 用途 |
|---|---|---|---|
| 世界系 | W (ENU) | x 东, y 北, **z 上**, 右手系 | 弹道位置/速度、PyBullet 原生 |
| 机体系 | B | **x 前弹轴**, y 右, z 下, 右手系 | 气动力、角速度、舵面定义 |
| 相机系 | C | **z 前光轴**, x 右, y 下 (CV 惯例) | 识别模块的输入几何 |
| 图像系 | I | u 右, v 下, 原点左上 (px) | 检测框/像素坐标 |

- 姿态: **四元数 q = [qw qx qy qz]** (W→B 旋转), 全程禁用欧拉角做内部运算(展示打印可用)。
- 旋转向量约定: 所有三元组在文档/接口中显式标注所属系, 如 `v_B [m/s]`。
- B 系与 PyBullet URDF 系(x前y左z上)的转换在仿真器内完成, 对外仅暴露 B/C/I 三系。

## 3. 模块划分与数据流

```
[仿真器 sim_core]      [识别 detector(算法侧)]      [制导 guidance(算法侧)]
 状态 x_k = {p_W, v_W, q, ω_B}                     混控矩阵 M(4x3)
 舵机模型(限位/速率)        img + meta ──► 检测+PnP ──► 误差 e ──► δ_cmd
 气动查表 aero(x, δ)                ▲
        ▲                           │
        └────── 虚拟相机 render(位姿) ┘ (33 Hz, 仿真器内建)
```

时间基准: 仿真主循环 **dt = 2 ms (500 Hz)**; 相机/识别 **30 Hz**(每 ~17 个主循环一帧);
识别延迟建模 66 ms(2 帧), 由制导律侧决定是否补偿; 全链时间戳统一仿真秒 t_sim。

## 4. 接口规范

### 4.1 气动数据库 (CFD 侧交付, 只读)

`aero/aero_table.csv`, 长表格式, 一行一个工况:

```
V[m/s], alpha[deg], beta[deg], de[deg], da[deg], dr[deg], CL, CD, Cm, CY, Cl, Cn
```

- 用途: 仿真器内三线性插值出气动力/力矩系数 → 力/力矩。
- 4 舵映射: +字布局约定 δ1/δ2=水平对(差动=滚转, 同向=俯仰), δ3/δ4=垂直对(同向=偏航)。
  `de=(δ1+δ2)/2, da=(δ1−δ2)/2, dr=(δ3+δ4)/2` — 混控细节 Phase 0 与布局一起冻结。
- 算法侧无需直接修改此文件; 制导律调参需要"舵偏效率导数"曲线时向 CFD 侧索取。

### 4.2 仿真器 sim_core (待建 — 算法侧按附录 §5.2 骨架自举, 接口冻结如下)

```python
sim = SimCore(cfg)                       # cfg: 初值/质量/惯量/舵机参数/风
obs  = sim.step(delta_cmd)               # 推进 dt=2ms
# obs: dict {
#   t, p_W(3), v_W(3), q(4),          # 状态
#   omega_B(3), acc_B(3),             # 角速度/比力(过载用)
#   elevator/aileron/rudder_actual,    # 舵机实际偏角(含动力学)
#   hit, miss_distance                # 终止标志与脱靶量
# }
```

- 舵机模型: 一阶滞后 τ=10ms + 速率限制 300°/s + 行程 ±20°, 输入为指令角。
- 风扰动: `cfg.wind` 常值/阵风(Dryden 可选), Monte Carlo 时注入。
- 终止: 命中(<命中半径)/落地/超时/数值发散。

### 4.3 虚拟摄像头 (与 sim_core 同包实现, 接口冻结如下)

```python
frame = sim.get_camera()                 # dict {
#   'img':  (480, 640, 3) uint8 RGB,
#   'K':    (3,3) 相机内参 (fx, fy, cx, cy), FOV 60°,
#   'T_WC': (4,4) 相机位姿 (相机刚性固连弹头, 视场朝前),
#   't':    仿真时间戳
# }
```

- 渲染: PyBullet 内建 renderer, 场景=地面+靶标+可选障碍物; 光照固定(seed 可复现)。
- 分辨率/FOV 可调但 **30Hz+640×480 是标称工作点**, 调整需双方确认。

### 4.4 目标识别模块 (算法侧开发)

```python
class Detector:
    def __init__(self, **params): ...
    def detect(self, frame) -> dict:
        # 输入:  §4.3 的 frame dict
        # 输出:  {
        #   'found': bool,
        #   'uv':     (u, v) 目标参考点像素坐标,          # 必交
        #   'los_C':  (3,) 视线单位向量 (相机系),          # 必交 (由 uv+K 反算或 PnP)
        #   'p_T_C':  (3,) 目标位置 (相机系, m),           # PnP 可得时必交
        #   'conf':   float 0-1, 'latency_ms': float      # 建议附上
        # }
```

- 靶标: Phase 3 用 **ArUco 6x6 (id=0, 边长 0.5m)** 贴在 2×2m 靶板上 (高对比、可复现);
  后续可选真实纹理图像+YOLO (Phase 6 之前的升级项)。
- 验收: 标称弹道全视场 **检测率 ≥99%**, 静止像素抖动 ≤2px, 单帧耗时 ≤20ms。

### 4.5 制导律模块 (算法侧开发)

```python
class Guidance:
    def __init__(self, **params): ...
    def update(self, obs, detection, dt_vis=1/30) -> (4,):
        # 输入: 仿真器状态 obs (§4.2) + 识别输出 (§4.4)
        # 输出: δ_cmd = [δ1,δ2,δ3,δ4] 弧度, 行程 ±20° (仿真器内再限幅)
```

识别丢失时的保持策略(锁定上帧 LOS / 归零舵差动)需写入状态机; 比例导引项与姿态稳定项分开调参。
控制与视觉两条链路的参考结构和数学依据见 §4.5.1-4.5.3, 可推翻重设计, 但接口不变。

#### 4.5.1 参考结构 I — 控制链路: 舵偏 → 姿态 (气动因果链)

物理链条: 舵偏 δ → 舵面附加升力 Lδ = ½ρV²·S·CLδ·δ → 对质心力矩 Mδ = Lδ × lδ(力臂)
→ 角加速度 ω̇ = Mδ/I → 姿态角速度 → 姿态角。实现分三层:

| 层 | 内容 | 用途 |
|---|---|---|
| 真值层 | CFD 扫气表 (aero_table): Cm/Cl/Cn 随 (α,β,δ) 变化, 舵效 Cmδ = ∂Cm/∂δ 由表差分 | 仿真器查表 (闭环真值) |
| 线性化层 | 工作点线性化 → 状态空间 ẋ = Ax + Bδ (B 即舵效矩阵) | 调 PID/PN 的设计模型, 避免在非线性 6DOF 上盲调 |
| 补偿层 | 动导数 Cmq / Cmα̇ 静扫不给 → 工程经验值近似 | 仿真器修正项, M3 前标定 |

三个必须处理的物理细节:
- 静稳定性 (Cmα < 0): 弹体天然回中, 舵面操纵即是对该特性的利用或对抗;
- 质心位置: 力臂 lδ 从质心量起 — 质心配置直接缩放舵效 B 矩阵 (与 URDF inertial 参数咬合);
- 舵机动力学: 指令 δcmd → 实际 δ 有 10ms 滞后 + 300°/s 速率限制, 内环增益过高会与其共振震荡。

#### 4.5.2 参考结构 II — 视觉链路: 误差 → 期望控制效果 (制导几何)

视觉链路输出的不是"控制效果", 是"误差"; 误差到控制的翻译由制导律完成。标准双环:

```
外环制导 (30 Hz, 视觉驱动):
  像素偏差 (u,v) —K 矩阵→ 视线角误差 (Δaz, Δel) —连续帧差分→ LOS 变化率 ω_LOS
  → 比例导引: 过载指令 a_cmd = N · V · ω_LOS   (N = 3~5)
内环控制 (500 Hz, 气动驱动):
  a_cmd → 期望攻角 α_cmd ≈ a_cmd·m / (q̄·S·CLα) → 姿态误差 → PID → 舵偏 δ
```

三个工程要点:
- 像素→角度映射无需标定: 虚拟相机 K 矩阵为仿真真值, Δ角 = atan((u−cx)/fx), 无标定误差;
- 延迟补偿: 66 ms 视觉延迟 × 25 m/s ≈ 1.65 m 空间滞后, 将误差直接引入回路会引起震荡 —
  用 LOS rate 外推 (α-β 滤波起步, 不够再上 KF);
- 内外环频率解耦: 外环给出弹道方向指令, 内环负责姿态跟踪, 舵机动力学位于内环带宽内。

#### 4.5.3 两条链路的咬合验证 (M3 之前必做)

- 控制链路标定: 施加 δ 阶跃 → 实测姿态响应时间/超调, 与线性化层 B 矩阵预测对比,
  偏差 >20% 说明气动表或动导数近似有问题;
- 视觉链路标定: 目标位置阶跃 10 px → 闭环修正应无持续震荡/超调发散, 验证外环增益与延迟补偿;
- 两项通过后才允许进入 Monte Carlo (§7 M3)。

### 4.6 打靶评估器 (与 sim_core 同包, 接口冻结如下)

```python
from eval import monte_carlo
monte_carlo(n=1000, guidance=YourGuidance, detector=YourDetector)
# 输出: cep50/cep90 直方图, 逐发弹道 csv, 丢帧/饱和统计
```

## 5. 选型结论、环境配置与自举

### 5.1 选型结论

- **主线 = Python 3.11 + PyBullet + OpenCV + numpy/scipy** (方案 C):
  闭环 500Hz 单核可跑, 虚拟相机原生, 单机夜间可完成数千发 Monte Carlo。
- 备选已评审: MATLAB/Simulink(控制验证), Unity/ML-Agents(展示/RL), Gazebo/ROS2(上真机前工程化) —
  接口语义与 §4 一致, 需要时再做桥接, **算法逻辑零重写**。

#### 5.1.1 3D 环境引擎分阶段选型 (视觉层与物理层解耦, 引擎是可替换插件)

架构约束: 6DOF 物理永远在 sim_core (PyBullet), **环境引擎的唯一职责是 `render(位姿) → 图像`** —
`sim_core/camera.py` 按 Renderer 抽象实现, 换引擎只写新桥, 识别/制导代码零改动。

| 阶段 | 引擎 | 理由 |
|---|---|---|
| M1-M3 算法验证 (现在) | **PyBullet 内建渲染** | 零成本, ArUco/色块靶标足够, 可复现跑得快 |
| M4 鲁棒性对抗 | **Unity 2022 LTS** (socket/共享内存出图; Sentis 可内嵌识别模型) 或 **UE5 + Colosseum** (AirSim 系, `simGetImage` RPC) | 真实纹理/光照/遮挡逼出识别与制导的弱点 |
| 上真机前工程化 | **Gazebo (gz-sim)** | 与 ROS 2 传感器接口同生态, camera topic 直迁 |
| 1 | ~~Isaac Sim / Omniverse                                      | 可换选 |
| 离线合成数据 (训视觉模型时) | **Blender** (渲染太慢不进闭环, 只离线批量生成训练集) | 域随机化强 |

### 5.2 环境配置

```bash
# 0) Python 3.11 (已有 3.10-3.12 可直接使用, 否则安装)
winget install -e --id Python.Python.3.11        # 或 https://python.org 安装包, 勾选 Add to PATH

# 1) 项目虚拟环境 (仓库根目录下)
cd D:\WindTunnelSuite\guidance_sim
python -m venv .venv
.venv\Scripts\activate                            # Git Bash 用 source .venv/Scripts/activate

# 2) 依赖 (国内镜像; 版本为本项目标定值, 勿随意升级)
pip install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple
# requirements.txt 内容:
#   pybullet==3.2.6
#   opencv-python==4.10.0.84
#   numpy==1.26.4
#   scipy==1.13.1
#   matplotlib==3.9.2

# 3) 验证 (三项全部通过即环境就绪)
python -c "import pybullet, cv2, numpy, scipy; print('env-ok', cv2.__version__)"
python scripts\hello_pybullet.py                  # 弹出 3D 窗口+落地板即渲染链路 OK
python scripts\selftest_simcore.py                # 6DOF 无控下落 3s, 状态连续无 NaN 即 OK
```

常见坑:
- `pip install pybullet` 编译报错 → 确认用的 Python 3.10-3.12 (3.13 无 wheel, 需本地编译);
- OpenCV 报 DLL 缺失 → 装 [VC++ 2015-2022 Redistributable x64](https://aka.ms/vs/17/release/vc_redist.x64.exe);
- 公司代理/离线机 → 用 `pip download -d wheels/` 在线机打包带回, `pip install --no-index --find-links=wheels/ -r requirements.txt`;
- 无显示器远程机 → `hello_pybullet.py` 用 `p.DIRECT` 模式 (脚本里已留开关), 图像照常出。

### 5.3 仓库骨架 (已提交, 直接填空)

```
guidance_sim\
├─ requirements.txt
├─ aero\aero_table.csv               ← CFD 侧交付 (M0 后到位)
├─ sim_core\sim_core.py              ← 待建: 按本文 §4.2 接口; 气动查表参考实现 lookup_aero.py 已给
├─ sim_core\camera.py                ← 待建: §4.3 接口; 按 Renderer 抽象实现 (换引擎见 §5.1.1)
├─ detector\      (算法侧)           ← §4.4
├─ guidance\      (算法侧)           ← §4.5
├─ eval\monte_carlo.py               ← 待建: §4.6
├─ scripts\hello_pybullet.py         ← 已提交 (环境验证用)
├─ scripts\selftest_simcore.py       ← 已提交
└─ scripts\record_nominal.py         ← 已提交: 跑标称弹道录 500 帧图像 (M1 的输入)
```

自举顺序: §5.2 环境 → 运行 scripts 下三个验证脚本 → 依 §4.2 接口实现 sim_core (先"无气动力自由落体"版, 气动查表参考实现可直接复用) → M1。
问题支持: 接口与气动语义问题由曼波 (CFD 侧) 负责; 文档未覆盖的自行决策要记录在 PR 描述里。

## 6. 目录与参考

```
D:\WindTunnelSuite\
├─ docs\guidance_loop_dev_spec.md   ← 本文
├─ docs\transient_rotation_plan.md  ← (既有) 瞬态旋转调研, 方法论参考
├─ guidance_sim\                    ← Phase 1 建立: sim_core / detector / guidance / eval
└─ openfoam\                        ← 气动库来源 (gen_* 生成器 + v12 扫气), 算法侧无需改动
```

- CFD 侧背景 (可选读): AUTOPILOT.md §2 状态快照; devlog/2026-10-06.md。
- 气动库可信域: V ∈ [15, 30] m/s, α ∈ [−5°, 15°], 出域插值结果**未标定**——制导律设计留余量。

## 7. 里程碑与验收

| 阶段 | 内容 | 验收标准 |
|---|---|---|
| M1 | 识别模块单元测试 (录制的 500 帧序列) | 检测率 ≥99%, 抖动 ≤2px, ≤20ms/帧 |
| M2 | 制导律闭环 (无风无扰) | 命中率 100% (n≥20), 舵面不持续饱和 |
| M3 | Monte Carlo 1000 发 (扰动+风) | CEP ≤ 指标值 (Phase 0 定), 无发散弹道 |
| M4 | 对抗性 (靶标移动/光照变化, 可选) | 双方商定 |

## 8. 责任矩阵与对接清单

| 模块 | 开发 | 验收 | 备注 |
|---|---|---|---|
| 参数化几何 + CFD 扫气 + 气动库 | **曼波** (气动声学仿真侧) | 双方 | 曼波只负责 CFD/气动(声学另立专项), 软件环境由算法侧负责 |
| sim_core (6DOF/舵机/气动查表) | 算法侧 (骨架+参考实现由 CFD 侧给) | 双方联调 | §5.2 自举, 遇气动语义问题由曼波支持 |
| 虚拟相机 | 算法侧 (同上) | 双方联调 | §4.3 |
| 目标识别 Detector | 算法侧 | M1 指标 | §4.4 |
| 制导律 Guidance | 算法侧 | M2/M3 指标 | §4.5 |
| 打靶评估器 monte_carlo | 算法侧 (指标定义双方冻结) | 双方 | §4.6 |

**算法侧从 CFD 侧获取**: 气动库 aero_table.csv + 气动查表参考实现 lookup_aero.py + 相机安装位姿参数 + 本文档。
**算法侧向 CFD 侧交付**: Detector/Guidance 两个类 (§4.4/4.5 签名) + 参数文件 + M3 评估结果。
**双方冻结 (Phase 0 会议)**: 舵面布局与混控矩阵、靶标规格、命中半径、CEP 指标、相机安装位姿、飞行速度域。
