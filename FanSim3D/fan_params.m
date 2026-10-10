function P = fan_params(preset)
% 抽风式离心风机 (前弯多叶 + 蜗壳) 总参数 — FanSim3D 并行项目
% 使命: 为 WindTunnel3D 桌面风洞 (试验段 300x200mm, 设计风速 25 m/s) 供风
% 链路: 吸入钟形口 -> 叶轮(后盘+叶片+前盘) -> 蜗壳 -> 出口弯头 -> 出口法兰
%       法兰 = WindTunnel3D 前置扩散段入口 (tunnel_params 的 P.fd.W_in/H_in 必须与此一致)
% 参考构型: 抽风式离心风机 (吸入口轴向进气, 蜗壳切向排出), 前弯多叶/后弯均可参数化
% 三个核心设计变量 (对应曼波的需求): 叶片出口角 beta2A, 叶片长度 (D2-D1)/2, 整体尺寸比 (D1/D2, b2/D2, Z)
if nargin < 1, preset = 'balanced'; end
P.preset = preset;

%% ---- 任务工况 (与 WindTunnel3D 对表) ----
% 套件相对定位: WindTunnel3D 与本项目同级 (<suite>/WindTunnel3D); 环境变量 WT3D_DIR 可覆盖。
% 不再硬编码盘符 — 整个套件文件夹移动后无需改代码。
FAN = fileparts(mfilename('fullpath'));
WT = getenv('WT3D_DIR');
if isempty(WT), WT = fullfile(FAN, '..', 'WindTunnel3D'); end
P.load.path      = WT;                  % 风洞项目路径 (取 tunnel_params 做单一真源)
P.load.V_test    = 25;                 % 目标试验段风速 (m/s), 范围 20-30
P.load.marginQ   = 0.15;               % 风量设计裕度 (风机留 15% 风量能力, 变频调节用)

%% ---- 叶轮 (核心设计变量集中在这里) ----
% 默认 = fan_sweep 前弯家族最优 (2026-09-27 扫描: beta2A=130°, D1/D2=0.60, η静 41%, 裕度 68%)
P.imp.D2      = 0.46;    % 叶轮外径 (m) — 整体尺寸的基准
P.imp.D1      = 0.276;   % 叶片进口直径 (m) — 决定叶片径向长度 (D2-D1)/2 = 92mm
P.imp.D0      = 0.375;   % 吸入口眼径 (m) — 眼环 (D0>D1 为增速环, 模型按突扩损失计)
P.imp.Z       = 32;      % 叶片数 — 前弯多叶 24-64, 后弯 6-16
P.imp.beta2A  = 130;     % 叶片出口安装角 (deg, 自切向量起, >90=前弯, <90=后弯, 90=径向) ← 核心变量
P.imp.beta1A  = [];     % 叶片进口角 (deg, 自切向量起) — []=按设计流量无冲击自动求解
P.imp.b2      = 0.24;    % 叶轮出口宽度 (m) — 前弯多叶 b1=b2 (等宽), b2/D2≈0.45-0.65
P.imp.tB      = 0.003;   % 叶片厚度 (m) — 3D 打印建议 ≥2.4mm
P.imp.rpm     = 960;     % 转速 (6 极电机直联; 2 极 2900 / 4 极 1440; 变频微调命中 25 m/s)
P.imp.shroudT = 0.004;   % 前/后盘厚度 (m)
P.imp.hubR    = 0.030;   % 轴毂半径 (m)
P.imp.clearR  = 0.003;  % 叶轮外缘-蜗壳侧隙 (m)
P.imp.slipK   = [];     % 滑移修正系数 ([]=自动: beta2A<=90 用 Wiesner, >90 乘 1.15 前弯修正)
P.imp.etaMech = 0.97;   % 机械效率 (直联)

%% ---- 蜗壳 (蜗牛壳) ----
P.vol.kv      = 0.80;    % 蜗壳设计周速系数: c_th = kv·c2u (0.7-0.9, 大=蜗壳窄)
P.vol.B       = 0.27;    % 蜗壳宽度 (m) — ≥b2+侧隙, 出口高度=此值
P.vol.gapT    = 0.020;   % 舌部间隙 (m) — 0.04-0.08·D2, 小=高压高效但噪声大
P.vol.angT    = 18;      % 舌部安装角 (deg)
P.vol.W_out   = 0.30;    % 蜗壳出口宽度 (m) — 出口流速 = Q/(W_out·B)
P.vol.etaV    = 0.65;    % 蜗壳动能回收效率 (0.55-0.75)
P.vol.zetaV   = 0.10;    % 蜗壳摩擦损失系数 (referenced to c2)
P.vol.bendR   = 0.18;    % 出口 90° 弯头中线半径 (m) — 切向出口转轴向进风洞
P.vol.zetaBend= 0.20;   % 弯头损失系数 (无导叶矩形 90° 弯头)

%% ---- 出口法兰 (= WindTunnel3D 前置扩散段入口, 须与 tunnel_params P.fd 一致) ----
P.fd.W_in     = 0.30;    % 法兰宽 (m) — = P.vol.W_out
P.fd.H_in     = 0.27;    % 法兰高 (m) — = P.vol.B

%% ---- 叶轮内损失模型系数 (1D 模型标定, GPU 求解可校验) ----
P.loss.zetaInc  = 0.70;  % 进口冲击损失 (referenced to w1, x sin^2 i)
P.loss.zetaCurv = 0.12;  % 叶道弯曲损失基准 (referenced to w1, 随前弯度增大)
P.loss.zetaRec  = 0.05;  % 二次流/回流损失系数 (referenced to u2^2, 前弯自动加码)
P.loss.etaVol   = 0.985; % 容积效率 (叶顶漏气)
P.loss.cM0      = 0.006; % 盘摩擦矩系数基准 (Re 修正)
P.loss.fBlade   = 0.018; % 叶片表面摩擦系数基准

%% ---- GPU 求解 (FluidX3D, 参照 WindTunnel3D 动态渲染管线) ----
P.gpu.dx      = 0.003;   % 晶格 (m): QUICK=0.006 / 默认=0.003 / HI=0.002
P.gpu.steps   = 30000;   % 步数 (含瞬态旋转叶轮)
P.gpu.snap    = 1500;    % VTK 快照间隔
P.gpu.diag    = 100;     % 流量/转矩诊断采样间隔 (步)
P.gpu.rotEvery= 120;     % 叶轮重体素化间隔 (步): 120 步转 ~2°, 每转 ~43 次
P.gpu.uLx     = 0.10;    % 晶格特征速度 (叶尖 0.13lx, 仍在稳定区)
P.gpu.Re      = 1200;    % 名义晶格 Re (以试验段宽计, SUBGRID LES 补足高 Re, 同风洞口径)
P.gpu.rotate  = false;   % 默认=冻结转子 (frozen rotor, 已验证稳定); true=瞬态旋转叶轮(实验性, 重体素化路径待调)
P.gpu.tuFan   = 0.10;    % 叶轮出口湍流度估计 (供对比, GPU 直接解算脉动)

%% ---- 预设 (一键换构型) ----
switch preset
    case 'hiStatic'     % 高静压构型: 大叶轮, 兼顾更长的扩散链/滤网负载
        P.imp.D2 = 0.50;  P.imp.D1 = 0.30;  P.imp.D0 = 0.40;  P.imp.b2 = 0.26;
        P.imp.Z = 32;  P.imp.beta2A = 140;  P.imp.rpm = 960;
    case 'compact'      % 紧凑构型: 小叶轮 4 极, 占地小 (η 略低)
        P.imp.D2 = 0.38;  P.imp.D1 = 0.23;  P.imp.D0 = 0.30;  P.imp.b2 = 0.20;
        P.imp.Z = 36;  P.imp.beta2A = 135;  P.imp.rpm = 1440;
        P.vol.B = 0.22;  P.vol.W_out = 0.26;  P.fd.W_in = 0.26;  P.fd.H_in = 0.22;
    case 'backward'     % 后弯构型: fan_sweep 全域最优家族 (η静≈45%), 曲线平坦噪声低
        P.imp.D2 = 0.46;  P.imp.D1 = 0.276;  P.imp.D0 = 0.375;  P.imp.b2 = 0.22;
        P.imp.Z = 14;  P.imp.beta2A = 60;  P.imp.beta1A = 35;  P.imp.rpm = 1070;
    otherwise           % 'balanced' 均衡: 前弯多叶 (与参考图一致), 6 极直联
end

%% ---- 一致性派生 ----
P.imp.bladeLen = (P.imp.D2 - P.imp.D1)/2;   % 叶片径向长度 (m) ← 核心变量之一
P.imp.D1D2     = P.imp.D1/P.imp.D2;
P.imp.b2D2     = P.imp.b2/P.imp.D2;
P.imp.u2       = pi*P.imp.D2*P.imp.rpm/60;  % 叶尖速度 (m/s)
P.vol.r2       = P.imp.D2/2;
P.vol.rTongue  = P.vol.r2 + P.vol.gapT;     % 舌部半径
P.vol.rEnd     = [];                        % 蜗壳终端半径 (fan_geometry 按型线算)
end
