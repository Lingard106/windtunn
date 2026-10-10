function P = tunnel_params(config)
% 桌面级风洞总体参数 (单位: m, 除非注明)
% 目标: 试验段 20-30 m/s 稳定风速
% 构型预设: 'balanced'(均衡) | 'lowTurb'(低湍流) | 'compact'(紧凑) | 'aniso'(矩形截面)
if nargin < 1, config = 'balanced'; end
P.config = config;

%% ---- 运行工况 ----
P.V_test      = 25;        % 试验段设计风速 (m/s)
P.V_range     = [20 30];   % 风速范围

%% ---- 试验段 (宽 W / 高 H / 长 L 均独立可调) ----
P.ts.W        = 0.300;     % 试验段宽度 (m)
P.ts.H        = 0.200;     % 试验段高度 (m)
P.ts.L        = 0.200;     % 试验段长度 (m)
P.ts.growth   = 0.2;       % 壁面微扩张角 (deg, 补偿边界层排挤)

%% ---- 收缩段 (研究变量: R 比 / 长度 / 型线构型) ----
P.ct.curve    = 'quintic'; % 型线: 'quintic'|'cubic'|'sine'|'linear'|'twostage'
switch config
    case 'lowTurb'        % 低湍流构型: 大收缩比+长收缩
        P.ct.R = 8;   P.ct.L = 0.42;  P.ct.curve = 'quintic';
    case 'compact'        % 紧凑构型: 小收缩比+短收缩
        P.ct.R = 5;   P.ct.L = 0.22;  P.ct.curve = 'cubic';
    case 'aniso'          % 矩形试验段构型
        P.ts.W = 0.20;  P.ts.H = 0.12;  P.ct.R = 6;  P.ct.L = 0.30;
        P.df.W_out = 0.34;  P.df.H_out = 0.22;
    otherwise             % 均衡构型: L=0.60 使大收缩段峰值壁面角 ≤34° (quintic 峰值斜率 1.875)
        P.ct.R = 6;   P.ct.L = 0.60;
end
P.ct.R        = P.ct.R;    % 收缩比 R = A_in/A_out (可直接修改)

%% ---- 前置扩散段 (风机出口法兰 -> 稳定段入口; 供 FanSim3D 抽风式风机对接) ----
P.fd.W_in  = 0.300;   % 风机出口法兰宽 (m) — 须与 FanSim3D fan_params 的 P.vol.W_out 一致
P.fd.H_in  = 0.270;   % 法兰高 (m) — = FanSim3D 蜗壳宽 B
P.fd.L     = 0.500;   % 前置扩散段长 (m) — 宽角扩散 (面积比 ~4.4), 出口带整流网
P.fd.etaD  = 0.72;    % 扩散效率 (宽角+整流网经验值)
P.fd.screen= true;    % 出口整流网 (稳定宽角扩散边界层)

%% ---- 稳定段 ----
P.sc.L        = 0.450;     % 稳定段长度 (m) — ≥0.6×入口当量尺寸, 保证纱网后沉降
P.sc.honeyL   = 0.050;     % 蜂窝器长度 (m)
P.sc.honeyD   = 0.004;     % 蜂窝器孔径 (m)
P.sc.nScreen  = 2;         % 纱网层数
P.sc.screenMesh = 30;      % 纱网目数

%% ---- 扩散段 (出口截面独立可调) ----
if ~strcmp(config, 'aniso')
    P.df.W_out = 0.400;    % 扩散段出口宽度 (m) — 面积比 1.73, 等效锥角 2θ≈5.6°<7° 无分离
    P.df.H_out = 0.260;    % 扩散段出口高度 (m)
end
P.df.L        = 0.750;     % 扩散段长度 (m)

%% ---- 壁面材料: PLA (3D 打印, 层纹粗糙度影响边界层) ----
P.wall.on       = true;     % 粗糙壁物理开关 (关闭=光滑壁解析解)
P.wall.material = 'PLA';
P.wall.ks       = 0.0002;   % 当量砂粒粗糙度 (m): FDM 打印件层纹 ~0.2mm
                            % (打磨/涂层后可降到 0.00005)

%% ---- 动态 LBM 风场 (风洞涵道 + 测试物联合瞬态仿真) ----
P.lbm.on        = true;
P.lbm.dx        = 0.004;    % 晶格尺寸 (m): GPU 建议 4mm; CPU 快速模式自动放宽
P.lbm.uLx       = 0.08;     % 晶格速度上限 0.08 (Ma≈0.14; 超出部分自动截断保稳定)
P.lbm.Re        = 1200;     % 演示雷诺数 (数值稳定上限; 真实 Re~1e5 需外部求解器)
P.lbm.steps     = 4000;     % 演化步数 (~4-6 个 through-flow, GPU 约几分钟)
P.lbm.rampFrac  = 0.2;      % 来流爬坡段占比 (软启动防失稳)
P.lbm.snapEvery = 250;      % 快照间隔步数
P.lbm.object    = 'stl';    % 测试物: 'stl' (推荐, 用户提供模型) | 'sphere'|'cylinder'|'plate'|'naca'
P.lbm.objSize   = 0.030;    % 参数化测试物的特征尺寸 (m); STL 模型保持原始尺寸
P.lbm.objFile   = 'test_object.stl';   % STL 路径 (CAD 导出二进制 STL, 中心自动对齐试验段)
P.lbm.objMotion = 'rot';    % 'static' | 'rot' (绕 x 轴旋转) | 'osc' (俯仰振荡)
P.lbm.margins   = [0.12 0.28]; % 动态域在试验段前/后的延伸 (m)


%% ---- 流体物性与物理开关 ----
P.nu          = 1.516e-5;  % 运动粘度 (m^2/s)
P.rho         = 1.204;     % 密度 (kg/m^3)
P.aSound      = 343;       % 声速 (m/s)
P.gamma       = 1.4;       % 比热比
P.transitionRe= 5e5;       % 平板转捩雷诺数 Re_x
P.gravityOn   = false;     % 竖直界面浮力分层开关 (Boussinesq)
P.dT_wall     = 2.0;       % 底-顶壁温差 (K), gravityOn 时生效
P.T0          = 300;       % 参考温度 (K)

%% ---- 派生尺寸 (收缩比 R 决定入口截面; 网格与各求解器共用) ----
ar = sqrt(P.ct.R);
P.ct.W_in = P.ts.W * ar;   % 稳定段/收缩段入口宽 (m)
P.ct.H_in = P.ts.H * ar;   % 入口高 (m)

%% ---- 3D 体素求解器设置 (按目标单元尺寸自动定网格, 默认即最高档) ----
P.flow.dx_cell = 0.003;    % 目标单元尺寸 (m): 试验段边界层全程解析 (大截面下 δ≥2.5 格)
P.flow.sweeps  = 300;      % 每站横向泊松 Jacobi 扫描次数 (跨站热启动, 单精度扫描)
% 网格由几何派生 (域: 全长 L, 宽 W_in×1.06, 高 H_in×1.06, 与 flow3d_solve 域一致)
Lt = P.fd.L + P.sc.L + P.ct.L + P.ts.L + P.df.L;
P.flow.nx = max(60, round(Lt/P.flow.dx_cell));            % 流向单元数
P.flow.ny = max(32, round(P.ct.W_in*1.06/P.flow.dx_cell)); % 横向单元数 (矩形截面 y/z 独立)
P.flow.nz = max(32, round(P.ct.H_in*1.06/P.flow.dx_cell)); % 竖向单元数
% 降档 (visualize_3d/explore_3d 自动等比降采样):
%   setenv('WT_MID','1')   → 单元 ×2   (求解 ~25 s, 快速查看)
%   setenv('WT_QUICK','1') → 单元 ×6   (秒级冒烟验证)
end
