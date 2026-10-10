function G = fan_geometry(P, R)
% fan_geometry — 抽风式离心风机 3D 几何: 叶轮(圆弧叶片+前/后盘) + 蜗壳 + 轴向出口 + 法兰
% 布局 (x = 轴向/风洞流向, 叶轮旋转平面为 y-z, 旋转正向 +y→+z 绕 +x):
%   钟形口 -> 吸力眼 -> [后盘|叶片|前盘] -> 蜗壳腔(上/下游侧板夹持, 阿基米德蜗壁)
%   -> 下游侧板轴向出口方孔(W_out×B) -> 等截面短管 -> 出口法兰(=风洞前置扩散段入口)
% 输入: P=fan_params; R=fan_performance 结果(缺省自动算, 取设计点 c2u 定蜗壳型线)
% 输出: G — 尺寸布局表 Lx/半径/型线 + 叶片造型数据 + 可选网格面片 (G.mesh)
if nargin < 2 || isempty(R), R = fan_performance(P); end
if ~isfinite(R.op.Q)
    error('fan_geometry: 1D 模型无工况点, 请先调 fan_params (fan_sweep 选型)');
end

% ---- 叶片造型 (单一真源: fan_blade_arc) ----
r1 = P.imp.D1/2;  r2 = P.imp.D2/2;  r0eye = P.imp.D0/2;
beta1A = R.tri.beta1A;  beta2A = R.tri.beta2A;
[phiBl, Cb, Rab, Lb, thb] = fan_blade_arc(r1, r2, beta1A, beta2A);
tS  = P.imp.shroudT;
tW  = 0.008;                          % 蜗壳壁/侧板厚 (m)

% ---- 蜗壳型线 (阿基米德, 等周速设计) ----
c2u  = R.tri.c2u;
c_th = P.vol.kv * c2u;
B    = P.vol.B;
aSp  = R.op.Q / (2*pi*B*c_th);        % 蜗壳增长率 (m/rad)
rTongue = P.vol.rTongue;
rEnd = rTongue + aSp*2*pi;            % 蜗壳终端半径
Rvol = rEnd + tW;                     % 蜗壳外廓半径

% ---- 轴向布局 (m) ----
Lx = struct();
Lx.xBell   = [0, 0.05];               % 钟形口 (收缩 r: Rbell -> r0eye)
Lx.Rbell   = r0eye + 0.022;
Lx.xEye    = [0.05, 0.09];            % 吸力眼圆筒
Lx.xRear   = [0.09, 0.09+tS];         % 后盘
Lx.xBlade  = [0.09+tS, 0.09+tS+P.imp.b2];   % 叶片
Lx.xFront  = [Lx.xBlade(2), Lx.xBlade(2)+tS];% 前盘
Lx.xSideUp = [0.09-tS, 0.09];         % 上游侧板 (贴后盘, 带眼孔)
Lx.xSideDn = [Lx.xFront(2), Lx.xFront(2)+tS]; % 下游侧板 (带 W_out×B 出口方孔)
Lx.xVol    = [Lx.xSideUp(1), Lx.xSideDn(2)];
Lx.xStub   = [Lx.xSideDn(2), Lx.xSideDn(2)+0.15];  % 出口短管 (等截面)
Lx.xFlange = Lx.xStub(2);
Lx.xDiff   = [Lx.xFlange, Lx.xFlange+0.50];        % 前置扩散段 (与 tunnel_params.P.fd 一致)
Lx.xSettle = [Lx.xDiff(2), Lx.xDiff(2)+0.30];      % 稳定段短管 (GPU 出口侧)
Lx.W_out = P.vol.W_out;  Lx.H_out = B;
Lx.W_sett = 0.735;  Lx.H_sett = 0.490;             % 稳定段截面 (WindTunnel3D balanced)
try   % 风洞项目在场时取真实值 (单一真源)
    if ~any(strcmp(P.load.path, path)), addpath(P.load.path); end   % 幂等 addpath
    Tt = tunnel_params();
    Lx.W_sett = Tt.ct.W_in;  Lx.H_sett = Tt.ct.H_in;
    Lx.xDiff  = [Lx.xFlange, Lx.xFlange + Tt.fd.L];
    Lx.fdCurve = 'quintic';
catch
    Lx.fdCurve = 'linear';
end
Lx.xSettle = [Lx.xDiff(2), Lx.xDiff(2)+0.30];      % 稳定段短管 (GPU 出口侧)
Lx.wallT = tW;

% ---- 打包 ----
G.P = P;  G.R = R;
G.blade = struct('r1',r1,'r2',r2,'beta1A',beta1A,'beta2A',beta2A, ...
    'phi',phiBl,'C',Cb,'Ra',Rab,'Lb',Lb,'th',thb,'tB',P.imp.tB,'Z',P.imp.Z);
G.vol = struct('c2u',c2u,'c_th',c_th,'a',aSp,'rTongue',rTongue,'rEnd',rEnd, ...
    'Rvol',Rvol,'B',B,'tW',tW,'kv',P.vol.kv);
G.Lx = Lx;
G.r0eye = r0eye;
G.mesh = [];

% ---- 一致性检查 ----
if abs(P.vol.W_out - P.fd.W_in) > 1e-6 || abs(B - P.fd.H_in) > 1e-6
    fprintf('[fan_geometry] [!] 出口法兰 %.0fx%.0f 与 fan_params.fd %.0fx%.0f 不一致, 以蜗壳为准\n', ...
        P.vol.W_out*1e3, B*1e3, P.fd.W_in*1e3, P.fd.H_in*1e3);
end
fprintf('[fan_geometry] 叶片: 包角 %.0f° 弧长 %.0fmm 弦比 Ra=%.0fmm | 蜗壳: c_th=%.1f m/s a=%.1fmm/rad rEnd=%.0fmm\n', ...
    phiBl, Lb*1e3, Rab*1e3, c_th, aSp*1e3, rEnd*1e3);
fprintf('[fan_geometry] 轴向总长 %.0fmm (法兰在 x=%.0fmm) | 蜗壳外廓 D=%.0fmm | 出口 %.0fx%.0fmm\n', ...
    Lx.xSettle(2)*1e3, Lx.xFlange*1e3, 2*Rvol*1e3, Lx.W_out*1e3, Lx.H_out*1e3);
end
