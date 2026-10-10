%% export_fluidx3d_fan — 风机驱动全涵道 GPU 求解衔接器 (WindTunnel3D × FanSim3D 耦合)
% 风源不再是理想来流: 旋转叶轮(冻结转子动壁)实际激发风场, 链路
%   大气(+y 域面) → 增压箱 → 钟形口 → 吸力眼 (气流沿 -y 轴向进入)
%   → 叶轮 (绕 y 轴旋转, 旋转平面 = xoz, 前弯多叶)
%   → 蜗壳 (xz 平面阿基米德螺旋) → 底部切向出口沿 +x 喷出 → 前置扩散段(整流网)
%   → 稳定段 → 收缩段 → 试验段 → 自适应背压出口
% 布局定向 (2026-09-30 曼波定, 修复旧版风机与风洞同轴内联的本质问题):
%   风洞工作方向 = x | 抽风口面朝 +y (气体沿 -y 被抽入) | 叶轮旋转平面 = xoz
%   | 蜗壳切向出口对准 +x (旋转切线方向) 喷入风洞
% 旋向配对: 螺旋沿 Θ 逆时针生长 ↔ omega 取负 (绕 +y), 叶轮底部切向速度 = +x;
%   WT_SPIN=-1 时整机绕 z=0 镜像 (出口改顶部, 螺旋顺时针, omega 取正), 配对关系不变
% 产物与标准管线同名兼容: windtunnel.raw / gpu_meta.mat / vtk\ / forces.csv / fan_diag.csv
%   → explore_gpu / gpu_mov / gpu_report 直接可用 (meta.x0 + meta.Gext 扩展几何)
% 标记: raw=2 叶轮(TYPE_S|TYPE_X 动壁) / raw=3 测试物(TYPE_S|TYPE_Y, forces.csv)
% 开关: WT_QUICK=1 冒烟 | WT_DX=晶格 | WT_STEPS=步数 | WT_XTRIM=1 风机域(隧道折入背压)
%       WT_SPIN=-1 反转(整机 z 镜像) | WT_OBJ=sphere/cylinder/none
clear; clc;
% 套件相对定位: FanSim3D 与本项目同级 (<suite>/FanSim3D); 环境变量 FAN3D_DIR 可覆盖
RT = fileparts(mfilename('fullpath'));
FAN = getenv('FAN3D_DIR');
if isempty(FAN), FAN = fullfile(RT, '..', 'FanSim3D'); end
addpath(FAN);
assert(isfile(fullfile(FAN,'fan_params.m')), '未找到 FanSim3D (尝试 %s) — 请确认套件目录布局或设 FAN3D_DIR', FAN);
P = tunnel_params();
Pf = fan_params();
if ~isempty(getenv('WT_OBJ')), Pf.obj = getenv('WT_OBJ'); else, Pf.obj = 'none'; end
G  = tunnel_geometry(P);
Gf = fan_geometry(Pf);
Rf = fan_performance(Pf, [], [], [], true);
if ~isfinite(Rf.op.Q), error('风机 1D 无工况点 — 先查 fan_params'); end
Lf = Rf.load;

%% ---- GPU 档位 ----
GPU = 'full';
if ~isempty(getenv('WT_QUICK')), GPU = 'smoke'; end
switch GPU
    case 'smoke', dxg = 0.006; nSteps = 4000;  nSnap = 500;
    otherwise,    dxg = 0.004; nSteps = 15000; nSnap = 750;
end
dxEnv = getenv('WT_DX');
if ~isempty(dxEnv) && isfinite(str2double(dxEnv))
    dxg = min(0.01, max(0.0005, str2double(dxEnv)));
end
stEnv = getenv('WT_STEPS');
if ~isempty(stEnv) && isfinite(str2double(stEnv))
    nSteps = round(str2double(stEnv));
    fprintf('[WT_STEPS] 步数覆盖: %d\n', nSteps);
end
nDiag = 100;  nAvgStart = floor(nSteps/2);
% 叶轮 CFD 加厚: 叶片厚 < 2 格时泵送被黏性泄漏扼杀 (打印几何不受影响)
Gf.blade.tB = max(Gf.blade.tB, 1.2*dxg);
spinS = 1;
if ~isempty(getenv('WT_SPIN')), spinS = sign(str2double(getenv('WT_SPIN'))); end
% 快照 VTK 档位: 'u' (默认) | 'all' (rho+F; F 仅测试物存在时有意义) — 详见下方快照块注释
vtkAll = ~isempty(getenv('WT_VTK')) && strcmpi(getenv('WT_VTK'), 'all');

%% ---- 风机定向布局 (局部 x_l=贯流轴 -> 全局 -y; 截面极坐标 -> 全局 xoz 平面) ----
Lx = Gf.Lx;  vf = Gf.vol;  bl = Gf.blade;
yB0 = (Lx.xSideUp(2) + Lx.xSideDn(1))/2;   % 蜗壳腔 y 向中心基准 (腔体对中 y=0)
yL  = @(xl) yB0 - xl;                       % 局部贯流坐标 -> 全局 y (气流 -y)
WcavH = (Lx.xSideDn(1) - Lx.xSideUp(2))/2;  % 蜗壳腔半宽 (y 向): 腔带 yL(xSideUp(2))..yL(xSideDn(1)) 对中 0
THt   = deg2rad(300);                       % 舌部角 (Theta 系; Theta = mod(spinS*phi, 2pi))
rSb   = vf.rTongue + vf.a*deg2rad(330);     % 出口扇区壁半径 (Theta=270°)
zF    = spinS*(rSb + vf.rTongue)/2;         % 蜗壳心高度: 切向出口 z 带对中风洞轴线
xC    = -(vf.Rvol + 0.03);                  % 蜗壳心 x (整体位于风洞上游 x<0)
xE    = xC + vf.Rvol;                       % 蜗壳 +x 边缘 (此后出口短管渐扩)
if spinS > 0, zDLo = zF - rSb; zDHi = zF - vf.rTongue;   % 底部切向出口
else,         zDLo = zF + vf.rTongue; zDHi = zF + rSb;  % 镜像: 顶部切向出口
end

%% ---- 几何与域 ----
xDom1 = G.xSec(4) + 0.10;
xTrim = ~isempty(getenv('WT_XTRIM'));
if xTrim
    xDom1 = 0.02;
    fprintf('[WT_XTRIM] 风机域模式: 隧道全部折入背压 (定量测风机工作点)\n');
end
xDom0 = xC - vf.Rvol - 0.02;
yMin  = -(max(G.a) + 0.025);                % 覆盖稳定段全宽
yMax  = yB0 + 0.30;                         % 增压箱 +y 伸到域面 (大气盖)
zLoLim = -(max(G.b) + 0.025);
zHiLim =  (max(G.b) + 0.025);
if spinS > 0, zHiLim = max(zHiLim, zF + vf.Rvol + 0.015);
else,         zLoLim = min(zLoLim, zF - vf.Rvol - 0.015);
end
nx = round((xDom1-xDom0)/dxg);
ny = ceil((yMax-yMin)/dxg);
nz = ceil((zHiLim-zLoLim)/dxg);
xs = xDom0 + ((1:nx)-0.5)'*dxg;
yc = yMin + ((1:ny)-0.5)*dxg;
zc = zLoLim + ((1:nz)-0.5)'*dxg;
X3 = reshape(xs, nx,1,1);
Y3 = reshape(yc, 1,ny,1);
Z3 = reshape(zc, 1,1,nz);
RCxz = sqrt((X3-xC).^2 + (Z3-zF).^2);                 % 叶轮极坐标半径 (xoz 平面)
THg  = mod(spinS*atan2(Z3-zF, X3-xC), 2*pi);          % 叶轮极坐标角 (镜像随 spinS)
rSpiral = vf.rTongue + vf.a*mod(THg - THt, 2*pi);     % 阿基米德蜗壁 (沿旋转方向生长)

% ---- 隧道段型线 (fd 折在稳定段内, x>=0 不变) ----
aT = reshape(interp1(G.x, G.a, max(xs, 0), 'pchip'), [], 1, 1);
bT = reshape(interp1(G.x, G.b, max(xs, 0), 'pchip'), [], 1, 1);
aT(xs < 0) = -1;  bT(xs < 0) = -1;
inTunnel = (X3 >= 0) & (abs(Y3) < aT) & (abs(Z3) < bT);

% ---- 风机段 CSG (进气链沿 -y, 蜗壳螺旋在 xoz) ----
inPlenum = (Y3 >= yB0) & (abs(X3-xC) < 0.30) & (abs(Z3-zF) < 0.30);   % 增压箱 (+y 大气盖)
inBell   = (Y3 >= yL(Lx.xBell(2))) & (Y3 < yB0) & ...
           (RCxz < Lx.Rbell + (Gf.r0eye-Lx.Rbell)*(yB0-Y3)/Lx.xBell(2)); % 钟形口收缩 (-y 向)
inEye    = (Y3 >= yL(Lx.xEye(2))) & (Y3 < yL(Lx.xBell(2))) & (RCxz < Gf.r0eye); % 吸力眼
inCavity = (Y3 >= yL(Lx.xSideDn(1))) & (Y3 < yL(Lx.xSideUp(2))) & (RCxz < rSpiral); % 蜗壳腔
% 切向出口: 蜗壳底部 (spinS>0) 扇形开孔穿透蜗壁, 沿 +x 渐扩对接前置扩散段法兰
sW = min(max((X3 - xE)/(0 - xE), 0), 1);                  % 蜗壳外缘->法兰 渐扩因子
wYd = WcavH + (P.fd.W_in/2 - WcavH)*sW;
zDLoE = zDLo + (-P.fd.H_in/2 - zDLo)*sW;
zDHiE = zDHi + ( P.fd.H_in/2 - zDHi)*sW;
inDuct = (X3 >= xC) & (X3 < 0) & (abs(Y3) < wYd) & (Z3 > zDLoE) & (Z3 < zDHiE);

% ---- 叶轮 + 盘 + 毂 (旋转平面 xoz, 沿 y 拉伸) ----
blade2D = fan_blade_mask(Gf.blade, dxg, X3, Z3, RCxz, THg);
slBlade = (Y3 >= yL(Lx.xBlade(2))) & (Y3 < yL(Lx.xBlade(1)));
slRear  = (Y3 >= yL(Lx.xRear(2)))  & (Y3 < yL(Lx.xRear(1)));
slFront = (Y3 >= yL(Lx.xFront(2))) & (Y3 < yL(Lx.xFront(1)));
rot0  = slBlade & blade2D;   % blade2D=(nx,1,nz) 截面掩码 × y 向叶宽波段 -> (nx,ny,nz)
mDisc = (slRear  & (RCxz >= Gf.r0eye) & (RCxz <= bl.r2)) | ...   % 后盘 (进气侧)
        (slFront & (RCxz >= Gf.r0eye) & (RCxz <= bl.r2));        % 前盘
mHub  = (Y3 > yL(Lx.xSideDn(2))) & (Y3 <= yL(Lx.xRear(1))) & (RCxz < bl.r1); % 电机毂: 封轴向短路

% ---- 旋转模式 (WT_ROT=1: 瞬态旋转叶轮, 重体素化; 移植自 FanSim3D) ----
% frozen rotor (默认) 只给叶片格动壁速度、几何不转 → 流场准稳态;
% 旋转模式每 nRotEvery 步把叶片真实转一个角度增量, 扫掠包络内逐格重判流体/叶片
ROTATE = ~isempty(getenv('WT_ROT')) && strcmpi(getenv('WT_ROT'), '1');
if ~ROTATE, rotList = zeros(0,3); end   % 占位 (冻结转子不导出扫掠包络)
snEnv = getenv('WT_SNAPV');   % 快照间隔覆盖 (旋转可视化用小值; 避开 explore_gpu 的 WT_SNAP)
if ROTATE && ~isempty(snEnv) && str2double(snEnv) >= 50
    nSnap = round(str2double(snEnv));
    fprintf('[WT_ROT] 旋转模式: 快照间隔 %d 步 (%.1f°/帧)\n', nSnap, 2*pi*Pf.imp.rpm/60*(0.08*dxg/25)*nSnap*180/pi);
end
dC0 = norm(bl.C);  tbh = max(bl.tB/2, 0.6*dxg);
envR = [max(0, bl.r1 - 3*dxg), bl.r2 + 3*dxg];   % 叶片弧带径向跨度=[r1,r2], 包络收紧 (原 FanSim3D 公式算出 0~663mm 全域, 4.8M 格纯浪费)
wrapDegR = mod(bl.th(2)-bl.th(1), 360);
pitchDegR = 360/bl.Z;
nRotEvery = 10;    % 每 10 步重体素化 (转 ~0.74°): 界面增量最小化 — 硬翻转是 LBM 稳定性关键
if ROTATE
    bandY = abs(Y3 - mean([mean(yL(Lx.xBlade))])) <= (Lx.xBlade(2)-Lx.xBlade(1))/2 + 0.5*dxg;
    env = reshape(bandY, 1, ny, 1) & RCxz >= envR(1) & RCxz <= envR(2);
    iiR = find(env);
    mIdxR = floor((iiR(:)-1)/nx);  izR = floor(mIdxR/ny) + 1;  iR1 = mod(iiR(:)-1, nx) + 1;
    rrR = sqrt((xs(iR1) - xC).^2 + (zc(izR) - zF).^2);
    ppR = mod(atan2(zc(izR) - zF, xs(iR1) - xC), 2*pi);
    rotList = [double(iiR(:)) - 1, rrR, ppR];   % (n 0-based, r m, phi 物理角 rad)
    fprintf('[WT_ROT] 扫掠包络 %d 格 (环带 r %.0f~%.0fmm × 叶宽) | 每 %d 步转 %.2f°\n', ...
        size(rotList,1), envR(1)*1e3, envR(2)*1e3, nRotEvery, 2*pi*Pf.imp.rpm/60*(0.08*dxg/25)*nRotEvery*180/pi);
end

% ---- 整流网 (2 格棋盘穿孔板) ----
chk2 = @(xslab) abs(X3 - xslab) < dxg/2 & (mod(floor((Y3-yc(1))/(2*dxg)) + floor((Z3-zc(1))/(2*dxg)), 2) == 0);
scrFd = chk2(P.fd.L);

% ---- 测试物 ----
obj = [];
if ~strcmpi(Pf.obj, 'none')
    obj = false(nx, ny, nz);
    cTs = (G.xSec(3)+G.xSec(4))/2;
    switch lower(Pf.obj)
        case 'sphere'
            r = 0.030;  obj = (X3-cTs).^2 + Y3.^2 + Z3.^2 <= r^2;
        case 'cylinder'
            r = 0.030;  obj = (X3-cTs).^2 + Z3.^2 <= r^2;
        otherwise
            fprintf('[WT_OBJ] 耦合模式支持 sphere/cylinder/none\n');
    end
    obj = obj & inTunnel;
    fprintf('测试物: %s (Ø%.0fmm)\n', Pf.obj, 2*r*1e3);
end

% ---- 流体域合成 (掩码即用即清: 全域 logical 各 ~50MB, 峰值内存砍半) ----
fluid = inPlenum | inBell | inEye | inCavity | inDuct | inTunnel;
clear inPlenum inBell inEye inCavity inDuct
fluid = fluid & ~rot0 & ~mDisc & ~mHub & ~scrFd;
if ~isempty(obj), fluid = fluid & ~obj; end
nObj = nnz(obj);
iiObj = find(obj);
clear mDisc mHub scrFd obj
solidW = ~fluid;

fprintf('耦合域: %dx%dx%d = %.1fM 格 (dx=%.1fmm) | x ∈ [%.2f, %.2f] m\n', ...
    nx, ny, nz, nx*ny*nz/1e6, dxg*1e3, xDom0, xDom1);
fprintf('叶片 %d 格 | 测试物 %d 格\n', nnz(rot0), nObj);
fprintf('朝向自检: 抽风口 +y (大气盖 y=%.3fm) | 叶轮轴 y (心 x=%.3f z=%.3f) | 切向出口 +x (z 带 %.3f~%.3f)\n', ...
    yMax, xC, zF, zDLo, zDHi);

% ---- 连通性/半径自检 ----
iiB = find(rot0);
mIdx = floor((iiB(:)-1)/nx);
iB  = mod(iiB(:)-1, nx) + 1;      % x 分量索引
iyB = mod(mIdx, ny) + 1;
izB = floor(mIdx/ny) + 1;
rrB = sqrt((xs(iB) - xC).^2 + (zc(izB) - zF).^2);  rrB = rrB(:);
assert(isempty(rrB) || (min(rrB) > 0.85*bl.r1 && max(rrB) < 1.1*bl.r2), ...
    '叶片格半径越界: [%.3f, %.3f] 应在叶轮环带内', min(rrB), max(rrB));
faceY = fluid(:,end,:);  faceX = fluid(end,:,:);   % +y 域面 / +x 域面
assert(any(faceY(:)), '+y 域面无增压箱流体 (大气盖失效)');
assert(any(faceX(:)), '+x 域面无出口流体 (背压出口失效)');
iDuct = find(fluid & X3 < -dxg/2 & X3 >= xC-2*dxg & abs(Y3) < WcavH & Z3 > min(zDLo,zDHi) & Z3 < max(zDLo,zDHi), 1);
assert(~isempty(iDuct), '切向出口/蜗壳腔不连通');

%% ---- 写产物 ----
raw = zeros(nx,ny,nz,'uint8');
raw(solidW) = 1;
raw(rot0)   = 2;
if ~isempty(iiObj), raw(iiObj) = 3; end
clear solidW fluid iiObj
fid = fopen('windtunnel.raw','wb');  fwrite(fid, raw, 'uint8');  fclose(fid);
fprintf('windtunnel.raw 已写 (%.1f MB)\n', numel(raw)/2^20);

ppB = mod(atan2(zc(izB) - zF, xs(iB) - xC), 2*pi);  ppB = ppB(:);   % 物理角 (xoz 系, 蜗壳心系)
bladeList = [double(iiB(:)) - 1, rrB, ppB];
fid = fopen('fan_blade.bin','wb');
fwrite(fid, [bladeList(:,1), bladeList(:,2)/dxg, bladeList(:,3)]', 'float32');
fclose(fid);
fprintf('fan_blade.bin 已写 (%d 叶片格)\n', size(bladeList,1));
fid = fopen('fan_rotor.bin','wb');
RLs = zeros(3, size(rotList,1), 'single');
RLs(1,:) = rotList(:,1);  RLs(2,:) = rotList(:,2)/dxg;  RLs(3,:) = rotList(:,3);
fwrite(fid, RLs, 'float32');  fclose(fid);  clear RLs
fprintf('fan_rotor.bin 已写 (%d 扫掠包络格%s)\n', size(rotList,1), string(ROTATE).replace("true", "").replace("false", ", 冻结转子空占位"));

%% ---- 单位换算 ----
uLx  = 0.08;
Vref = P.V_test;
Sv   = Vref/uLx;
nuL  = max(uLx*(P.ts.W/dxg)/P.lbm.Re, 0.010);   % [3mm 修复] 下限从 0.005 提到 0.010: 3mm 时原公式给 0.0067 → τ=0.52 贴近不稳定边界 0.5; 下限 0.010 → τ=0.53 安全 (FanSim3D 原生同值)
nuEnv = getenv('WT_NU');   % 晶格黏度覆盖
if ~isempty(nuEnv) && isfinite(str2double(nuEnv)) && str2double(nuEnv) > 0
    nuL = str2double(nuEnv);
    fprintf('[WT_NU] 晶格黏度覆盖: %.4f\n', nuL);
else
    % [2026-10-02 修复] 风机耦合解的 Re 特征长度须用风机口径 (FanSim3D 同款), 不是试验段宽:
    %   ν 由叶轮间隙流决定 — 用试验段宽算出的 ν 在 3mm 时低至 0.0067, 间隙流落入数值失效区,
    %   风机零泵送 (Q=-0.019); 改用蜗壳特征尺寸 Rvol 后 3mm 得 ν=0.0100, 泵送恢复 (+1.59)。
    Lfan = max(Gf.vol.Rvol, P.ct.W_in);        % 风机特征尺寸: 蜗壳外廓半径
    nuL  = max(uLx*Lfan/dxg/1200, 0.010);      % Re=1200 同口径, 下限 0.010 保间隙流可解
    fprintf('[ν] 风机口径晶格黏度: %.4f (L=%.3fm, 间隙流可解下限 0.010)\n', nuL, Lfan);
end
dtP  = uLx*dxg/Vref;
dxP  = dxg;
rhoF = 1.204;
rpmV = Pf.imp.rpm;
rpmEnv = getenv('WT_RPM');   % 演示降速 (旋转模式调稳用; 定量仍以 fan_params.rpm 为准)
if ~isempty(rpmEnv) && isfinite(str2double(rpmEnv)) && str2double(rpmEnv) > 0
    rpmV = str2double(rpmEnv);
    fprintf('[WT_RPM] 转速覆盖: %.0f rpm (演示档)\n', rpmV);
end
omega = -spinS*2*pi*rpmV/60;   % 绕 +y: 螺旋逆时针生长配 omega<0 (spinS=-1 整机镜像后取正)
omegaLx = omega*dtP;
Krear = Lf.Krear;
Kfold = (0.22 + 2*0.80)*rhoF/(2*(P.ct.W_in*P.ct.H_in)^2);
if xTrim, Kload = Lf.K + Kfold; else, Kload = Krear + Kfold; end
if ~isempty(getenv('WT_K0')), Kload = 0; fprintf('[WT_K0] 负载清零 (空载泵送甄别)\n'); end
dp2drho = 3/(rhoF*Sv^2);
Fscale  = rhoF*dxP^4/dtP^2;
if xTrim, xmLx = nx - 4; else, xmLx = round((G.xSec(3) + 2*dxg - xDom0)/dxg); end
xcL = (xC - xDom0)/dxg - 0.5;            % 转矩枢轴 (晶格系)
zcL = (zF - zLoLim)/dxg - 0.5;
fprintf('晶格: nu=%.4f | dt=%.2es | Kload=%.0f Pa/(m³/s)² | omega=%.3f rad/s (绕+y)\n', nuL, dtP, Kload, omega);

%% ---- 扩展几何 (包络: 严格递增 x + 正 a/b, 供 explore_gpu 轮廓) ----
xFe = [xDom0, xC-0.31, xC+0.31, xE+0.001, -0.001];
aFe = [0.02,  0.30,    0.30,    0.15,     0.15];
bFe = [0.02,  0.52,    0.52,    0.14,     0.14];
Gext = struct('x', [xFe(:); G.x + 0*G.x], 'a', [aFe(:); G.a], 'b', [bFe(:); G.b]);
Gext.xSec = G.xSec;
Gext.secNames = [{'风机(增压箱-y进气→叶轮y轴→蜗壳切向+x)'}, G.secNames];

%% ---- gpu_meta.mat ----
Aref = pi*0.03^2/4;
meta = struct('dx',dxP, 'dt',dtP, 'velScale',Sv, 'nx',nx, 'ny',ny, 'nz',nz, ...
    'V_test',P.V_test, 'rho',rhoF, 'Aref',Aref, 'tSnap_lattice',nSnap, 'nSteps',nSteps, ...
    'ReLattice', round(uLx*(P.ts.W/dxg)/P.lbm.Re), 'objSize',0.030, 'objType',Pf.obj, ...
    'tuInlet',NaN, 'hasTimeAvg',true, 'objBBox',[0.06 0.06 0.06], 'blockage',0, ...
    'isFanCoupled',true, 'x0',xDom0, 'y0',yMin, 'z0',zLoLim, 'Gext',Gext, ...
    'impOmega',omega, 'impSpin',spinS, 'impCenterX',xC, 'impCenterZ',zF, 'rotate',ROTATE, ...
    'Krear',Krear, 'nAvgStart',nAvgStart, ...
    'opQ',Rf.op.Q, 'blade',Gf.blade, 'Lx',Lx, 'vol',Gf.vol, ...
    'fanLayout', struct('yB0',yB0,'xC',xC,'zF',zF,'xE',xE,'THt',THt, ...
        'yBell',yL(Lx.xBell),'yEye',yL(Lx.xEye),'ySideUp',yL(Lx.xSideUp), ...
        'yRear',yL(Lx.xRear),'yBlade',yL(Lx.xBlade),'yFront',yL(Lx.xFront), ...
        'ySideDn',yL(Lx.xSideDn),'zDuct',[zDLo zDHi],'WcavH',WcavH,'rSb',rSb));
save('gpu_meta.mat', 'meta');
fprintf('gpu_meta.mat 已写\n');

%% ---- 生成 setup_windtunnel_fan.cpp ----
NL = sprintf('\n');
C = {};
C{end+1} = '#include "setup.hpp"';
C{end+1} = '#include <vector>';
C{end+1} = 'void writeVTKLike(const char* fname, const float* data, const uint ncomp, const uint Nx, const uint Ny, const uint Nz);';
C{end+1} = ['void main_setup() { // auto-generated (dx=' num2str(dxP*1e3,'%.1f') 'mm) 风机侧置: 进气+y/叶轮绕y/蜗壳切向+x'];
C{end+1} = ['	LBM lbm(' num2str(nx) 'u, ' num2str(ny) 'u, ' num2str(nz) 'u, ' num2str(nuL,'%.6f') 'f);'];
C{end+1} = '	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz();';
C{end+1} = '	uchar* raw = new uchar[lbm.get_N()];';
C{end+1} = '	FILE* f = fopen("windtunnel.raw","rb");';
C{end+1} = '	if(!f) { print_error("windtunnel.raw not found"); exit(1); }';
C{end+1} = '	if(fread(raw, 1, lbm.get_N(), f)!=lbm.get_N()) { print_error("raw size mismatch"); exit(1); }';
C{end+1} = '	fclose(f);';
C{end+1} = '	parallel_for(lbm.get_N(), [&](ulong n) { // 0流体 1壁 2叶轮(S|X) 3测试物(S|Y)';
C{end+1} = '		lbm.flags[n] = raw[n]==0u ? 0u : (raw[n]==2u ? (TYPE_S|TYPE_X) : (raw[n]==3u ? (TYPE_S|TYPE_Y) : TYPE_S));';
C{end+1} = '	});';
C{end+1} = '	// 增压箱大气盖(+y 域面, 气体沿 -y 抽入) / 出口自适应背压+外流速度 (仅流体格)';
C{end+1} = '	parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);';
C{end+1} = '		if(y==Ny-1u && raw[n]==0u) { lbm.flags[n]=TYPE_E; lbm.rho[n]=1.0f; lbm.u.x[n]=0.0f; lbm.u.y[n]=0.0f; lbm.u.z[n]=0.0f; }';
C{end+1} = '		if(x==Nx-1u && raw[n]==0u) { lbm.flags[n]=TYPE_E; lbm.rho[n]=1.0f; lbm.u.x[n]=0.0f; lbm.u.y[n]=0.0f; lbm.u.z[n]=0.0f; }';
C{end+1} = '	});';
C{end+1} = ['	const float omegaFull = ' num2str(omegaLx,'%.7f') 'f; // 绕 +y, 符号与蜗壳螺旋配对'];
C{end+1} = ['	const float rotPx = ' num2str(xcL,'%.4f') 'f, rotPz = ' num2str(zcL,'%.4f') 'f; // 转矩枢轴 (晶格系, xoz 平面)'];
C{end+1} = ['	const ulong nBlade = ' num2str(size(bladeList,1)) 'u;'];
C{end+1} = '	float* bladeData = new float[3u*nBlade];';
C{end+1} = '	f=fopen("fan_blade.bin","rb");';
C{end+1} = '	if(!f) { print_error("fan_blade.bin not found"); exit(1); }';
C{end+1} = '	if(fread(bladeData, 4u, 3u*nBlade, f)!=3u*nBlade) { print_error("blade bin mismatch"); exit(1); }';
C{end+1} = '	fclose(f);';
C{end+1} = ['	const bool ROTATE = ' num2str(double(ROTATE)) 'u==1u; // WT_ROT=1: 瞬态旋转叶轮 (重体素化); false=冻结转子'];
C{end+1} = ['	const float C0x=' num2str(bl.C(1)/dxg,'%.4f') 'f, C0z=' num2str(bl.C(2)/dxg,'%.4f') 'f, RaLx=' num2str(bl.Ra/dxg,'%.4f') 'f, tBh=' num2str(tbh/dxg,'%.4f') 'f; // 叶片0 圆心/半径/半厚 (晶格)'];
C{end+1} = ['	const float th1=' num2str(deg2rad(bl.th(1)),'%.7f') 'f, wrap=' num2str(deg2rad(wrapDegR),'%.7f') 'f, pitch=' num2str(deg2rad(pitchDegR),'%.7f') 'f; const uint mMax=' num2str(ceil((wrapDegR+pitchDegR+4)/pitchDegR)) 'u;'];
C{end+1} = ['	const float spinS = ' num2str(spinS) '.0f; const uint nRotEvery = ' num2str(nRotEvery) 'u;'];
C{end+1} = ['	const ulong nRotList = ' num2str(size(rotList,1)) 'u;'];
C{end+1} = '	float* rotData = nullptr;  uchar* rotState = nullptr;';
C{end+1} = '	if(ROTATE) {';
C{end+1} = '		rotData = new float[3u*nRotList];';
C{end+1} = '		f=fopen("fan_rotor.bin","rb");';
C{end+1} = '		if(!f) { print_error("fan_rotor.bin not found"); exit(1); }';
C{end+1} = '		if(fread(rotData, 4u, 3u*nRotList, f)!=3u*nRotList) { print_error("rotor bin mismatch"); exit(1); }';
C{end+1} = '		fclose(f);';
C{end+1} = '		rotState = new uchar[nRotList];';
C{end+1} = '	}';
C{end+1} = ['	const uint xm = ' num2str(xmLx) 'u; // 流量测量面'];
C{end+1} = '	std::vector<ulong> meas;';
C{end+1} = '	for(uint z=0u; z<Nz; z++) for(uint y=0u; y<Ny; y++) { const ulong n=((ulong)z*Ny+(ulong)y)*Nx+(ulong)xm;';
C{end+1} = '		if(raw[n]==0u) meas.push_back(n); }';
C{end+1} = '	// 出口平面流体格 (动量匹配外流边界: E 的 u 固定 0 会形成动量吸收墙憋死出流)';
C{end+1} = '	std::vector<ulong> outp;';
C{end+1} = '	for(uint z=0u; z<Nz; z++) for(uint y=0u; y<Ny; y++) { const ulong n=((ulong)z*Ny+(ulong)y)*Nx+(ulong)(Nx-1u);';
C{end+1} = '		if(raw[n]==0u) outp.push_back(n); }';
C{end+1} = ['	const float velPhys = ' num2str(Sv,'%.4f') 'f;'];
% [初始正向流] 全域零初值会锁死在反向渗流假稳态 (3mm 数值耗散小, 微弱回流存活; 4mm 数值黏性天然抑制)。
%   初始化时给全场流体格 ux=+2 m/s, 推系统入正向分支。WT_INITFLOW=0 可关 (对照实验)。
initFlow = ~isempty(getenv('WT_INITFLOW')) && strcmpi(getenv('WT_INITFLOW'), '0');
if ~initFlow
    C{end+1} = ['	const float uxInit = ' num2str(2.0/Sv,'%.6f') 'f; // [初始正向流] +2 m/s 推入正向分支 (devlog 2026-10-02)'];
    C{end+1} = '	parallel_for(lbm.get_N(), [&](ulong n) { if(raw[n]==0u) lbm.u.x[n] = uxInit; });';
else
    C{end+1} = '	// WT_INITFLOW=0: 零初值对照';
end
C{end+1} = ['	const float dxPhys = ' num2str(dxP,'%.6f') 'f;'];
C{end+1} = '	const float Aout = (float)outp.size()*dxPhys*dxPhys;';
C{end+1} = ['	const float Kload = ' num2str(Kload,'%.4f') 'f;'];
C{end+1} = ['	const float dp2drho = ' num2str(dp2drho,'%.7f') 'f;'];
C{end+1} = ['	const float Tscale = ' num2str(Fscale*dxP,'%.7f') 'f;'];
C{end+1} = ['	const float Fscale = ' num2str(Fscale,'%.6f') 'f;'];
C{end+1} = ['	const ulong nSteps = ' num2str(nSteps) 'u, nAvgStart = ' num2str(nAvgStart) 'u;'];
C{end+1} = ['	const uint nDiag = ' num2str(nDiag) 'u;'];
C{end+1} = '	float rhoOut = 1.0f;  float uxS = 0.0f;  float theta = 0.0f;  ulong tLastRot = 0ull; // theta: 叶轮累积转角 (Θ系, rad)';
C{end+1} = '	auto applyRotor = [&](const float omega) { // 冻结转子动壁 (绕 +y: u=(w*z, 0, -w*x), 每次都写, 对齐官方动边界模式)';
C{end+1} = '		lbm.u.read_from_device();';
C{end+1} = '		parallel_for(nBlade, [&](ulong i) {';
C{end+1} = '			const ulong n = (ulong)bladeData[3u*i];';
C{end+1} = '			if(lbm.flags[n]==(TYPE_S|TYPE_X)) {';
C{end+1} = '				const float xr = bladeData[3u*i+1u]*cosf(bladeData[3u*i+2u]), zr = bladeData[3u*i+1u]*sinf(bladeData[3u*i+2u]); // 轴心系 (xoz 平面)';
C{end+1} = '				lbm.u.x[n]=omega*zr; lbm.u.y[n]=0.0f; lbm.u.z[n]=-omega*xr;';
C{end+1} = '			}';
C{end+1} = '		});';
C{end+1} = '		lbm.u.write_to_device();';
C{end+1} = '	};';
C{end+1} = '	auto bladeTest = [&](const float r, const float psiT, const float th) { // Θ系极角下是否在任一叶片内 (xoz 平面)';
C{end+1} = '		for(uint m=0u; m<mMax; m++) {';
C{end+1} = '			const float pm = fmodf(psiT - th, pitch) + (float)m*pitch;';
C{end+1} = '			const float x = r*cosf(pm), z = r*sinf(pm);';
C{end+1} = '			const float dd = sqrtf((x-C0x)*(x-C0x)+(z-C0z)*(z-C0z));';
C{end+1} = '			if(fabsf(dd-RaLx) <= tBh) {';
C{end+1} = '				const float a = fmodf(atan2f(z-C0z, x-C0x) - th1 + 6.2831853f, 6.2831853f);';
C{end+1} = '				if(a <= wrap) return true;';
C{end+1} = '			}';
C{end+1} = '		}';
C{end+1} = '		return false;';
C{end+1} = '	};';
C{end+1} = '	auto rotateUpdate = [&](const float omega) { // 重体素化: 扫掠包络内逐格翻转 叶片<->流体 (先读回再改)';
C{end+1} = '		lbm.flags.read_from_device();  lbm.u.read_from_device();  lbm.rho.read_from_device();';
C{end+1} = '		parallel_for(nRotList, [&](ulong i) {';
C{end+1} = '			const float r = rotData[3u*i+1u], phiP = rotData[3u*i+2u];';
C{end+1} = '			const ulong n = (ulong)rotData[3u*i];';
C{end+1} = '			const float psiT = fmodf(spinS*phiP + 25.1327412f, 6.2831853f); // 物理角 -> Θ系 (保正)';
C{end+1} = '			const bool inB = bladeTest(r, psiT, theta);';
C{end+1} = '			const uchar want = inB ? 1u : 0u;';
C{end+1} = '			if(want != rotState[i]) {';
C{end+1} = '				const float x = r*cosf(phiP), z = r*sinf(phiP);';
C{end+1} = '				lbm.flags[n] = want ? (TYPE_S|TYPE_X) : 0u;';
C{end+1} = '				if(want) { lbm.u.x[n]=omega*z; lbm.u.y[n]=0.0f; lbm.u.z[n]=-omega*x; } // 新叶片格: 动壁速度';
C{end+1} = '				else      { lbm.u.x[n]=0.0f;      lbm.u.y[n]=0.0f; lbm.u.z[n]=0.0f;      } // 新暴露流体格: 静止';
C{end+1} = '				// [修复] rho 保留原值不硬设 1.0 — 密度跳变是二次冲击源';
C{end+1} = '				rotState[i] = want;';
C{end+1} = '			}';
C{end+1} = '		});';
C{end+1} = '		lbm.flags.write_to_device();  lbm.u.write_to_device();  lbm.rho.write_to_device();';
C{end+1} = '		lbm.update_moving_boundaries();';
C{end+1} = '	};';
C{end+1} = ['	FILE* fc = fopen("fan_diag.csv","w"); fprintf(fc, "t,Q_m3s,torque_Nm,power_W,rhoOut' NL '"); fclose(fc);'];
C{end+1} = ['	FILE* fo = fopen("forces.csv","w"); fprintf(fo, "t_lattice,Fx,Fy,Fz' NL '"); fclose(fo);'];
C{end+1} = '	float* uSum = new float[3u*lbm.get_N()]();  float* u2Sum = new float[3u*lbm.get_N()]();';
C{end+1} = '	ulong nAvg = 0ull;  float Qphys = 0.0f;';
C{end+1} = '	lbm.run(0u); // 初始化';
C{end+1} = '	if(ROTATE) { parallel_for(nRotList, [&](ulong i) { rotState[i] = bladeTest(rotData[3u*i+1u], fmodf(spinS*rotData[3u*i+2u]+25.1327412f, 6.2831853f), 0.0f) ? 1u : 0u; }); }';
C{end+1} = '	applyRotor(omegaFull);';
C{end+1} = '	lbm.update_moving_boundaries();';
C{end+1} = ['	for(uint s=0u; s<nSteps; s+=' num2str(nSnap) 'u) {'];
C{end+1} = ['		for(uint ss=0u; ss<' num2str(nSnap) 'u; ss+=nDiag) {'];
C{end+1} = '			const float ramp = fminf(1.0f, (float)lbm.get_t()/(0.1f*(float)nSteps));';
C{end+1} = '			applyRotor(omegaFull*ramp);';
C{end+1} = '			if(ROTATE && lbm.get_t()>0ull && lbm.get_t()%nRotEvery==0ull) { theta += omegaFull*(float)(lbm.get_t()-tLastRot)*ramp; tLastRot = lbm.get_t(); rotateUpdate(omegaFull*ramp); } // 叶轮真实转动 (重体素化)';
C{end+1} = '			lbm.run(nDiag, nSteps);';
C{end+1} = '			lbm.u.read_from_device();';
C{end+1} = '			double qLx = 0.0;';
C{end+1} = '			for(ulong k=0ull; k<meas.size(); k++) qLx += (double)lbm.u.x[meas[k]];';
C{end+1} = '			Qphys = (float)qLx*velPhys*dxPhys*dxPhys;';
C{end+1} = '			const float dpLoad = Kload*Qphys*Qphys;';
C{end+1} = '			const float Keff = Kload*fminf(1.0f, (float)lbm.get_t()/(0.5f*(float)nSteps)); // [启动阀] 负载前 50% 步数线性爬升: 防低流量启动滑入喘振左支 (3mm 全域锁死根因)';
C{end+1} = '			rhoOut = fmaxf(0.97f, 1.0f - 0.5f*dp2drho*Keff*Qphys*Qphys);';
% [3mm 全域锁死终修] 出口速度不再由 Q 反馈驱动 (正反馈环路: Q 偶发为负 → uxS<0 → 出口吸入 → Q 更负
%   → 锁死 ±10.7 级反向流)。改为纯压力出口: E 的 u 恒 0, 只用 rho=背压驱动 —— 压差自然建立通流,
%   无反馈环路即无锁死。uxS/dpLoad 反馈仅保留在诊断输出 (不再写回边界)。
% [R3 实验] 背压爬升 (Keff 启动阀已在上方) + 有符号 uxS 反馈 —— 唯一未试组合
% 实验矩阵: 即开即用+钳位=−0.019 | 即开即用+有符号=−10.7暴冲 | 即开即用+纯压=−0.026 | 爬升+钳位=−10.7 | **爬升+有符号=R3**
uxSigned = true;
if uxSigned
    C{end+1} = '			uxS += 0.1f*(Qphys/fmaxf(Aout,1e-9f) - uxS);';
else
    C{end+1} = '			uxS = 0.0f; // 纯压力出口: u 恒 0 (反馈环路已拆除, 见上注释)';
end
C{end+1} = '			// 出口 E: rho=背压, u=外流速度 (动量匹配, 去除 u=0 动量吸收墙)';
C{end+1} = '			lbm.rho.read_from_device();';
C{end+1} = '			parallel_for((ulong)outp.size(), [&](ulong i) { const ulong n = outp[i];';
C{end+1} = '				lbm.rho[n] = rhoOut; lbm.u.x[n] = uxS; lbm.u.y[n] = 0.0f; lbm.u.z[n] = 0.0f; });';
C{end+1} = '			lbm.rho.write_to_device();  lbm.u.write_to_device();';
C{end+1} = '			lbm.update_force_field();';
C{end+1} = ['			const float3 TQ = lbm.object_torque(float3(rotPx, 0.0f, rotPz), TYPE_S|TYPE_X); // 绕 y 轴转矩'];
C{end+1} = '			const float tq = TQ.y*Tscale;';
C{end+1} = ['			fc = fopen("fan_diag.csv","a"); fprintf(fc, "%u,%.5f,%.4f,%.3f,%.5f' NL '", lbm.get_t(), Qphys, tq, tq*omegaFull, rhoOut); fclose(fc);'];
C{end+1} = '			const float3 FV = lbm.object_force(TYPE_S|TYPE_Y);';
C{end+1} = ['			fo = fopen("forces.csv","a"); fprintf(fo, "%u,%.6g,%.6g,%.6g' NL '", lbm.get_t(), FV.x*Fscale, FV.y*Fscale, FV.z*Fscale); fclose(fo);'];
C{end+1} = '			if(lbm.get_t()>=nAvgStart) {';
C{end+1} = '				parallel_for(lbm.get_N(), [&](ulong n) { const float ux=lbm.u.x[n], uy=lbm.u.y[n], uz=lbm.u.z[n];';
C{end+1} = '					uSum[0u*lbm.get_N()+n]+=ux;  uSum[1u*lbm.get_N()+n]+=uy;  uSum[2u*lbm.get_N()+n]+=uz;';
C{end+1} = '					u2Sum[0u*lbm.get_N()+n]+=ux*ux;  u2Sum[1u*lbm.get_N()+n]+=uy*uy;  u2Sum[2u*lbm.get_N()+n]+=uz*uz; });';
C{end+1} = '				nAvg++;';
C{end+1} = '			}';
C{end+1} = '		}';
C{end+1} = '		lbm.update_force_field(); // [2026-09-30 修复] 快照前不再调 update_fields(): 其镜像重构损坏, 主内核每步已维护真实镜像';
C{end+1} = '		lbm.u.write_device_to_vtk("vtk/u_"+to_string(lbm.get_t()));';
if vtkAll
    C{end+1} = '		lbm.rho.write_device_to_vtk("vtk/rho_"+to_string(lbm.get_t()));';
    if ~strcmpi(Pf.obj, 'none')
        C{end+1} = '		lbm.F.write_device_to_vtk("vtk/F_"+to_string(lbm.get_t()));';
    end
end
C{end+1} = '		print_info("snapshot t="+to_string(lbm.get_t())+" Q="+to_string(Qphys)+" m3/s rhoOut="+to_string(rhoOut));';
C{end+1} = '	}';
C{end+1} = '	if(nAvg>0ull) {';
C{end+1} = '		float* meanBuf = new float[3u*lbm.get_N()];  float* tuBuf = new float[lbm.get_N()];';
C{end+1} = '		for(ulong n=0ull; n<lbm.get_N(); n++) {';
C{end+1} = '			const float mx=uSum[0u*lbm.get_N()+n]/(float)nAvg, my=uSum[1u*lbm.get_N()+n]/(float)nAvg, mz=uSum[2u*lbm.get_N()+n]/(float)nAvg;';
C{end+1} = '			meanBuf[3u*n+0u]=mx;  meanBuf[3u*n+1u]=my;  meanBuf[3u*n+2u]=mz;';
C{end+1} = '			const float v2 = fmaxf(u2Sum[0u*lbm.get_N()+n]/(float)nAvg-mx*mx,0.0f)+fmaxf(u2Sum[1u*lbm.get_N()+n]/(float)nAvg-my*my,0.0f)+fmaxf(u2Sum[2u*lbm.get_N()+n]/(float)nAvg-mz*mz,0.0f);';
C{end+1} = ['			tuBuf[n] = 100.0f*sqrtf(v2)/' num2str(uLx,'%.5f') 'f;'];
C{end+1} = '		}';
C{end+1} = '		writeVTKLike("vtk/mean_u.vtk", meanBuf, 3u, Nx, Ny, Nz);';
C{end+1} = '		writeVTKLike("vtk/tu.vtk", tuBuf, 1u, Nx, Ny, Nz);';
C{end+1} = '		delete[] meanBuf;  delete[] tuBuf;';
C{end+1} = '	}';
C{end+1} = '	delete[] uSum;  delete[] u2Sum;  delete[] bladeData;  delete[] raw;';
C{end+1} = '	print_info("fan-coupled wind tunnel done");';
C{end+1} = '}';
C{end+1} = 'void writeVTKLike(const char* fname, const float* data, const uint ncomp, const uint Nx, const uint Ny, const uint Nz) {';
C{end+1} = '	FILE* f = fopen(fname, "wb");  if(!f) { print_error(std::string("cannot write ")+fname); return; }';
C{end+1} = '	char hdr[512];';
C{end+1} = ['	sprintf(hdr, "# vtk DataFile Version 3.0' NL 'WindTunnel3D FanCoupled' NL 'BINARY' NL 'DATASET STRUCTURED_POINTS' NL 'DIMENSIONS %u %u %u' NL 'ORIGIN %.8E %.8E %.8E' NL 'SPACING 1.00000000 1.00000000 1.00000000' NL 'POINT_DATA %llu' NL 'SCALARS data float %u' NL 'LOOKUP_TABLE default' NL '", Nx, Ny, Nz, -0.5*(double)(Nx-1u), -0.5*(double)(Ny-1u), -0.5*(double)(Nz-1u), (unsigned long long)Nx*Ny*Nz, ncomp);'];
C{end+1} = '	fputs(hdr, f);';
C{end+1} = '	const ulong Ncell = (ulong)Nx*Ny*Nz;';
C{end+1} = '	float* buf = new float[Ncell*ncomp];';
C{end+1} = '	parallel_for(Ncell*ncomp, [&](ulong n) { union { float f; unsigned int u; } v; v.f = data[n]; v.u = (v.u>>24u)|((v.u>>8u)&0xFF00u)|((v.u<<8u)&0xFF0000u)|(v.u<<24u); buf[n] = v.f; });';
C{end+1} = '	fwrite(buf, 4u*Ncell*ncomp, 1u, f);  fclose(f);  delete[] buf;';
C{end+1} = '}';
fid = fopen(fullfile(pwd,'setup_windtunnel_fan.cpp'),'w');
for k = 1:numel(C)
    Ck = strrep(C{k}, newline, '\n');  fprintf(fid, '%s\n', Ck);
end
fclose(fid);
fprintf('setup_windtunnel_fan.cpp 已生成 (%d 行)\n', numel(C));

%% ---- 编译脚本 ----
batLines = regexp(['@echo off\ncall "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1\ncd /d %~dp0fluidx3d\ncopy /y %~dp0setup_windtunnel_fan.cpp src\setup_windtunnel_fan.cpp >nul\ncopy /y %~dp0windtunnel.raw windtunnel.raw >nul\ncopy /y %~dp0fan_blade.bin fan_blade.bin >nul\ncopy /y %~dp0fan_rotor.bin fan_rotor.bin >nul\ncl.exe /nologo /O2 /EHsc /std:c++17 /W0 /utf-8 /DGRAPHICS=0 /DFORCE_FIELD /DMOVING_BOUNDARIES /I src\OpenCL\include src\graphics.cpp src\info.cpp src\kernel.cpp src\lbm.cpp src\lodepng.cpp src\main.cpp src\setup_windtunnel_fan.cpp src\shapes.cpp /link /LIBPATH:src\OpenCL\lib OpenCL.lib /OUT:windtunnel_fan.exe\necho BUILD_EXITCODE=%ERRORLEVEL%'], '\\n', 'split');
fid = fopen(fullfile(pwd,'build_windtunnel_fan.bat'),'w');
fprintf(fid, '%s\r\n', batLines{:});
fclose(fid);
if ~exist(fullfile(pwd,'vtk'),'dir'), mkdir(fullfile(pwd,'vtk')); end
fprintf('build_windtunnel_fan.bat 已生成\n--- 耦合衔接完成 ---\n');
