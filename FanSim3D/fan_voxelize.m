function [solid, rot0, rotList, grid] = fan_voxelize(P, G)
% fan_voxelize — 风机+前置扩散段 CFD 域体素化 (FluidX3D 衔接)
% 输出: solid   = 静止壁 mask (raw 值 1)
%       rot0    = 旋转叶轮初始 mask θ=0 (raw 值 2): 后盘+叶片×Z+前盘
%       rotList = 旋转区格点清单 [n, r, psi] (C++ 逐步重体素化用, n 线性索引 0-based)
%       grid    = 尺寸/坐标/换算信息
% 流体域 CSG: 增压箱(大气盖) -> 钟形口 -> 吸力眼(贯通上游侧板) -> 叶轮腔(蜗壳型线内,
%   扣除叶轮) -> 下游侧板方孔 -> 出口短管 -> 前置扩散段 -> 稳定段短管 (末端大气出口)
dx = P.gpu.dx;
Lx = G.Lx;  bl = G.blade;  vol = G.vol;

if ~isempty(getenv('FAN_FULL')), dom = 'full'; else, dom = 'fan'; end
switch dom
    case 'full',  x0 = -0.15;  x1 = Lx.xSettle(2);  stubEnd = Lx.xStub(2);
    otherwise,    x0 = -0.15;  x1 = Lx.xStub(2) + 0.05;  stubEnd = x1;  % 短管贯通到域末端 (E 出口)
end
Rbox = vol.Rvol + 0.015;                     % 蜗壳段 y-z 半宽 (箱形域)
nx = round((x1-x0)/dx);
ny = 2*ceil(Rbox/dx);
nz = ny;
xs = x0 + ((1:nx)-0.5)'*dx;
yc = -ny*dx/2 + ((1:ny)-0.5)*dx;
zc = -nz*dx/2 + ((1:nz)-0.5)*dx;
X3 = reshape(xs, nx,1,1);
Y3 = reshape(yc, 1,ny,1);
Z3 = reshape(zc, 1,1,nz);
RC = sqrt(Y3.^2 + Z3.^2);                    % 距轴半径 (y-z 平面)
TH = mod(atan2(Z3, Y3), 2*pi);               % 极角 (自 +y 向 +z, 与蜗壳型线同口径)

% ---- 蜗壳型线 + 叶片 2D 测试 (θ=0; 共享帮手 fan_blade_mask, 与耦合导出同源) ----
rSpiral = vol.rTongue + vol.a*TH;            % r_s(θ) (ny×nz)
tbh = max(bl.tB/2, 0.6*dx);                  % 薄壁自适应 (envR 用)
blade2D = fan_blade_mask(bl, dx, Y3, Z3, RC, TH);
% 叶片扫掠包络半径 (rotList 范围, 取宽裕值)
dC = norm(bl.C);
envR = [max(0, min(bl.r1, abs(dC-bl.Ra)-tbh) - 2*dx), max(bl.r2, dC+bl.Ra+tbh) + 2*dx];

% ---- 逐段流体域 (nx×ny×nz 广播) ----
inPlenum = (X3 < Lx.xBell(1)) & (abs(Y3) < 0.30) & (abs(Z3) < 0.30);
inBell   = (X3 >= Lx.xBell(1)) & (X3 < Lx.xBell(2)) & ...
           (RC < Lx.Rbell + (G.r0eye-Lx.Rbell)*(X3-Lx.xBell(1))/diff(Lx.xBell));
inEye    = (X3 >= Lx.xEye(1)) & (X3 < Lx.xEye(2)) & (RC < G.r0eye);   % 含上游侧板眼孔
inCavity = (X3 >= Lx.xSideUp(2)) & (X3 < Lx.xSideDn(1)) & (RC < reshape(rSpiral,1,ny,nz));
inPort   = (X3 >= Lx.xSideDn(1)) & (X3 < Lx.xSideDn(2)) & ...   % 下游侧板轴向出口方孔
           (abs(Y3) < Lx.W_out/2) & (abs(Z3) < Lx.H_out/2);
inStub   = (X3 >= Lx.xSideDn(2)) & (X3 < stubEnd) & ...
           (abs(Y3) < Lx.W_out/2) & (abs(Z3) < Lx.H_out/2);
sDz = min(max((X3 - Lx.xDiff(1)) / diff(Lx.xDiff), 0), 1);
fQ = 10*sDz.^3 - 15*sDz.^4 + 6*sDz.^5;       % quintic 0→1
wD = Lx.W_out + (Lx.W_sett - Lx.W_out)*fQ;
hD = Lx.H_out + (Lx.H_sett - Lx.H_out)*fQ;
inDiff = (X3 >= Lx.xDiff(1)) & (X3 < Lx.xDiff(2)) & (abs(Y3) < wD/2) & (abs(Z3) < hD/2);
inSettle = (X3 >= Lx.xSettle(1)) & (abs(Y3) < Lx.W_sett/2) & (abs(Z3) < Lx.H_sett/2);

fluid = inPlenum | inBell | inEye | inCavity | inPort | inStub | inDiff;
clear inPlenum inBell inEye inCavity inPort inStub inDiff   % 掩码即用即清 (全域 logical 各 ~30MB)
if strcmp(dom, 'full'), fluid = fluid | inSettle; end

% ---- 旋转叶轮初始 mask (θ=0): 仅叶片 (前/后盘按静止壁处理: 盘面平行于旋转平面,
%      动壁效应仅剩盘缘线速度, 对性能影响归入 1D 盘摩擦; 静止标记使转矩测量只含叶片) ----
slBlade = (X3 >= Lx.xBlade(1)) & (X3 < Lx.xBlade(2));
slRear  = (X3 >= Lx.xRear(1))  & (X3 < Lx.xRear(2));
slFront = (X3 >= Lx.xFront(1)) & (X3 < Lx.xFront(2));
rot0 = slBlade & reshape(blade2D,1,ny,nz);                        % 旋转叶片 (raw=2)
clear blade2D
mDisc = (slRear  & (RC >= G.r0eye) & (RC <= bl.r2)) | ...      % 后盘: 内径=眼径 (吸入口穿孔!)
        (slFront & (RC >= G.r0eye)   & (RC <= bl.r2));
mHub = (X3 >= Lx.xRear(1)) & (X3 < Lx.xSideDn(2)) & (RC < bl.r1);  % 毂实体: 堵死眼→出口轴向短路
fluid = (fluid & ~rot0) & ~mDisc & ~mHub;
clear mDisc mHub slRear slFront
solid = ~fluid;                               % 静止壁 = 域内非流体 (外壳/蜗壳/侧板/管壁/盘)
clear fluid

% ---- 旋转区格点清单 (叶片层 ∩ 包络环) ----
idx = slBlade & (RC >= envR(1)) & (RC <= envR(2));
ii = find(idx);
mIdx = floor((ii(:)-1)/nx);  iy = mod(mIdx, ny) + 1;  iz = floor(mIdx/ny) + 1; % x 最快序
rrL = sqrt(yc(iy).^2 + zc(iz).^2);  rrL = rrL(:);
ppL = mod(atan2(zc(iz), yc(iy)), 2*pi);  ppL = ppL(:);
rotList = [double(ii(:)) - 1, rrL, ppL];      % 0-based n, r(m), psi(rad) (写盘处按列转 single)

grid = struct('dx',dx, 'x0',x0, 'nx',nx, 'ny',ny, 'nz',nz, 'x',xs, 'y',yc, 'z',zc, ...
    'dom',dom, 'Rbox',Rbox, 'envR',envR);
fprintf('[fan_voxelize] 域 %s: %dx%dx%d = %.1fM 格 (dx=%.1fmm) | 旋转区 %d 格 | 叶片 %.1f 格厚\n', ...
    dom, nx, ny, nz, nx*ny*nz/1e6, dx*1e3, size(rotList,1), bl.tB/dx);
if bl.tB/dx < 1.2
    fprintf('[fan_voxelize] [!] 叶片厚 %.1fmm < 1.2 格 — 建议 WT_DX≤%.0fmm 或加厚叶片\n', bl.tB*1e3, bl.tB/1.2*1e3);
end
end
