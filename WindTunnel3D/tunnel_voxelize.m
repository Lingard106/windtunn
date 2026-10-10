function [solid, objMask, grid] = tunnel_voxelize(P, G)
% tunnel_voxelize — 动态风场域体素化: 涵道壁 + 测试物 (参数化或 STL)
% 域: 试验段 ± margins; 输出 solid / objMask (仅测试物) / grid
dx = P.lbm.dx;
x0 = G.xSec(3) - P.lbm.margins(1);
x1 = G.xSec(4) + P.lbm.margins(2);
nx = round((x1-x0)/dx);

xs = x0 + ((1:nx)-0.5)'*dx;
aI = interp1(G.x, G.a, xs, 'pchip', 'extrap');
bI = interp1(G.x, G.b, xs, 'pchip', 'extrap');

% 矩形截面域: y/z 两方向各自按最大半宽/半高独立取格数 (方截面时退化相同)
Hy = 2*max(aI) + 4*dx;                 % 域宽 ≥ 涵道最宽处 (收缩段入口侧)
Hz = 2*max(bI) + 4*dx;                 % 域高
ny = round(Hy/dx);  nz = round(Hz/dx);
yc = -Hy/2 + ((1:ny)-0.5)*dx;
zc = -Hz/2 + ((1:nz)-0.5)*dx;

% 广播维度: reshape 视图 (nx,1,1)/(1,ny,1)/(1,1,nz) -> 隐式扩展成 (nx,ny,nz)
X3 = reshape(xs, nx,1,1);
Y3 = reshape(yc, 1,ny,1);
Z3 = reshape(zc, 1,1,nz);
A3 = reshape(aI, nx,1,1);  B3 = reshape(bI, nx,1,1);

if ~isempty(getenv('WT_DBGSZ'))
    fprintf('  [vox] nx=%d ny=%d nz=%d\n', nx,ny,nz);
end
inDuct = abs(Y3) < A3 & abs(Z3) < B3;        % 洞内流体
solid = ~inDuct;                             % 涵道壁
objMask = false(nx,ny,nz);


c = (G.xSec(3)+G.xSec(4))/2;                   % 试验段中心 (各测试物共用)
switch lower(P.lbm.object)
    case 'sphere'
        r = P.lbm.objSize/2;
        objMask = objMask | ((X3-c).^2 + Y3.^2 + Z3.^2 <= r^2);
    case 'cylinder'
        r = P.lbm.objSize/2;
        objMask = objMask | ((X3-c).^2 + Z3.^2 <= r^2);     % 轴沿 y (横向, 可脱涡)
    case 'plate'
        w = P.lbm.objSize;  h = w/3;
        objMask = objMask | (abs(X3-c) <= h & abs(Y3) <= w/2 & abs(Z3) <= w/2);
    case 'naca'
        objMask = objMask | nacaWing((G.xSec(3)+G.xSec(4))/2 - P.lbm.objSize/2, ...
            ny, nz, Z3, P.lbm.objSize);
    case 'stl'
        xTc = (G.xSec(3)+G.xSec(4))/2;         % 模型包围盒中心自动平移到试验段中心
        objMask = objMask | voxelizeSTL(P.lbm.objFile, dx, x0, xs, yc, zc, nx, ny, nz, xTc);
    otherwise
        error('未知测试物类型: %s', P.lbm.object);
end
objMask(~inDuct) = false;
if nnz(objMask) == 0
    error('测试物体素为 0 — 测试物超出涵道范围或尺寸过小 (objSize/dx=%.1f), 请增大 objSize 或减小 dx', ...
        P.lbm.objSize/dx);
end
% wallSolid: 仅静止壁 (涵道壁+涵道环), 不含旋转测试物 — 供 lbm 逐步更新
wallSolid = solid;
solid = solid | objMask;

grid = struct('dx',dx, 'x0',x0, 'nx',nx, 'ny',ny, 'nz',nz, 'x',xs, 'y',yc, 'z',zc, ...
    'xc',c, ...
    'Ain', 4*aI(1)*bI(1), 'Ats', P.ts.W*P.ts.H);   % 入口/试验段截面积 (通量匹配)
end

%% ==================== NACA0012 翼型拉伸体 (弦沿 x, 前缘=xE, 展沿 y) ====================
function m = nacaWing(xE, ny, nz, Z3, chord)
    t = 0.12;
    xs = (0:0.002:1)'*chord + xE;
    nx = numel(xs);
    xc = 0:1/(nx-1):1;
    yt = 5*t*(0.2969*xc - 0.1260*xc.^2 - 0.3516*xc.^3 + 0.2843*xc.^4 - 0.1036*xc.^5);
    X3 = reshape(xs, nx,1,1);
    TH = reshape(yt, nx,1,1);
    inX = true(nx,1,1);
    m = repmat(abs(Z3) <= TH & inX, [1 ny 1]);
end

%% ==================== STL 体素化 (中心自动对齐试验段中心; 表面带+沿x偶奇填充) ====================
function m = voxelizeSTL(fname, dx, x0, xs, yc, zc, nx, ny, nz, xCenter)
    if ~exist(fname,'file')
        error(['找不到 STL: %s\n将你的 CAD 模型导出为二进制 STL 放到该路径, ' ...
            '或修改 tunnel_params 的 P.lbm.objFile'], fname);
    end
    TR = stlread(fname);
    V = TR.Points;  F = TR.ConnectivityList;
    % 平移: 模型包围盒中心 → (xCenter, 0, 0)
    c = (min(V)+max(V))/2;
    V = V - c + [xCenter, 0, 0];
    fprintf('  [STL] %d 顶点, %d 三角面 | 尺寸 %.1f×%.1f×%.1f mm (已对齐试验段中心)\n', ...
        size(V,1), size(F,1), (max(V(:,1))-min(V(:,1)))*1e3, ...
        (max(V(:,2))-min(V(:,2)))*1e3, (max(V(:,3))-min(V(:,3)))*1e3);
    ymin = min(yc);  zmin = min(zc);
    m = false(nx,ny,nz);
    for f = 1:size(F,1)
        p = V(F(f,:),:);
        i1 = max(1, floor((min(p(:,1))-x0)/dx)+1);   i2 = min(nx, ceil((max(p(:,1))-x0)/dx)+1);
        j1 = max(1, floor((min(p(:,2))-ymin)/dx)+1); j2 = min(ny, ceil((max(p(:,2))-ymin)/dx)+1);
        k1 = max(1, floor((min(p(:,3))-zmin)/dx)+1); k2 = min(nz, ceil((max(p(:,3))-zmin)/dx)+1);
        if i2<i1 || j2<j1 || k2<k1, continue; end
        m(i1:i2, j1:j2, k1:k2) = true;       % 表面带
    end
    c2 = cumsum(double(m), 1);
    tot = reshape(c2(end,:,:), 1, ny, nz);
    interior = ~m & c2 > 0 & c2 < repmat(tot, [nx 1 1]);
    m = m | interior;                        % 内部填充
end
