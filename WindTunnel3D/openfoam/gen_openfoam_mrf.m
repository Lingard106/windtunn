% gen_openfoam_mrf — 生成 openfoam/windtunnel-mrf case (MRF 叶轮 + 蜗壳 + 五段风洞)
% 几何: export_fluidx3d_fan.m 耦合布局的精确复刻 (同一套 CSG 公式, 单一真源 FanSim3D)
%   坐标系: 风洞轴 +x, 叶轮轴 +y (旋转平面 xoz), 进气链沿 -y (增压箱->钟形口->吸力眼->叶轮)
%   蜗壳: 阿基米德蜗壁 r(TH)=rTongue+a·mod(TH-300°,2π), 切向出口带在底部 (-z), 渐扩短管对接法兰
% 网格: blockMesh 背景盒 20mm + snappyHexMesh (casing 10mm / rotor 圆柱 5mm / 试验段 5mm)
% MRF : topoSet cylinderToCell 建 rotor cellZone; omega 绕 +y 取负 (与 FluidX3D omegaFull 符号配对)
% 输出: <suite>/openfoam/windtunnel-mrf/{tri/*.stl, 0/*, constant/*, system/*} + geometry_check.png
RT   = 'D:\WindTunnelSuite\WindTunnel3D';
FAND = 'D:\WindTunnelSuite\FanSim3D';
CASE = 'D:\WindTunnelSuite\openfoam\windtunnel-mrf';
addpath(RT, FAND);

%% ---- 参数与几何 (单一真源) ----
P  = tunnel_params();
Pf = fan_params();
R  = fan_performance(Pf);
Gf = fan_geometry(Pf, R);
Lx = Gf.Lx;  vf = Gf.vol;  bl = Gf.blade;

%% ---- 耦合布局 (逐式复刻 export_fluidx3d_fan.m L57-69, spinS=+1) ----
yB0    = (Lx.xSideUp(2) + Lx.xSideDn(1))/2;
yL     = @(xl) yB0 - xl;                       % 局部贯流坐标 -> 全局 y
WcavH  = (Lx.xSideDn(1) - Lx.xSideUp(2))/2;    % 蜗壳腔半宽 (腔体对中 y=0)
THt    = deg2rad(300);                         % 舌部角
rSb    = vf.rTongue + vf.a*deg2rad(330);       % 出口扇区壁半径 (TH=270°)
zF     = (rSb + vf.rTongue)/2;                 % 蜗壳心高度 (出口带对中风洞轴线)
xC     = -(vf.Rvol + 0.03);                    % 蜗壳心 x
xE     = xC + vf.Rvol;                         % 蜗壳 +x 边缘
zDLo   = zF - rSb;   zDHi = zF - vf.rTongue;   % 切向出口带
sWfun  = @(x) min(max((x - xE)/(0 - xE), 0), 1);
wYdfun = @(x) WcavH + (Pf.fd.W_in/2 - WcavH)*sWfun(x);
zLofun = @(x) zDLo + (-Pf.fd.H_in/2 - zDLo)*sWfun(x);
zHifun = @(x) zDHi + ( Pf.fd.H_in/2 - zDHi)*sWfun(x);

yCavU = yL(Lx.xSideUp(2));   yCavD = yL(Lx.xSideDn(1));   % 腔带 ±0.129
yBl0  = yL(Lx.xBlade(1));    yBl1  = yL(Lx.xBlade(2));    % 叶片带
yRr0  = yL(Lx.xRear(1));     yFr1  = yL(Lx.xFront(2));    % 盘内侧面
yFr0  = yL(Lx.xFront(1));                                  % 前盘 -y 面
yHub0 = yL(Lx.xSideDn(2));   yHub1 = yL(Lx.xRear(1));     % 毂
r0eye = Gf.r0eye;  Rbell = Lx.Rbell;
yEye0 = yL(Lx.xEye(2));      yEye1 = yL(Lx.xBell(2));     % 吸力眼筒
yBell1= yB0;                 yPlTop= yB0 + 0.30;          % 钟形口出口平面 / 箱顶(大气)
gapShroud = 0.002;                                        % 后盘-上游板退让 (防 STL 共面)
yRr1c = yCavU - gapShroud;
tBeff = max(bl.tB, 1.2*0.005);                            % 叶片 CFD 加厚 (同 FluidX3D 1.2dx 法则)

% 风洞五段 (与验证过的 windtunnel-base/blockMeshDict 逐值一致)
S  = [0 0.5 0.95 1.55 1.75 2.5];
HY = [0.150 0.3675 0.3675 0.150 0.150 0.200];
HZ = [0.135 0.245 0.245 0.100 0.100 0.130];

rpm   = Pf.imp.rpm;
omega = -2*pi*rpm/60;    % 绕 +y, 负号与蜗壳螺旋配对 (setup_windtunnel_fan.cpp omegaFull 同号)

%% ---- 摘要 ----
fprintf('=== windtunnel-mrf 布局摘要 ===\n');
fprintf('蜗壳心 (x,z) = (%.4f, %.4f) m | Rvol=%.1fmm rTongue=%.1fmm a=%.2fmm/rad rSb=%.1fmm\n', ...
    xC, zF, vf.Rvol*1e3, vf.rTongue*1e3, vf.a*1e3, rSb*1e3);
fprintf('出口带 z ∈ [%.3f, %.3f] | 短管: wYd %.1f->%.1fmm, z底 %.1f->%.1f, z顶 %.1f->%.1f mm\n', ...
    zDLo, zDHi, wYdfun(xE)*1e3, wYdfun(0)*1e3, zLofun(xE)*1e3, zLofun(0)*1e3, zHifun(xE)*1e3, zHifun(0)*1e3);
fprintf('叶片: Z=%d beta1A=%.1f° beta2A=%.1f° 包角=%.0f° tB_eff=%.1fmm | u2=%.2f m/s | omega=%.3f rad/s\n', ...
    bl.Z, R.tri.beta1A, R.tri.beta2A, bl.phi, tBeff*1e3, Pf.imp.u2, omega);
fprintf('y带: 腔[%.3f,%.3f] 叶[%.3f,%.3f] 后盘[%.3f,%.3f] 前盘[%.3f,%.3f] 毂[%.3f,%.3f] 眼[%.3f,%.3f] 箱顶=%.3f\n', ...
    yCavD, yCavU, yBl1, yBl0, yRr0, yRr1c, yFr1, yFr0, yHub0, yHub1, yEye0, yEye1, yPlTop);

%% ---- 2D 截面掩码 (xoz, 复刻 CSG) 与轮廓提取 ----
% 槽道项止于 xE (平直段末端): 多边形封口边 = xE 截面环, 与解析渐扩短管起始环严丝合缝;
% 若写到 x=0, contourc 会在法兰内侧生成封口墙, 蜗壳与隧道被隔断 (2026-10-04 区域被删根因)
secMask = @(X,Z) ( hypot(X-xC, Z-zF) < (vf.rTongue + vf.a*mod(atan2(Z-zF, X-xC) - THt, 2*pi)) ) ...
             | ( (X >= xC) & (X < xE) & (Z > zDLo) & (Z < zDHi) );
dxr = 0.001;
xr = (xC-vf.Rvol-0.05):dxr:0.005;  zr = (zF-vf.Rvol-0.02):dxr:(zF+vf.Rvol+0.02);
[Xg, Zg] = meshgrid(xr, zr);
maskSec = secMask(Xg, Zg) & hypot(Xg-xC, Zg-zF) > bl.r1;   % 挖毂孔 (挤出孔环用)
CS = contourc(xr, zr, double(maskSec), [0.5 0.5]);
loops = {}; k = 1;
while k <= size(CS,2)
    n = CS(2,k);  loops{end+1} = CS([1 2], k+1:k+n)';  k = k + n + 1;
end
areas = cellfun(@(L) abs(polyarea(L(:,1), L(:,2))), loops);
keep = find(areas > 0.01);                                  % 滤噪: 只留大环
loops = loops(keep);  areas = areas(keep);
[~, ord] = sort(areas, 'descend');
outerLoop = loops{ord(1)};
holeLoop  = loops{ord(2)};             % 毂孔环 (与下游板帽共点)
assert(size(holeLoop,1) > 8, '毂孔环提取失败');
fprintf('截面外环 %d 点 / 毂孔环 %d 点 | 外环面积 %.4f m²\n', ...
    size(outerLoop,1), size(holeLoop,1), polyarea(outerLoop(:,1), outerLoop(:,2)));

%% ==================== STL: casing ====================
V = zeros(0,3); F = zeros(0,3);
bandY = yCavD + (yCavU - yCavD)*[0 1/4 2/4 3/4 1];
% 外环挤出: 跳过槽道封口边 (两端 x 都在 xE 附近的外环边) — 封口若成墙会堵死蜗壳->短管
nL = size(outerLoop,1);
skipE = false(nL,1);
for e = 1:nL
    e2 = mod(e,nL)+1;
    if outerLoop(e,1) > xE-1.5e-3 && outerLoop(e2,1) > xE-1.5e-3, skipE(e) = true; end
end
fprintf('槽道封口边(跳过): %d / %d 段\n', nnz(skipE), nL);
for e = 1:nL
    if skipE(e), continue; end
    e2 = mod(e,nL)+1;
    p1 = outerLoop(e,:);  p2 = outerLoop(e2,:);
    for b = 1:numel(bandY)-1
        % [x, y, z] 顺序 (p=(x,z), bandY=y — 勿犯 (x,z,y) 转置)
        [V,F] = pushQuad(V,F, [p1(1) bandY(b) p1(2)], [p2(1) bandY(b) p2(2)], ...
                            [p2(1) bandY(b+1) p2(2)], [p1(1) bandY(b+1) p1(2)]);
    end
end
[V,F] = extrudeLoopWalls(V, F, holeLoop, bandY);
% 下游侧板面 (y=yCavD): 外环 + 毂孔环, r>=r1
[V, F] = fillCap({outerLoop, holeLoop}, ...
    @(X,Z) secMask(X,Z) & hypot(X-xC,Z-zF) >= bl.r1, yCavD, V, F, xC, zF);
% 上游侧板面 (y=yCavU): 外环 + 吸力眼解析环 (与眼筒底环共点), r>=r0eye
Nring = 96;
eyeRingB = ringVerts(r0eye, yCavU, Nring, xC, zF);
[V, F] = fillCap({outerLoop, eyeRingB(:,[1 3])}, ...
    @(X,Z) secMask(X,Z) & hypot(X-xC,Z-zF) >= r0eye, yCavU, V, F, xC, zF);
% 渐扩短管 4 壁 (x: xE->0, 10mm 步)
xs = (xE:0.01:0)';
for s = 1:4
    for i = 1:numel(xs)-1
        c1 = ductCorners(xs(i),   wYdfun, zLofun, zHifun);
        c2 = ductCorners(xs(i+1), wYdfun, zLofun, zHifun);
        [V,F] = pushQuad(V, F, [xs(i)   c1(s,:)            ], [xs(i)   c1(mod(s,4)+1,:)    ], ...
                            [xs(i+1) c2(mod(s,4)+1,:)], [xs(i+1) c2(s,:)            ]);
    end
end
% 吸力眼筒 + 钟形口
eyeRingT = ringVerts(r0eye, yEye1, Nring, xC, zF);
[V,F] = loftRings(V, F, eyeRingB, eyeRingT);
nb = 4;  yb = linspace(yEye1, yBell1, nb+1);  rb = linspace(r0eye, Rbell, nb+1);
for b = 1:nb
    [V,F] = loftRings(V, F, ringVerts(rb(b), yb(b), Nring, xC, zF), ...
                            ringVerts(rb(b+1), yb(b+1), Nring, xC, zF));
end
% 增压箱: 4 侧壁 + 底面 (方形环 - 钟形口圆孔)
bx = [xC-0.30, xC+0.30];  bz = [zF-0.30, zF+0.30];
sq = resampleLoop([bx(1) bz(1); bx(2) bz(1); bx(2) bz(2); bx(1) bz(2)], 0.02);
bellT = ringVerts(Rbell, yBell1, Nring, xC, zF);
for e = 1:size(sq,1)
    p1 = sq(e,:); p2 = sq(mod(e,size(sq,1))+1,:);
    % [x, y, z] 顺序 (p=(x,z))
    [V,F] = pushQuad(V, F, [p1(1) yPlTop p1(2)], [p2(1) yPlTop p2(2)], ...
                         [p2(1) yBell1 p2(2)], [p1(1) yBell1 p1(2)]);
end
[V, F] = fillCap({sq, bellT(:,[1 3])}, ...
    @(X,Z) X>=bx(1) & X<=bx(2) & Z>=bz(1) & Z<=bz(2) & hypot(X-xC,Z-zF) >= Rbell, ...
    yBell1, V, F, xC, zF);
casing = triangulation(F, V);
fprintf('casing.stl: %d 面片\n', size(casing.ConnectivityList,1));

%% ==================== STL: tunnel ====================
Vt = zeros(0,3); Ft = zeros(0,3);
for seg = 1:numel(S)-1
    xs = linspace(S(seg), S(seg+1), max(2, round((S(seg+1)-S(seg))/0.05)+1));
    for i = 1:numel(xs)-1
        c1 = tunnelCorners(xs(i), S, HY, HZ);
        c2 = tunnelCorners(xs(i+1), S, HY, HZ);
        for s = 1:4
            [Vt,Ft] = pushQuad(Vt, Ft, [xs(i)   c1(s,:)            ], [xs(i)   c1(mod(s,4)+1,:)    ], ...
                                       [xs(i+1) c2(mod(s,4)+1,:)], [xs(i+1) c2(s,:)            ]);
        end
    end
end
tunnel = triangulation(Ft, Vt);
fprintf('tunnel.stl: %d 面片\n', size(tunnel.ConnectivityList,1));

%% ==================== STL: impeller ====================
Vi = zeros(0,3); Fi = zeros(0,3);
Nhub = 64;
yHubT = yCavU - 0.001;   % 毂顶下移 1mm: 与 casing 毂盖(静止)留缝, 防 STL 共面
hB = ringVerts(bl.r1, yHub0, Nhub, xC, zF);
hT = ringVerts(bl.r1, yHubT, Nhub, xC, zF);
[Vi,Fi] = loftRings(Vi, Fi, hB, hT);
[Vi,Fi] = fanCap(Vi, Fi, hB, xC, zF);
[Vi,Fi] = fanCap(Vi, Fi, hT, xC, zF);
for sd = 1:2   % 1=后盘(0.125~0.127 退让) 2=前盘(-0.119~-0.115, 保留 10mm 真实间隙带)
    if sd == 1, ya = yRr0; yb2 = yRr1c; else, ya = yFr1; yb2 = yFr0; end
    iA = ringVerts(r0eye, ya, Nhub, xC, zF);  iB = ringVerts(r0eye, yb2, Nhub, xC, zF);
    oA = ringVerts(bl.r2,  ya, Nhub, xC, zF); oB = ringVerts(bl.r2,  yb2, Nhub, xC, zF);
    [Vi,Fi] = loftRings(Vi, Fi, iA, iB);
    [Vi,Fi] = loftRings(Vi, Fi, oA, oB);
    for k = 1:Nhub
        k2 = mod(k,Nhub)+1;
        [Vi,Fi] = pushQuad(Vi,Fi, iA(k,:), iA(k2,:), oA(k2,:), oA(k,:));
        [Vi,Fi] = pushQuad(Vi,Fi, iB(k,:), iB(k2,:), oB(k2,:), oB(k,:));
    end
end
% 32 圆弧叶片: fan_blade_arc 单一真源, 局部 (yl,zl) 绕蜗壳心旋转到 xoz
[~, Cb, Rab, ~, thB] = fan_blade_arc(bl.r1, bl.r2, R.tri.beta1A, R.tri.beta2A); tE = linspace(0,1,41)'; Pc = Cb + Rab*[cosd(thB(1)+tE*diff(thB)), sind(thB(1)+tE*diff(thB))];
% 叶根向内延伸 4mm 吃进毂
u1 = (Pc(1,:) - Cb);  u1 = u1 / norm(u1);
Pc = [Cb + u1*(bl.r1 - 0.004); Pc];
tg = [diff(Pc,1,1); Pc(end,:)-Pc(end-1,:)];  tg = tg ./ vecnorm(tg, 2, 2);
nrm = [-tg(:,2), tg(:,1)];
fl = sign(sum((Pc - Cb).*nrm, 2));  fl(fl==0) = 1;  nrm = nrm .* fl;   % 法向指离弧心
oP = Pc + nrm*tBeff/2;  oM = Pc - nrm*tBeff/2;
N = size(Pc,1);
csXZ = [oP; oM(end:-1:1,:)];                              % 截面闭环 (yl,zl) 2N 点
for zb = 1:bl.Z
    thk = (zb-1)*2*pi/bl.Z;
    Rm = [cos(thk) -sin(thk); sin(thk) cos(thk)];
    csr = csXZ * Rm';
    csg = [xC + csr(:,1), zF + csr(:,2)];
    ylo = yBl1 - 0.001;  yhi = yBl0 + 0.001;               % 轴向各吃 1mm 进前/后盘
    n2 = size(csg,1);
    plo = [csg(:,1), repmat(ylo, n2, 1), csg(:,2)];        % [x, y, z] 顺序 (勿犯 y/z 转置)
    phi3 = [csg(:,1), repmat(yhi, n2, 1), csg(:,2)];
    for e = 1:n2                                           % 侧壁
        e2 = mod(e, n2)+1;
        [Vi,Fi] = pushQuad(Vi,Fi, plo(e,:), plo(e2,:), phi3(e2,:), phi3(e,:));
    end
    for k2 = 1:N-1                                         % 两端帽 (条带三角化)
        j1 = 2*N - k2 + 1;  j2 = 2*N - k2;                 % oM 反序对应
        [Vi,Fi] = pushTri(Vi,Fi, plo(k2,:), plo(k2+1,:), plo(j1,:));
        [Vi,Fi] = pushTri(Vi,Fi, plo(k2,:), plo(j1,:), plo(j2,:));
        [Vi,Fi] = pushTri(Vi,Fi, phi3(k2,:), phi3(k2+1,:), phi3(j1,:));
        [Vi,Fi] = pushTri(Vi,Fi, phi3(k2,:), phi3(j1,:), phi3(j2,:));
    end
end
impeller = triangulation(Fi, Vi);
fprintf('impeller.stl: %d 面片 | 叶片 r∈[%.1f, %.1f]mm tB_eff=%.1fmm\n', ...
    size(impeller.ConnectivityList,1), (bl.r1-0.004)*1e3, bl.r2*1e3, tBeff*1e3);

%% ==================== STL: 被测物 (可选) ====================
% 被测物接口: OBJ='none'|'sphere'|'cylinder', 与 FluidX3D 耦合管线同参数位 (cTs=1.25)
% sphere: r=OBJ_R 球 @ (OBJ_C,0,0); cylinder: 轴沿 y 贯穿, 圆截面 r=OBJ_R @ xz=(OBJ_C,0)
objEnv = getenv('WT_MRF_OBJ'); if isempty(objEnv), objEnv = 'none'; end
OBJ   = objEnv;        % ← 改 'sphere'/'cylinder' 启用
OBJ_R = 0.030;
OBJ_C = 1.25;          % = (xSec(3)+xSec(4))/2, 与 export_fluidx3d_fan 的 cTs 同位
Vobj = [];  Fobj = [];
switch lower(OBJ)
    case 'sphere'
        nlat = 24;  nlon = 48;
        Vobj = zeros(nlat*nlon + 2, 3);
        Vobj(1,:) = [OBJ_C+OBJ_R, 0, 0];                    % +x 极 (球心 OBJ_C,0,0)
        Vobj(end,:) = [OBJ_C-OBJ_R, 0, 0];
        vi = 2;
        for il = 1:nlat-1
            phi = -pi/2 + pi*il/nlat;
            for io = 1:nlon
                th = 2*pi*io/nlon;
                Vobj(vi,:) = [OBJ_C+OBJ_R*cos(phi)*cos(th), OBJ_R*cos(phi)*sin(th), OBJ_R*sin(phi)];
                vi = vi + 1;
            end
        end
        Fobj = zeros((nlat-2)*nlon*2 + 2*nlon, 3);  fi = 1;
        for io = 1:nlon                                     % 南北极帽
            io2 = mod(io, nlon) + 1;
            Fobj(fi,:) = [1, 1+io, 1+io2];  fi = fi + 1;
            Fobj(fi,:) = [size(Vobj,1), (nlat-2)*nlon+io2+1, (nlat-2)*nlon+io+1];  fi = fi + 1;
        end
        for il = 1:nlat-2                                   % 主体四边形
            for io = 1:nlon
                io2 = mod(io, nlon) + 1;
                a = 1 + (il-1)*nlon + io;  b = 1 + (il-1)*nlon + io2;
                c = a + nlon;  d = b + nlon;
                Fobj(fi,:) = [a, b, c];  fi = fi + 1;
                Fobj(fi,:) = [b, d, c];  fi = fi + 1;
            end
        end
        Fobj = Fobj(1:fi-1, :);
        fprintf('testobj: sphere r=%.0fmm @ x=%.2f, %d 面片\n', OBJ_R*1e3, OBJ_C, fi-1);
    case 'cylinder'
        nc = 64;  ylo = -0.20;  yhi = 0.20;                 % 贯穿试验段高度
        Vobj = zeros(2*nc, 3);
        th = (0:nc-1)'*2*pi/nc;
        Vobj(1:nc,:) = [OBJ_C + OBJ_R*cos(th), repmat(ylo, nc,1), OBJ_R*sin(th)];
        Vobj(nc+1:end,:) = [OBJ_C + OBJ_R*cos(th), repmat(yhi, nc,1), OBJ_R*sin(th)];
        Fobj = zeros(nc*2 + 2*nc, 3);  fi = 1;
        for k = 1:nc                                        % 侧壁
            k2 = mod(k, nc) + 1;
            Fobj(fi,:) = [k, k2, nc+k2];  fi = fi + 1;
            Fobj(fi,:) = [k, nc+k2, nc+k];  fi = fi + 1;
        end
        Fobj(fi,:) = [1, 2, nc+2];  fi = fi+1;  Fobj(fi,:) = [1, nc+2, nc+1];  fi = fi+1;      % 端帽 (扇形近似)
        Fobj(fi,:) = [nc, 2*nc, nc+1];  fi = fi+1;  Fobj(fi,:) = [nc, nc+1, 1];  fi = fi+1;
        Fobj = Fobj(1:fi-1, :);
        fprintf('testobj: cylinder r=%.0fmm 轴沿 y @ x=%.2f, %d 面片\n', OBJ_R*1e3, OBJ_C, fi-1);
    otherwise
        fprintf('被测物: none\n');
end

%% ---- 封闭帽 (奇偶射线判定要求 skin 全封闭; 帽面=边界 patch) ----
% 增压箱顶帽 (inletCap): 全矩形 (bell 在箱底, 顶面无孔)
[Vic, Fic] = fillCap({sq}, @(X,Z) X>=bx(1) & X<=bx(2) & Z>=bz(1) & Z<=bz(2), yPlTop, zeros(0,3), zeros(0,3), xC, zF);
% 风洞出口帽 (outletCap): 0.40x0.26 @ x=2.5 (x=常平面, 直接 2 三角)
Voc = [S(end) -HY(end) -HZ(end); S(end) HY(end) -HZ(end); S(end) HY(end) HZ(end); S(end) -HY(end) HZ(end)];
Foc = [1 2 3; 1 3 4];
% 毂盖 (casing 侧, 静止): 毂孔轮廓线内 r<=r1 圆盘 @ y=yCavU (叶轮毂顶在其下方 1mm)
[V, F] = fillCap({holeLoop}, @(X,Z) hypot(X-xC,Z-zF) <= bl.r1, yCavU, V, F, xC, zF);

%% ---- 写盘 ----
if exist(CASE, 'dir'), rmdir(CASE, 's'); end
mkdir(fullfile(CASE,'tri')); mkdir(fullfile(CASE,'0')); mkdir(fullfile(CASE,'constant')); mkdir(fullfile(CASE,'system'));
% skin.stl: ASCII 三 solid (casing 静壁含风洞/毂盖 | inletCap | outletCap) — 单文件保证奇偶射线判定封闭
% 隧道壁 (Vt,Ft) 必须并入 casing solid — 漏并 = 隧道无墙 = 流体直通环境 (2026-10-04 泄漏根因之一)
writeSTLascii(fullfile(CASE,'tri','skin.stl'), 'casing', [V; Vt], [F; Ft + size(V,1)], ...
    'inletCap', Vic, Fic, 'outletCap', Voc, Foc);
stlwrite(impeller, fullfile(CASE,'tri','impeller.stl'));
if ~isempty(Vobj)
    stlwrite(triangulation(Fobj, Vobj), fullfile(CASE,'tri','testobj.stl'));
end
writeDicts(CASE, xC, zF, omega, rpm, bl.r2, yPlTop, OBJ, OBJ_R, OBJ_C);
jsonOut = struct('xC',xC,'zF',zF,'Rvol',vf.Rvol,'rTongue',vf.rTongue,'a',vf.a,'rSb',rSb, ...
    'c2u',R.tri.c2u,'beta1A',R.tri.beta1A,'beta2A',R.tri.beta2A,'u2',Pf.imp.u2, ...
    'omega',omega,'rpm',rpm,'tBeff',tBeff,'bladeBandY',[yBl1 yBl0],'Qdesign',R.op.Q);
fid = fopen(fullfile(CASE,'geometry_summary.json'),'w'); fwrite(fid, jsonencode(jsonOut)); fclose(fid);

%% ---- 校验图 ----
fig = figure('Position',[50 50 1500 680], 'Color','w');
subplot(1,2,1); hold on; axis equal; grid on;
th = linspace(0, 2*pi, 720);
plot(xC + vf.Rvol*cos(th), zF + vf.Rvol*sin(th), 'k-.');
patch(outerLoop(:,1), outerLoop(:,2), [0.75 0.85 1], 'EdgeColor','b', 'FaceAlpha',0.5);
patch(holeLoop(:,1), holeLoop(:,2), 'w', 'EdgeColor','r');
for zb = 1:bl.Z
    thk = (zb-1)*2*pi/bl.Z;  Rm = [cos(thk) -sin(thk); sin(thk) cos(thk)];
    csr = csXZ * Rm';
    plot(xC + csr(:,1), zF + csr(:,2), 'r-', 'LineWidth', 0.5);
end
xt = linspace(0, 2.5, 200);
hyt = interp1(S, HY, xt);  hzt = interp1(S, HZ, xt);
patch([xt fliplr(xt)], [hyt -hyt], [0.9 1 0.9], 'EdgeColor','g', 'FaceAlpha',0.4);
patch([xE 0 0 xE], [zLofun(xE) zLofun(0) zHifun(0) zHifun(xE)], [1 0.9 0.95], 'EdgeColor','m');
plot(xC, zF, 'ko', 'MarkerFaceColor', 'k');
plot(bx, [bz(1) bz(1)], 'c-'); plot(bx, [bz(2) bz(2)], 'c-'); plot([bx(1) bx(1)], bz, 'c-'); plot([bx(2) bx(2)], bz, 'c-');
title('俯视 xoz: 蜗壳腔(蓝) 叶片(红) 短管(品红) 风洞(绿) 增压箱(青)');
xlabel('x (m)'); ylabel('z (m)');
subplot(1,2,2); hold on; axis equal; grid on;
plot([-1.1 2.5], [0 0], 'k-');
yy = [yHub0 yCavD yBl1 yFr1 yFr0 yBl0 yRr0 yRr1c yCavU yEye0 yEye1 yBell1 yPlTop];
lb = {'毂底','板D','叶带-','前盘1','前盘0','叶带+','后盘0','后盘1c','板U','眼底','眼顶','箱底','箱顶'};
for i = 1:numel(yy)
    plot([-1.1 2.5], [yy(i) yy(i)], 'b-');
    text(2.52, yy(i), sprintf('%s %.1f', lb{i}, yy(i)*1e3), 'FontSize', 7);
end
title('侧视 xy: y 向布局带 (数值 mm)'); xlabel('x (m)'); ylabel('y (m)');
saveas(fig, fullfile(CASE, 'geometry_check.png'));
fprintf('geometry_check.png 已写\n=== 生成完毕: %s ===\n', CASE);

%% ==================== 局部函数 ====================
function L = resampleLoop(L, dmax)
P = [L; L(1,:)];  d = vecnorm(diff(P,1,1), 2, 2);  s = [0; cumsum(d)];
n = max(8, ceil(s(end)/dmax));  si = linspace(0, s(end), n+1)';
Q = [interp1(s, P(:,1), si), interp1(s, P(:,2), si)];
L = Q(1:end-1,:);
end

function rv = ringVerts(r, y, N, xC, zF)
th = (0:N-1)'*2*pi/N;
rv = [xC + r*cos(th), repmat(y, N, 1), zF + r*sin(th)];
end

function [V,F] = pushTri(V,F,a,b,c)
i0 = size(V,1);
V(end+1:end+3,:) = [a; b; c];
F(end+1,:) = i0 + (1:3);
end

function [V,F] = pushQuad(V,F,a,b,c,d)
i0 = size(V,1);
V(end+1:end+4,:) = [a; b; c; d];
F(end+1:end+2,:) = [i0+1 i0+2 i0+3; i0+1 i0+3 i0+4];
end

function [V,F] = loftRings(V,F,rA,rB)
n = size(rA,1);
for k = 1:n
    k2 = mod(k,n)+1;
    [V,F] = pushQuad(V,F, rA(k,:), rA(k2,:), rB(k2,:), rB(k,:));
end
end

function [V,F] = fanCap(V,F,ring,xC,zF)
ctr = [xC, ring(1,2), zF];
n = size(ring,1);
for k = 1:n
    k2 = mod(k,n)+1;
    [V,F] = pushTri(V,F, ctr, ring(k,:), ring(k2,:));
end
end

function [V,F] = extrudeLoopWalls(V, F, L, bandY)
% 轮廓线沿 y 挤出成壁 (分带)
n = size(L,1);
for e = 1:n
    e2 = mod(e,n)+1;
    p1 = L(e,:);  p2 = L(e2,:);
    for b = 1:numel(bandY)-1
        % [x, y, z] 顺序 (p=(x,z))
        [V,F] = pushQuad(V,F, [p1(1) bandY(b) p1(2)], [p2(1) bandY(b) p2(2)], ...
                            [p2(1) bandY(b+1) p2(2)], [p1(1) bandY(b+1) p1(2)]);
    end
end
end

function c = ductCorners(x, wYdfun, zLofun, zHifun)
wy = wYdfun(x); zl = zLofun(x); zh = zHifun(x);
c = [-wy zl; wy zl; wy zh; -wy zh];
end

function c = tunnelCorners(x, S, HY, HZ)
hy = interp1(S, HY, x);  hz = interp1(S, HZ, x);
c = [-hy -hz; hy -hz; hy hz; -hy hz];
end

function [V,F] = fillCap(loops, keepFn, y, V, F, xC, zF)
% 多环约束 Delaunay 平面帽 (2D xz 剖分, keepFn 过滤, 抬升到 y 平面)
Pall = []; C = []; off = 0;
for k = 1:numel(loops)
    Lg = loops{k};  n = size(Lg,1);
    Pall = [Pall; Lg]; %#ok<AGROW>
    e1 = off + (1:n)';  e2 = off + mod((1:n)', n) + 1;
    C = [C; e1 e2]; %#ok<AGROW>
    off = off + n;
end
sd = 0.004;
xs = (xC-0.56):sd:2.45;  zs = (zF-0.56):sd:(zF+0.56);
[Xq, Zq] = meshgrid(xs, zs);
in = keepFn(Xq, Zq);
seeds = [Xq(in), Zq(in)];
DT = delaunayTriangulation([Pall; seeds], C);
T = DT.ConnectivityList;
Pt = DT.Points;
fprintf('DEBUG fillCap: 输入点 %d -> DT 点 %d | 约束 %d | 三角 %d\n', ...
    size([Pall;seeds],1), size(Pt,1), size(C,1), size(T,1));
cx = (Pt(T(:,1),1) + Pt(T(:,2),1) + Pt(T(:,3),1)) / 3;
cz = (Pt(T(:,1),2) + Pt(T(:,2),2) + Pt(T(:,3),2)) / 3;
keepM = find(keepFn(cx, cz));
nV = size(V,1);
V = [V; Pt(:,1), repmat(y, size(Pt,1),1), Pt(:,2)];
F = [F; T(keepM,:) + nV];
end

function writeSTLascii(filepath, varargin)
% ASCII STL 多 solid 写出: varargin = {'name', V, F, 'name', V, F, ...}
assert(mod(numel(varargin), 3) == 0, 'writeSTLascii: 参数须为 name/V/F 三元组');
fid = fopen(filepath, 'w');
for k = 1:3:numel(varargin)
    nm = varargin{k};  V = varargin{k+1};  F = varargin{k+2};
    fprintf(fid, 'solid %s\n', nm);
    for i = 1:size(F,1)
        p = V(F(i,:), :);
        u = p(2,:) - p(1,:);  w = p(3,:) - p(1,:);
        n = cross(u, w);  nl = norm(n);
        if nl > 0, n = n / nl; else, n = [0 0 1]; end
        fprintf(fid, ' facet normal %.6e %.6e %.6e\n  outer loop\n', n);
        for j = 1:3
            fprintf(fid, '   vertex %.6f %.6f %.6f\n', p(j,:));
        end
        fprintf(fid, '  endloop\n endfacet\n');
    end
    fprintf(fid, 'endsolid %s\n', nm);
end
fclose(fid);
end

function writeDicts(CASE, xC, zF, omega, rpm, r2, yPlTop, OBJ, OBJ_R, OBJ_C)
% 被测物条件条目 (OBJ='none' 时为空串)
useObj = ~strcmpi(OBJ, 'none');
% ---- blockMeshDict: 背景盒 (帽面不贴盒面留 20mm; y0/z0 错位避共面: 20mm 栅格与
%      轴对齐壁面 y=±0.15/±0.3675, z=±0.10/±0.135/±0.13/±0.245 均不共面 → 阻断面判定有效) ----
x0=-1.13; x1=2.52; y0=-0.395; y1=yPlTop+0.02; z0=-0.273; z1=zF+0.55;
nx=round((x1-x0)/0.02); ny=round((y1-y0)/0.02); nz=round((z1-z0)/0.02);
fid = fopen(fullfile(CASE,'system','blockMeshDict'),'w');
fprintf(fid, ['FoamFile\n{\n    version 2.0;\n    format ascii;\n    class dictionary;\n' ...
    '    object blockMeshDict;\n}\n// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //\n\n']);
fprintf(fid, ['convertToMeters 1;\n\nvertices\n(\n' ...
    '    (%.4f %.4f %.4f)\n    (%.4f %.4f %.4f)\n    (%.4f %.4f %.4f)\n    (%.4f %.4f %.4f)\n' ...
    '    (%.4f %.4f %.4f)\n    (%.4f %.4f %.4f)\n    (%.4f %.4f %.4f)\n    (%.4f %.4f %.4f)\n);\n'], ...
    x0,y0,z0, x1,y0,z0, x1,y1,z0, x0,y1,z0, x0,y0,z1, x1,y0,z1, x1,y1,z1, x0,y1,z1);
fprintf(fid, '\nblocks\n(\n    hex (0 1 2 3 4 5 6 7) (%d %d %d) simpleGrading (1 1 1)\n);\n\n', nx, ny, nz);
fprintf(fid, ['edges\n(\n);\n\nboundary\n(\n' ...
    '    inlet\n    {\n        type patch;\n        faces\n        (\n            (3 2 6 7)\n        );\n    }\n' ...
    '    outlet\n    {\n        type patch;\n        faces\n        (\n            (1 5 6 2)\n        );\n    }\n' ...
    '    boxUnused\n    {\n        type patch;\n        faces\n        (\n            (0 4 5 1)\n            (4 7 6 5)\n            (0 3 7 4)\n            (0 1 2 3)\n        );\n    }\n);\n\nmergePatchPairs\n(\n);\n\n']);
fclose(fid);

% ---- snappyHexMeshDict (v1912 模板结构: #includeEtc cfg 提供默认值) ----
fid = fopen(fullfile(CASE,'system','snappyHexMeshDict'),'w');
% org v12: 以实测验证过的模板为基底 (#includeEtc cfg 提供默认值),
% 被测物启用时在 geometry 与 refinementSurfaces 注入 testobj 条目
tpl = fileread(fullfile('D:\WindTunnelSuite\WindTunnel3D\openfoam', 'snappyHexMeshDict.v12.template'));
objGeo = '';  objSurf = '';
if useObj
    % 模板已清成无 testobj 基底 (2026-10-05); 注入是唯一来源, 锚在 impeller 行后;
    % v12 必需 file 键; 替换串用 char(10) 拼接, 禁止源码内多行字符串字面量
    objGeo  = sprintf('    testobj.stl { type triSurfaceMesh; file "testobj.stl"; name testobj; }%s', char(10));
    objSurf = sprintf('    testobj { level (2 2); patchInfo { type wall; } }%s', char(10));
    tpl = regexprep(tpl, '(impeller\.stl \{ type triSurfaceMesh; file "impeller\.stl"; name impeller; \}\n)', ['$1' char(10) objGeo]);
    tpl = regexprep(tpl, '(    impeller \{ level \(2 2\); patchInfo \{ type wall; \} \}\n)', ['$1' char(10) objSurf]);
end
fprintf(fid, '%s', tpl);
fclose(fid);

% meshQualityDict: v12 snappy 不读此文件 (v1912 行为)! 实际覆盖在 snappyHexMeshDict.v12.template
% 的 meshQualityControls 段 (maxBoundarySkewness 4/maxInternalSkewness 3, T3 治理)。留档无害。
fmq = fopen(fullfile(CASE,'system','meshQualityDict'),'w');
fprintf(fmq, '#includeEtc "caseDicts/mesh/generation/meshQualityDict.cfg"\n');
fclose(fmq);

fid = fopen(fullfile(CASE,'system','topoSetDict'),'w');
fprintf(fid, ['FoamFile\n{\n    version 2.0;\n    format ascii;\n    class dictionary;\n    object topoSetDict;\n}\n\n' ...
    'actions\n(\n' ...
    '    {\n        name    rotorZone;\n        type    cellSet;\n        action  new;\n        source  cylinderToCell;\n        p1      (%.4f 0.1225 %.4f);\n        p2      (%.4f -0.1185 %.4f);\n        radius  %.4f;\n    }\n' ...
    '    {\n        name    tsZone;\n        type    cellSet;\n        action  new;\n        source  boxToCell;\n        box     (1.56 -0.145 -0.095) (1.74 0.145 0.095);\n    }\n);\n\n'], ...
    xC, zF, xC, zF, r2+0.002);
fclose(fid);

fid = fopen(fullfile(CASE,'constant','MRFProperties'),'w');
% org v12 格式 (mixerVessel2DMRF 教程同款): 顶层键=zone 名, cellZone 指向 cellZone;
% omega 带量纲 [rpm]; 负号=叶片 +φ 扫与蜗壳螺旋配对 = 正向泵送 (2026-10-04 实证);
% 搭配 0/U impeller 的 MRFnoSlip 才有泵送 (普通 noSlip = 静止壁, 源项自抵消)
fprintf(fid, ['FoamFile\n{\n    version 2.0;\n    format ascii;\n    class dictionary;\n    location "constant";\n    object MRFProperties;\n}\n\n' ...
    'MRF\n{\n    cellZone    rotorZone;\n' ...
    '    origin      (%.4f 0 %.4f);\n    axis        (0 1 0);\n' ...
    '    omega       %.1f [rpm];\n}\n'], ...
    xC, zF, -double(rpm));
fclose(fid);

fid = fopen(fullfile(CASE,'constant','transportProperties'),'w');
fprintf(fid, ['FoamFile\n{\n    version 2.0;\n    format ascii;\n    class dictionary;\n    location "constant";\n    object transportProperties;\n}\n\n' ...
    'transportModel  Newtonian;\n\nnu              [0 2 -1 0 0 0 0] 1.516e-05;\n\n']);
fclose(fid);
fid = fopen(fullfile(CASE,'constant','turbulenceProperties'),'w');
fprintf(fid, ['FoamFile\n{\n    version 2.0;\n    format ascii;\n    class dictionary;\n    location "constant";\n    object turbulenceProperties;\n}\n\n' ...
    'simulationType  RAS;\n\nRAS\n{\n    RASModel        kOmegaSST;\n    turbulence      on;\n    printCoeffs     on;\n}\n']);
fclose(fid);

fid = fopen(fullfile(CASE,'0','U'),'w');
objU = '';  if useObj, objU = sprintf('    testobj   { type noSlip; }\n'); end
fprintf(fid, ['FoamFile\n{\n    version 2.0;\n    format ascii;\n    class volVectorField;\n    object U;\n}\n\n' ...
    'dimensions      [0 1 -1 0 0 0 0];\n\ninternalField   uniform (0 0 0);\n\nboundaryField\n{\n' ...
    '    skin_inletCap\n    {\n        type            pressureInletOutletVelocity;\n        value           uniform (0 0 0);\n    }\n' ...
    '    skin_outletCap\n    {\n        type            pressureInletOutletVelocity;\n        value           uniform (0 0 0);\n    }\n' ...
    '    casing    { type noSlip; }\n    impeller  { type MRFnoSlip; }   // org v12: 旋转壁必须用此 BC (2026-10-04 实证)\n' ...
    '    inlet     { type noSlip; }\n    outlet    { type noSlip; }\n    boxUnused { type noSlip; }\n%s}\n'], objU);
fclose(fid);
fid = fopen(fullfile(CASE,'0','p'),'w');
objP = '';  if useObj, objP = sprintf('    testobj   { type zeroGradient; }\n'); end
fprintf(fid, ['FoamFile\n{\n    version 2.0;\n    format ascii;\n    class volScalarField;\n    object p;\n}\n\n' ...
    'dimensions      [0 2 -2 0 0 0 0];\n\ninternalField   uniform 0;\n\nboundaryField\n{\n' ...
    '    skin_inletCap\n    {\n        type            totalPressure;\n        p0              uniform 0;\n        value           uniform 0;\n    }\n' ...
    '    skin_outletCap\n    {\n        type            fixedValue;\n        value           uniform 0;\n    }\n' ...
    '    casing    { type zeroGradient; }\n    impeller  { type zeroGradient; }\n' ...
    '    inlet     { type zeroGradient; }\n    outlet    { type zeroGradient; }\n    boxUnused { type zeroGradient; }\n%s}\n'], objP);
fclose(fid);
fid = fopen(fullfile(CASE,'0','k'),'w');
objK = '';  if useObj, objK = sprintf('    testobj   { type kqRWallFunction; value uniform 0.6; }\n'); end
fprintf(fid, ['FoamFile\n{\n    version 2.0;\n    format ascii;\n    class volScalarField;\n    object k;\n}\n\n' ...
    'dimensions      [0 2 -2 0 0 0 0];\n\ninternalField   uniform 0.6;\n\nboundaryField\n{\n' ...
    '    skin_inletCap\n    {\n        type            turbulentIntensityKineticEnergyInlet;\n        intensity       0.05;\n        value           uniform 0.6;\n    }\n' ...
    '    skin_outletCap\n    {\n        type            inletOutlet;\n        inletValue      uniform 0.6;\n        value           uniform 0.6;\n    }\n' ...
    '    casing    { type kqRWallFunction; value uniform 0.6; }\n    impeller  { type kqRWallFunction; value uniform 0.6; }\n' ...
    '    inlet     { type kqRWallFunction; value uniform 0.6; }\n    outlet    { type kqRWallFunction; value uniform 0.6; }\n' ...
    '    boxUnused { type kqRWallFunction; value uniform 0.6; }\n%s}\n'], objK);
fclose(fid);
fid = fopen(fullfile(CASE,'0','omega'),'w');
objO = '';  if useObj, objO = sprintf('    testobj   { type omegaWallFunction; value uniform 500; }\n'); end
fprintf(fid, ['FoamFile\n{\n    version 2.0;\n    format ascii;\n    class volScalarField;\n    object omega;\n}\n\n' ...
    'dimensions      [0 0 -1 0 0 0 0];\n\ninternalField   uniform 500;\n\nboundaryField\n{\n' ...
    '    skin_inletCap  { type fixedValue; value uniform 500; }\n' ...
    '    skin_outletCap { type inletOutlet; inletValue uniform 500; value uniform 500; }\n' ...
    '    casing    { type omegaWallFunction; value uniform 500; }\n    impeller  { type omegaWallFunction; value uniform 500; }\n' ...
    '    inlet     { type omegaWallFunction; value uniform 500; }\n    outlet    { type omegaWallFunction; value uniform 500; }\n' ...
    '    boxUnused { type omegaWallFunction; value uniform 500; }\n%s}\n'], objO);
fclose(fid);
fid = fopen(fullfile(CASE,'0','nut'),'w');
objN = '';  if useObj, objN = sprintf('    testobj   { type nutkWallFunction; value uniform 0; }\n'); end
fprintf(fid, ['FoamFile\n{\n    version 2.0;\n    format ascii;\n    class volScalarField;\n    object nut;\n}\n\n' ...
    'dimensions      [0 2 -1 0 0 0 0];\n\ninternalField   uniform 0;\n\nboundaryField\n{\n' ...
    '    skin_inletCap  { type calculated; value uniform 0; }\n' ...
    '    skin_outletCap { type calculated; value uniform 0; }\n' ...
    '    casing    { type nutkWallFunction; value uniform 0; }\n    impeller  { type nutkWallFunction; value uniform 0; }\n' ...
    '    inlet     { type nutkWallFunction; value uniform 0; }\n    outlet    { type nutkWallFunction; value uniform 0; }\n' ...
    '    boxUnused { type nutkWallFunction; value uniform 0; }\n%s}\n'], objN);
fclose(fid);

fid = fopen(fullfile(CASE,'system','controlDict'),'w');
% org v12 controlDict: 以实测验证过的模板为基底 (select/interpolationScheme 已 v12 化),
% 被测物启用时插 forces FO; startFrom/endTime 重置为脚本可控默认
tpl = fileread(fullfile('D:\WindTunnelSuite\WindTunnel3D\openfoam', 'controlDict.v12.template'));
objForces = '';
if useObj
    objForces = sprintf([ ...
        '    forces\n    {\n        type            forces;\n' ...
        '        libs            ("libforces.so");\n' ...
        '        writeControl    timeStep;\n        timeInterval    25;\n        log             yes;\n' ...
        '        patches         ("testobj");\n        rho             rhoInf;\n        rhoInf          1.2;\n' ...
        '        CofR            (%.3f 0 0);\n        pitchAxis       (0 1 0);\n    }\n'], OBJ_C);
end
tpl = regexprep(tpl, 'functions\s*\{', ['functions\n{' newline objForces]);
% ^ 锚 + lineanchors: endTime 只替换行首那条, 否则会命中 stopAt 行内的 endTime; 造成粘连 (2026-10-05 实锤)
tpl = regexprep(tpl, '^startFrom[^\n]*', 'startFrom       startTime;', 'lineanchors');
tpl = regexprep(tpl, '^endTime[^\n]*',  'endTime         3000;', 'lineanchors');
fprintf(fid, '%s', tpl);
fclose(fid);

copyfile('D:\WindTunnelSuite\openfoam\windtunnel-base\system\fvSchemes', fullfile(CASE,'system','fvSchemes'));
copyfile('D:\WindTunnelSuite\openfoam\windtunnel-base\system\fvSolution', fullfile(CASE,'system','fvSolution'));

% Ubuntu deb 版 v1912 无 scotch (dummyScotchDecomp) — 用 hierarchical; v1912 从顶层读 n
for np = [6 12]
    if np == 6, nstr = '(3 2 1)'; else, nstr = '(4 3 1)'; end
    fid = fopen(fullfile(CASE,'system',sprintf('decomposeParDict.%d',np)),'w');
    fprintf(fid, ['FoamFile\n{\n    version 2.0;\n    format ascii;\n    class dictionary;\n    object decomposeParDict;\n}\n\n' ...
        'numberOfSubdomains %d;\n\nmethod hierarchical;\n\nn %s;\n\ndelta 0.001;\n\n'], np, nstr);
    fclose(fid);
end
copyfile(fullfile(CASE,'system','decomposeParDict.6'), fullfile(CASE,'system','decomposeParDict'));

% patch 名加 surface 前缀 (v1912 实际生成 <surface>_<region>: skin_casing 等);
% 只改 0/场 + MRF + controlDict, snappyHexMeshDict 的 region 键名 (=STL solid 名) 不动
for fn = {fullfile(CASE,'0','U'), fullfile(CASE,'0','p'), fullfile(CASE,'0','k'), ...
          fullfile(CASE,'0','omega'), fullfile(CASE,'0','nut'), ...
          fullfile(CASE,'constant','MRFProperties'), fullfile(CASE,'system','controlDict')}
    txt = fileread(fn{1});
    txt = regexprep(txt, '\<inletCap\>', 'skin_inletCap');
    txt = regexprep(txt, '\<outletCap\>', 'skin_outletCap');
    txt = regexprep(txt, '\<casing\>', 'skin_casing');
    fid2 = fopen(fn{1}, 'w');  fwrite(fid2, txt);  fclose(fid2);
end
end
