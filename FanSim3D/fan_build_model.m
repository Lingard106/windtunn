%% fan_build_model — 风机 3D 建模 + 设计检查图 + STL 导出 (3D 打印用)
% 输出: fan_model.png (3D 渲染+叶片截面+蜗壳型线+p-Q 曲线+速度三角形+轴向布局)
%       impeller.stl (叶轮: 后盘+叶片+前盘) / volute.stl (蜗壳+侧板+出口短管+钟形口)
clear; clc; close all;
set(groot,'defaultFigureColor','w','defaultAxesColor','w', ...
    'defaultAxesXColor',[.15 .15 .15],'defaultAxesYColor',[.15 .15 .15], ...
    'defaultAxesZColor',[.15 .15 .15],'defaultTextColor',[.15 .15 .15]);

P = fan_params();
G = fan_geometry(P);
R = G.R;
Lx = G.Lx;

%% ==================== 面片构建 (四边形面片 → 三角形) ====================
bl = G.blade;  vol = G.vol;
mqI = quadsEmpty();  mqV = quadsEmpty();   % 叶轮 / 蜗壳 (quad 索引累积)

% ---- 叶片 ×Z (实心: 内外偏置圆弧带 + 两端盖, 沿 x 挤出) ----
nA = 40;
tArc = linspace(bl.th(1), bl.th(2), nA)';
Po = bl.C + (bl.Ra+bl.tB/2)*[cosd(tArc), sind(tArc)];
Pi = bl.C + (bl.Ra-bl.tB/2)*[cosd(tArc), sind(tArc)];
prof = [Po; flipud(Pi)];                    % 环带剖面 (外弧去+内弧回)
for kZ = 0:bl.Z-1
    a0 = kZ*360/bl.Z;
    Rm = [cosd(a0) -sind(a0); sind(a0) cosd(a0)];
    mqI = catQuads(mqI, extrudeBand(prof*Rm', Lx.xBlade));
end

% ---- 后盘 / 前盘 (环板) ----
mqI = catQuads(mqI, annulusQuads(P.imp.hubR, bl.r2, Lx.xRear, 72));
mqI = catQuads(mqI, annulusQuads(G.r0eye, bl.r2, Lx.xFront, 72));

% ---- 电机毂壳 (叶片内缘圆柱: 从后盘到下游板, 对应 CFD 的毂实体 — 电机装这里) ----
nH = 60;
thH = linspace(0, 360, nH+1)';
mqV = catQuads(mqV, cylinderSideQuads(bl.r1, [Lx.xRear(1), Lx.xSideDn(2)], nH));

% ---- 蜗壳螺旋壁 (阿基米德带, 叶宽段) ----
nT = 144;
thV = linspace(0, 360, nT+1)';
rIn  = vol.rTongue + vol.a*thV*pi/180;
rOut = rIn + vol.tW;
mqV = catQuads(mqV, spiralBandQuads(thV, rIn, rOut, Lx.xBlade));

% ---- 上游侧板 (眼孔) / 下游侧板 (出口方孔) ----
mpUp = plateWithHoles(vol.Rvol, Lx.xSideUp, 'circle', G.r0eye);
mpDn = plateWithHoles(vol.Rvol, Lx.xSideDn, 'rect', [Lx.W_out, Lx.H_out]);

% ---- 出口短管 (带壁厚方管) + 钟形口锥带 ----
mqV = catQuads(mqV, rectTubeQuads(Lx.W_out, Lx.H_out, Lx.xStub, vol.tW));
nB = 48;  tt = linspace(0, 1, 16)';
rrB = Lx.Rbell + (G.r0eye - Lx.Rbell)*tt;
mqV = catQuads(mqV, coneBandQuads(rrB, Lx.xBell(1) + diff(Lx.xBell)*tt, nB));

%% ==================== 转三角形 + STL 导出 ====================
MI = quadsToTris(mqI);
MV = quadsToTris(mqV);
MV = catMesh(MV, mpUp);  MV = catMesh(MV, mpDn);
writeBinarySTL(MI, fullfile(pwd,'impeller.stl'));
writeBinarySTL(MV, fullfile(pwd,'volute.stl'));
fprintf('impeller.stl (%d 三角形) / volute.stl (%d 三角形) 已导出\n', ...
    size(MI.F,1), size(MV.F,1));
fprintf('  叶轮 D2=%.0fmm 宽 %.0fmm 叶片厚 %.1fmm (打印: 层高 0.12mm, 4 遍壁, 叶尖配合间隙 %.1fmm)\n', ...
    P.imp.D2*1e3, diff(Lx.xBlade)*1e3, bl.tB*1e3, P.imp.clearR*1e3);
vImp = (pi*(bl.r2^2-P.imp.hubR^2) + pi*(bl.r2^2-G.r0eye^2)) * P.imp.shroudT ...
     + bl.Lb * bl.tB * diff(Lx.xBlade) * bl.Z;      % 解析: 环板×2 + 叶片×Z
mi = meshVolume(MI);
fprintf('  叶轮材料体积 ≈%.0f cm³ (PLA≈%.0f g, 网格体积 %.0f cm³ 仅供参考—双面面片不守恒)\n', ...
    vImp*1e6, vImp*1e6*1.24, mi*1e6);

%% ==================== 设计检查图 ====================
figure('Color','w','Position',[40 40 1500 860]);

% -- (1) 3D 渲染 --
subplot(2,3,[1 4]); hold on;
addPatch(MV, [0.75 0.78 0.82], 0.45);
addPatch(MI, [0.85 0.45 0.15], 1.0);
thv3 = linspace(0, 360, 200)';
rv3 = vol.rTongue + vol.a*thv3*pi/180;
plot3(Lx.xBlade(1)*ones(size(thv3)), rv3.*cosd(thv3), rv3.*sind(thv3), 'r-', 'LineWidth', 1.6);
text(Lx.xBlade(1)+0.05, 0.0, -0.60, '叶轮 (红=蜗壳型线)', 'FontSize', 9, 'Color', [0.8 0.2 0.1]);
text(0.15, 0.62, 0.30, '蜗壳 + 侧板', 'FontSize', 9);
text(Lx.xFlange+0.03, 0.15, 0.22, '出口法兰 → 风洞前置扩散段', 'FontSize', 9, 'Color', [0 0.3 0.7]);
text(0.01, -0.28, -0.20, '钟形吸入口', 'FontSize', 9);
axis equal; view(128, 20); camlight headlight; lighting gouraud;
grid on; xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
title(sprintf('抽风式离心风机 %s | D_2=%.0fmm Z=%d \\beta_{2A}=%.0f° n=%.0frpm', ...
    P.preset, P.imp.D2*1e3, P.imp.Z, bl.beta2A, P.imp.rpm));

% -- (2) 叶片截面 (y-z 视图 + 蜗壳型线) --
subplot(2,3,2); hold on; axis equal;
thAll = linspace(bl.th(1), bl.th(2), 60)';
for kZ = 0:bl.Z-1
    a0 = kZ*360/bl.Z;
    Rm = [cosd(a0) -sind(a0); sind(a0) cosd(a0)];
    xr = bl.C + (bl.Ra+bl.tB/2)*[cosd(thAll), sind(thAll)];
    xi = bl.C + (bl.Ra-bl.tB/2)*[cosd(thAll), sind(thAll)];
    M2 = [xr; flipud(xi)]*Rm;
    plot(M2(:,1), M2(:,2), 'b-', 'LineWidth', 0.8);
end
rectangle('Position',[-bl.r2 -bl.r2 2*bl.r2 2*bl.r2], 'Curvature',[1 1], 'EdgeColor','k','LineStyle',':');
rectangle('Position',[-G.r0eye -G.r0eye 2*G.r0eye 2*G.r0eye], 'Curvature',[1 1], 'EdgeColor','r','LineStyle',':');
rectangle('Position',[-bl.r1 -bl.r1 2*bl.r1 2*bl.r1], 'Curvature',[1 1], 'EdgeColor','m','LineStyle',':');
thVr = linspace(0, 360, 200)';
rv = vol.rTongue + vol.a*thVr*pi/180;
Ms = [rv.*cosd(thVr), rv.*sind(thVr)];
plot(Ms(:,1), Ms(:,2), 'k-', 'LineWidth', 1.3);
Mt = [vol.rTongue*cosd(thVr), vol.rTongue*sind(thVr)];
plot(Mt(:,1), Mt(:,2), 'k:', 'LineWidth', 0.8);
plot(0, 0, 'k+', 'MarkerSize', 10);
xlim([-0.60 0.60]); ylim([-0.60 0.60]);
title('叶片截面×Z (蓝) + 蜗壳型线 (黑) | 眼(红) D_1(品红)');
xlabel('y (m)'); ylabel('z (m)');

% -- (3) p-Q 性能曲线 + 工况点 --
subplot(2,3,3); hold on; grid on;
plot(R.Q, R.p_sf, 'b-', 'LineWidth', 1.6); plot(R.Q, R.load.dp(R.Q), 'r--', 'LineWidth', 1.4);
if isfinite(R.op.Q)
    plot(R.op.Q, R.op.p, 'ko', 'MarkerFaceColor', 'y', 'MarkerSize', 8);
    text(R.op.Q*0.98, R.op.p*1.06, sprintf('  Q=%.2f m³/s  V_{ts}=%.1f m/s\n  η=%.0f%%  P=%.0f W', ...
        R.op.Q, R.op.Vts, R.op.etaS*100, R.op.P), 'FontWeight', 'bold');
end
xlabel('Q (m³/s)'); ylabel('\Delta p (Pa)');
legend('风机静压 p_{sf}(Q)', '风洞阻力 K·Q²', 'Location', 'northeast');
title(sprintf('性能匹配 | 喘振裕度 %.0f%%', R.stab.surgeMargin*100));

% -- (4) 出口速度三角形 --
subplot(2,3,5); hold on; axis equal; grid on;
tri = R.tri;
quiver(0, 0, tri.u2, 0, 0, 'b', 'LineWidth', 1.8, 'MaxHeadSize', 0.5);
quiver(0, 0, tri.c2u, tri.c2m, 0, 'r', 'LineWidth', 1.8, 'MaxHeadSize', 0.5);
quiver(tri.c2u, tri.c2m, tri.u2-tri.c2u, -tri.c2m, 0, 'g', 'LineWidth', 1.4, 'MaxHeadSize', 0.5);
legend({'u_2 叶尖', 'c_2 绝对', 'w_2 相对'}, 'Location', 'best');
axis([-0.15 1.2*max(tri.u2, tri.c2u) -0.06 1.3*max(tri.c2m, 0.5)]);
title(sprintf('出口速度三角形 | \\beta_{2A}=%.0f°', bl.beta2A));
xlabel('切向 (m/s)'); ylabel('径向 (m/s)');

% -- (5) 轴向布局 --
subplot(2,3,6); hold on;
segs = {Lx.xBell,'钟形口',[.5 .7 .9]; Lx.xEye,'吸力眼',[.8 .8 .8]; ...
    Lx.xRear,'后盘',[.9 .5 .2]; Lx.xBlade,'叶片',[.95 .6 .1]; Lx.xFront,'前盘',[.9 .5 .2]; ...
    Lx.xSideUp,'上游侧板',[.6 .6 .65]; Lx.xSideDn,'下游侧板',[.6 .6 .65]; ...
    Lx.xStub,'出口短管',[.5 .75 .5]; Lx.xDiff,'前置扩散段',[.4 .7 .85]; Lx.xSettle,'稳定段短管',[.4 .7 .85]};
for k = 1:size(segs,1)
    s = segs{k,1};
    patch([s(1) s(2) s(2) s(1)], [0 0 1 1], segs{k,3}, 'EdgeColor','none', 'FaceAlpha',.9);
end
ylim([0 1]); xlim([-0.02, Lx.xSettle(2)*1.02]);
title('轴向布局 (法兰=风洞前置扩散段入口)'); xlabel('x (m)');

saveas(gcf, fullfile(pwd, 'fan_model.png'));
fprintf('fan_model.png 已保存\n');

%% ==================== 局部函数: 四边形面片工具 ====================
function Q = quadsEmpty()
Q = struct('V',zeros(0,3),'Q',zeros(0,4));
end

function A = catQuads(A, B)
if isempty(B.V), A = B; return; end
if isempty(A.V), A = B; return; end
o = size(A.V,1);
A.V = [A.V; B.V];  A.Q = [A.Q; B.Q + o];
end

function M = quadsToTris(A)
F = [A.Q(:,[1 2 3]); A.Q(:,[1 3 4])];
M = struct('V',A.V, 'F',F);
end

function Q = extrudeBand(prof2d, xRange)
% 环带剖面 (2m×2: 外弧去+内弧回) 沿 x 挤出: 侧面四边形 + 带格端盖
m = size(prof2d,1);  h = m/2;                 % h = 单弧点数
V = [xRange(1)*ones(m,1), prof2d; xRange(2)*ones(m,1), prof2d];
Q = [];
for i = 1:m
    j = mod(i, m) + 1;
    Q = [Q; i, j, h+j, h+i]; %#ok<AGROW>      % 侧面
end
for i = 1:h-1                                  % 端盖: 外弧 i..i+1 与内弧 (m+1-i)
    io = i;  ii = m+1-i;  jo = i+1;  ji = m-i;
    Q = [Q; io, jo, ji, ii]; %#ok<AGROW>       % x1 端
    Q = [Q; h+io, h+jo, h+ji, h+ii]; %#ok<AGROW> % x2 端
end
Q = struct('V',V, 'Q',Q);
end

function Q = cylinderSideQuads(r, xRange, nTh)
% 圆柱侧面 (电机毂壳): r 恒定, x 从 xRange(1) 到 xRange(2)
th = linspace(0, 360, nTh+1)';  th(end) = [];
ci = [cosd(th), sind(th)];
V = [xRange(1)*ones(nTh,1), r*ci; xRange(2)*ones(nTh,1), r*ci];
Q = [];
for i = 1:nTh
    j = mod(i, nTh) + 1;
    Q = [Q; i, j, nTh+j, nTh+i]; %#ok<AGROW>
end
Q = struct('V', V, 'Q', Q);
end

function Q = annulusQuads(rIn, rOut, xRange, nTh)
% 环板: 内外壁 + 两环端面
th = linspace(0, 360, nTh+1)';  th(end) = [];
ci = [cosd(th), sind(th)];
V = [xRange(1)*ones(nTh,1), rIn*ci;  xRange(1)*ones(nTh,1), rOut*ci; ...
     xRange(2)*ones(nTh,1), rIn*ci;  xRange(2)*ones(nTh,1), rOut*ci];
Q = [];
for i = 1:nTh
    j = mod(i, nTh) + 1;
    Q = [Q; i, j, nTh+j, nTh+i];              %#ok<AGROW>  x1 端面 (环)
    Q = [Q; 2*nTh+i, 2*nTh+j, 3*nTh+j, 3*nTh+i]; %#ok<AGROW>  x2 端面
    Q = [Q; i, 2*nTh+i, 2*nTh+j, j];          %#ok<AGROW>  内壁
    Q = [Q; nTh+i, 3*nTh+i, 3*nTh+j, nTh+j];  %#ok<AGROW>  外壁
end
Q = struct('V',V, 'Q',Q);
end

function Q = spiralBandQuads(thV, rIn, rOut, xRange)
% 蜗壳带: 内外面 (沿θ) + 两端径向条
nT = numel(thV);
V = [];
for kx = 1:2
    x = xRange(kx);
    V = [V; x*ones(nT,1), rIn.*[cosd(thV), sind(thV)]; ...
         x*ones(nT,1), rOut.*[cosd(thV), sind(thV)]]; %#ok<AGROW>
end
Q = [];
for i = 1:nT-1
    Q = [Q; i, i+1, 2*nT+i+1, 2*nT+i];        %#ok<AGROW>  内面
    Q = [Q; nT+i, nT+i+1, 3*nT+i+1, 3*nT+i];  %#ok<AGROW>  外面
end
for i = [1, nT]                               % 舌端/终端
    Q = [Q; i, nT+i, 3*nT+i, 2*nT+i];         %#ok<AGROW>
end
Q = struct('V',V, 'Q',Q);
end

function Q = rectTubeQuads(W, H, xRange, tW)
po = [W/2 H/2; -W/2 H/2; -W/2 -H/2; W/2 -H/2];
pi_ = [(W/2-tW) (H/2-tW); -(W/2-tW) (H/2-tW); -(W/2-tW) -(H/2-tW); (W/2-tW) -(H/2-tW)];
A = extrudeBand([po; flipud(pi_)], xRange);   % 外/内环带 (中空管)
Q = A;
end

function Q = coneBandQuads(rr, xx, nTh)
th = linspace(0, 360, nTh+1)';  th(end) = [];
ci = [cosd(th), sind(th)];
n = numel(rr);
V = [];
for k = 1:n
    V = [V; xx(k)*ones(nTh,1), rr(k)*ci]; %#ok<AGROW>
end
Q = [];
for k = 1:n-1
    for i = 1:nTh
        j = mod(i, nTh)+1;
        a = (k-1)*nTh+i; b = (k-1)*nTh+j; c = k*nTh+j; d = k*nTh+i;
        Q = [Q; a, b, c, d]; %#ok<AGROW>
    end
end
Q = struct('V',V, 'Q',Q);
end

function M = plateWithHoles(Rout, xRange, hType, hParam)
% 圆板 (中心圆孔/矩形孔) — 约束 Delaunay + 质心过滤, 挤出成板
th = linspace(0, 360, 73)';  th(end) = [];
outer = Rout*[cosd(th), sind(th)];
switch hType
    case 'circle'
        ih = linspace(0, 360, 49)';  ih(end) = [];
        hole = hParam*[cosd(ih), sind(ih)];
    case 'rect'
        hw = hParam(1)/2;  hh = hParam(2)/2;
        hole = [hw hh; -hw hh; -hw -hh; hw -hh];
end
n1 = size(outer,1);  n2 = size(hole,1);
P = [outer; hole];
C = [[(1:n1)', [2:n1, 1]']; [n1+(1:n2)', n1+[2:n2, 1]']];
dt = delaunayTriangulation(P, C);
T = dt.ConnectivityList;
Pt = dt.Points;
cx = (Pt(T(:,1),:) + Pt(T(:,2),:) + Pt(T(:,3),:))/3;
rC = hypot(cx(:,1), cx(:,2));
inHole = false(size(rC));
switch hType
    case 'circle', inHole = rC < hParam*0.999;
    case 'rect',   inHole = abs(cx(:,1)) < hParam(1)/2*0.999 & abs(cx(:,2)) < hParam(2)/2*0.999;
end
T = T(rC <= Rout*1.001 & ~inHole, :);
x1 = xRange(1);  x2 = xRange(2);
n = size(Pt,1);
V = [[x1*ones(n,1), Pt]; [x2*ones(n,1), Pt]];
F = [T; T + n];
E = [T(:,[1 2]); T(:,[2 3]); T(:,[3 1])];
[uc, ~, ic] = unique(sort(E, 2), 'rows');
counts = accumarray(ic, 1);
Eb = uc(counts == 1, :);
side = [];
for k = 1:size(Eb,1)
    i = Eb(k,1);  j = Eb(k,2);
    side = [side; i, j, n+j; i, n+j, n+i]; %#ok<AGROW>
end
M = struct('V',V, 'F',[F; side]);
end

function M = catMesh(M1, M2)
if isempty(M1), M1 = struct('V',zeros(0,3),'F',zeros(0,3)); end
if isempty(M2), M2 = struct('V',zeros(0,3),'F',zeros(0,3)); end
o = size(M1.V,1);
M = struct('V',[M1.V; M2.V], 'F',[M1.F; M2.F+o]);
end

function addPatch(M, colr, alpha)
if isempty(M), return; end
p = patch('Vertices', M.V, 'Faces', M.F, 'FaceColor', colr, ...
    'EdgeColor', 'none', 'FaceAlpha', alpha);
p.FaceLighting = 'gouraud';
end

function vol_ = meshVolume(M)
a = M.V(M.F(:,1),:);  b = M.V(M.F(:,2),:);  c = M.V(M.F(:,3),:);
vol_ = abs(sum(dot(a, cross(b, c), 2)))/6;
end

function writeBinarySTL(M, fname)
assert(max(M.F(:)) <= size(M.V,1), 'STL 索引越界: max(F)=%d > V=%d', max(M.F(:)), size(M.V,1));
fid = fopen(fname, 'wb');
fwrite(fid, repmat(uint8(0),1,80), 'uint8');
fwrite(fid, uint32(size(M.F,1)), 'uint32');
for t = 1:size(M.F,1)
    p = M.V(M.F(t,:),:);
    n = cross(p(2,:)-p(1,:), p(3,:)-p(1,:));
    nn = norm(n);  if nn == 0, nn = 1; end
    fwrite(fid, single(n(1:3)/nn), 'single');
    fwrite(fid, single(p'), 'single');
    fwrite(fid, uint16(0), 'uint16');
end
fclose(fid);
end
