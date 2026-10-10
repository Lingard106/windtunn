% slices_of — OpenFOAM 场切片可视化 (scatter 格心直绘, 零插值零歧义)
% 语义: 黑底/蓝青白流动/红固体/绿线叶轮轴/蓝线风洞轴 (对齐 explore_gpu)
% 用法: slices_of('VTK/windtunnel-mrf_2750.vtk')  → slice_y0 / slice_rotor / slice_testsection PNG
% 注意: 本文件位于 case 目录, 生成器重建 case 会删除它 — 正本在 WindTunnel3D/openfoam/tools/
function slices_of(vtkFile, outDir)
if nargin < 2, outDir = fileparts(vtkFile); end
if isempty(outDir), outDir = pwd; end

[P, U, magU, solid] = render_of(vtkFile);
fprintf('[slices_of] %d cell centres | solid pts %d | |U|max %.1f\n', size(P,1), size(solid,1), max(magU));

xC = -0.5556;  zF = 0.3726;
umaxClip = 25;

% 蓝青白流动色表 (低=深蓝, 高=白)
cmap = [linspace(0.05,0.45,86)', linspace(0.10,0.80,86)', linspace(0.45,1.00,86)';
        linspace(0.45,0.95,85)', linspace(0.80,1.00,85)', linspace(1.00,1.00,85)';
        linspace(0.95,1.00,85)', linspace(1.00,1.00,85)', linspace(1.00,0.90,85)'];

%% ---- 主图: y≈0 纵切片 ----
sl = abs(P(:,2)) < 0.025;
fig = figure('Position', [20 20 1500 560], 'Color', 'k');
ax = axes; hold(ax, 'on');
v = min(magU(sl), umaxClip);
scatter(P(sl,1), P(sl,3), 6, v, 'filled');
axis(ax, 'equal'); axis(ax, [-1.15 2.5 -0.30 0.95]);
colormap(ax, cmap); cb = colorbar(ax); ylim(cb, [0 umaxClip]); ylabel(cb, '|U| m/s');
draw_solid(solid, [-0.025 0.025]);
draw_axes_semantics(xC, zF);
title('y\approx0 slice |U| — black=solid, blue-white=flow, red dots=walls');
xlabel('x (m)'); ylabel('z (m)');
exportgraphics(fig, fullfile(outDir, 'slice_y0.png'), 'Resolution', 140);

%% ---- 叶轮中面放大 ----
fig = figure('Position', [20 20 780 680], 'Color', 'k');
ax = axes; hold(ax, 'on');
scatter(P(sl,1), P(sl,3), 6, min(magU(sl), 30), 'filled');
axis(ax, 'equal'); axis(ax, [-1.12 0.05 -0.14 0.92]);
colormap(ax, cmap); cb = colorbar(ax); ylim(cb, [0 30]);
draw_solid(solid, [-0.025 0.025]);
th = linspace(0, 2*pi, 360);
plot(xC + 0.23*cos(th), zF + 0.23*sin(th), 'w-', 'LineWidth', 1);
plot(xC, zF, 'wo', 'MarkerFaceColor', 'r', 'MarkerSize', 5);
title('rotor mid-plane |U|');
xlabel('x (m)'); ylabel('z (m)');
exportgraphics(fig, fullfile(outDir, 'slice_rotor.png'), 'Resolution', 140);

%% ---- 试验段横截面 x=1.65 (griddata 去重 + 掩膜同向) ----
sl2 = abs(P(:,1) - 1.65) < 0.02;
yg = -0.39:0.0025:0.53;  zg2 = -0.30:0.0025:0.95;
h2 = 0.0025;
ye2 = [yg(1)-h2/2, yg(1:end-1)+h2/2, yg(end)+h2/2];
ze2 = [zg2(1)-h2/2, zg2(1:end-1)+h2/2, zg2(end)+h2/2];
cnt2 = histcounts2(P(sl2,2), P(sl2,3), ye2, ze2);   % ny×nz (行=y); V2 是 nz×ny (meshgrid) — 必须转置后再掩膜
[Yg2, Zg2] = meshgrid(yg, zg2);
pts3 = [P(sl2,2), P(sl2,3)];  v3 = min(magU(sl2), umaxClip);
[~, iu3] = unique(pts3, 'rows', 'stable');
V2 = griddata(pts3(iu3,1), pts3(iu3,2), v3(iu3), Yg2, Zg2, 'natural');
V2(cnt2'==0) = NaN;   % 2026-10-05 T4 修复: 原 V2(cnt2==0) 因 numel 相同静默错向 (369×501 vs 501×369)
fig = figure('Position', [20 20 640 620], 'Color', 'k');
ax = axes; hold(ax, 'on');
imagesc('XData', yg, 'YData', zg2, 'CData', V2);
set(ax, 'YDir', 'normal'); axis(ax, 'equal'); axis(ax, [-0.39 0.53 -0.30 0.95]);
inS = solid(:,1) > 1.63 & solid(:,1) < 1.67;
plot(solid(inS,2), solid(inS,3), '.', 'Color', [0.85 0.1 0.1], 'MarkerSize', 1.2);
rectangle('Position', [-0.15 -0.10 0.30 0.20], 'EdgeColor', 'g', 'LineWidth', 1.5);
colormap(ax, cmap); cb = colorbar(ax); ylim(cb, [0 umaxClip]); ylabel(cb, '|U| m/s');
title('test section x=1.65 cross-slice (green frame = 300\times200)');
xlabel('y (m)'); ylabel('z (m)');
exportgraphics(fig, fullfile(outDir, 'slice_testsection.png'), 'Resolution', 140);

fprintf('[slices_of] done -> %s\n', outDir);
end

function draw_solid(solid, slYlim)
if isempty(solid), return; end
in = solid(:,2) > slYlim(1) & solid(:,2) < slYlim(2);
plot(solid(in,1), solid(in,3), '.', 'Color', [0.9 0.15 0.1], 'MarkerSize', 1.5);
end

function draw_axes_semantics(xC, zF)
plot(xlim, [0 0], 'b-', 'LineWidth', 2);              % 蓝粗线 = 风洞轴 x
plot([xC xC], ylim, 'g-', 'LineWidth', 2);            % 绿粗线 = 叶轮轴 y
plot(xC, zF, 'wo', 'MarkerFaceColor', 'r', 'MarkerSize', 5);
end
