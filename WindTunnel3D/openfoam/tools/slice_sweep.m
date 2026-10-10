% slice_sweep — 对一张收敛场沿 x 扫掠出 y=0 切片帧序列并合成 GIF (T5)
% 用法: slice_sweep('VTK/windtunnel-mrf_2000.vtk')  → 同目录 slice_sweep.gif
%       slice_sweep(vtkFile, gifPath)
% 语义对齐 slices_of: 黑底/蓝青白流动/红固体/绿线叶轮轴/蓝线风洞轴; 白竖线=当前扫掠位
% 实现: exportgraphics 逐帧导出(-batch 下可靠) + rgb2ind + imwrite 追加 gif
% 注意: 本文件在 tools/ 正本, 勿复制进 case 目录 (生成器 rmdir 重建会删)
function slice_sweep(vtkFile, gifPath)
if nargin < 2
    outDir = fileparts(vtkFile);
    if isempty(outDir), outDir = pwd; end
    gifPath = fullfile(outDir, 'slice_sweep.gif');
end

[P, ~, magU, solid] = render_of(vtkFile);
fprintf('[slice_sweep] %d cell centres | |U|max %.1f\n', size(P,1), max(magU));

xC = -0.5556;   % 叶轮轴位置 (绿线)
umaxClip = 25;
sl = abs(P(:,2)) < 0.025;
px = P(sl,1);  pz = P(sl,3);  v = min(magU(sl), umaxClip);
inS = solid(:,2) > -0.025 & solid(:,2) < 0.025;

% 蓝青白流动色表 (与 slices_of 同源)
cmap = [linspace(0.05,0.45,86)', linspace(0.10,0.80,86)', linspace(0.45,1.00,86)';
        linspace(0.45,0.95,85)', linspace(0.80,1.00,85)', linspace(1.00,1.00,85)';
        linspace(0.95,1.00,85)', linspace(1.00,1.00,85)', linspace(1.00,0.90,85)'];

fig = figure('Position', [40 40 1600 620], 'Color', 'k');
ax = axes; hold(ax, 'on');
scatter(px, pz, 5, v, 'filled');
if ~isempty(solid)
    plot(solid(inS,1), solid(inS,3), '.', 'Color', [0.9 0.15 0.1], 'MarkerSize', 1.5);
end
plot(ax, [-1.15 2.5], [0 0], 'b-', 'LineWidth', 2);   % 蓝粗线 = 风洞轴
plot(ax, [xC xC], [-0.30 0.95], 'g-', 'LineWidth', 2); % 绿粗线 = 叶轮轴
cur = plot(ax, [0 0], [-0.30 0.95], 'w-', 'LineWidth', 2);
axis(ax, 'equal'); axis(ax, [-1.15 2.5 -0.30 0.95]);
colormap(ax, cmap); cb = colorbar(ax); ylim(cb, [0 umaxClip]); ylabel(cb, '|U| m/s');
xlabel('x (m)'); ylabel('z (m)');

% 逐帧导出再合成 (exportgraphics 在 -batch 下可靠, getframe 不可靠)
xs = -1.0:0.03:2.4;
tmpd = tempfiledir();
n = numel(xs);
for k = 1:n
    set(cur, 'XData', [xs(k) xs(k)]);
    title(sprintf('y=0 slice |U| — sweep x = %.2f m (%d/%d)', xs(k), k, n));
    exportgraphics(fig, fullfile(tmpd, sprintf('f%04d.png', k)), 'Resolution', 100);
end
close(fig);

for k = 1:n
    [A, map] = rgb2ind(imread(fullfile(tmpd, sprintf('f%04d.png', k))), 128);
    if k == 1
        imwrite(A, map, gifPath, 'gif', 'LoopCount', Inf, 'DelayTime', 0.12);
    else
        imwrite(A, map, gifPath, 'gif', 'WriteMode', 'append', 'DelayTime', 0.12);
    end
end
delete(fullfile(tmpd, 'f*.png')); rmdir(tmpd, 's');
fprintf('[slice_sweep] %d frames -> %s\n', n, gifPath);
end

function d = tempfiledir()
d = fullfile(tempdir, sprintf('slicesweep_%d', matlabProcessID));
if exist(d, 'dir'), rmdir(d, 's'); end
mkdir(d);
end
