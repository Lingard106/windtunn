% orbit_of — OpenFOAM 线整体环绕渲染 (对标旧线 orbit_mov: 风洞本体+叶轮+流动, 相机转一圈)
% 用法: orbit_of(vtkFile)                      → 同目录 orbit_of.mp4 + .gif
%       orbit_of(vtkFile, outDir)
% 数据: render_of 读格心点云 (无结构网格) → 0.02m 规则格 bin 重采样 → isosurface
% 语义对齐 explore_gpu/orbit_mov: 白底; 红=叶轮; 深灰点云=skin_casing(风洞本体轮廓);
%       蓝色系三层等值面 lv=[0.90 0.55 0.30]*Veff alpha=[0.35 0.18 0.08];
%       turbo y=0 切片板; 绿线=叶轮轴(+y); 蓝线=风洞轴(x)
% R2026a 提醒: isosurface(X3g,Y3g,Z3g,Wd) 网格版无列序坑; 禁用无网格单数组调用
% 注意: 本文件在 tools/ 正本; 渲染产物出 renders/of/ (case 内会被生成器 rmdir)
function orbit_of(vtkFile, outDir)
if nargin < 2
    outDir = fileparts(vtkFile);
    if isempty(outDir), outDir = pwd; end
end

[P, ~, magU, solid] = render_of(vtkFile);
fprintf('[orbit_of] %d cells | solid pts %d\n', size(P,1), size(solid,1));

xC = -0.5556;  zF = 0.3726;

% ---- Veff: 试验段区 |U| 中位数 (稳态口径, 对齐 tsMean) ----
tsm = P(:,1)>1.56 & P(:,1)<1.74 & abs(P(:,2))<0.145 & abs(P(:,3))<0.095;
Veff = median(magU(tsm));
fprintf('[orbit_of] Veff (test-section median |U|) = %.2f m/s\n', Veff);

% ---- 0.02m 规则格重采样 (bin 均值, 空格=NaN → 等值面只在流体域) ----
% R2026a: histcounts3 已不可用 (未定义), 用 discretize 三维手动分箱
h = 0.02;
xe = -1.14:h:2.54;  ye = -0.41:h:0.55;  ze = -0.29:h:0.95;
ix = discretize(P(:,1), xe);  iy = discretize(P(:,2), ye);  iz = discretize(P(:,3), ze);
ok = ~(isnan(ix) | isnan(iy) | isnan(iz));
nb = [numel(xe)-1, numel(ye)-1, numel(ze)-1];
subs = [ix(ok), iy(ok), iz(ok)];
sumU = accumarray(subs, magU(ok), nb);
cnt  = accumarray(subs, 1, nb);
Wd = sumU ./ max(cnt, 1);
Wd(cnt == 0) = NaN;
[X3g, Y3g, Z3g] = ndgrid(xe(1:end-1)+h/2, ye(1:end-1)+h/2, ze(1:end-1)+h/2);
fprintf('[orbit_of] grid %dx%dx%d, filled %.0f%%\n', nb, 100*nnz(cnt)/numel(cnt));

% ---- y=0 切片板 (turbo, 与旧线 orbit 同款); CData 行=x 列=z ----
jy = round((0 - ye(1))/h);
Usl = squeeze(Wd(:, jy, :));    % [nx × nz]

% ---- 图元 ----
fig = figure('Color', 'w', 'Position', [40 40 1280 720], 'Visible', 'off');
ax = axes('Parent', fig);  hold(ax, 'on');

% 全网格面 + interp 逐顶点色 (R2026a 下 texturemap 四顶点写法 ZData 形状被拒, 全网格无此坑;
% NaN(固体/空格)自然镂空, 露出后方点云)
[Xq, Zq] = ndgrid(xe(1:end-1)+h/2, ze(1:end-1)+h/2);   % [nx × nz]
surface('XData', Xq, 'YData', zeros(size(Xq)), 'ZData', Zq, ...
    'CData', Usl, 'FaceColor', 'interp', 'EdgeColor', 'none', 'Parent', ax);
colormap(ax, turbo);  clim(ax, [0 25]);
hc = colorbar(ax);  ylabel(hc, '|U| (m/s)');

% 风洞本体轮廓: skin_casing patch 点云 (深灰); 叶轮: 红
% solid = render_of 拼接的 [impeller; skin_casing] — 按来源重读拆色
sd = dir(fullfile(fileparts(vtkFile), 'impeller', 'impeller_*.vtk'));
sc = dir(fullfile(fileparts(vtkFile), 'skin_casing', 'skin_casing_*.vtk'));
if ~isempty(sd)
    Pi = read_patch_pts(fullfile(sd(1).folder, sd(1).name));
    scatter3(ax, Pi(:,1), Pi(:,2), Pi(:,3), 2, 'r', 'filled', 'MarkerFaceAlpha', 0.6);
end
if ~isempty(sc)
    Pc = read_patch_pts(fullfile(sc(1).folder, sc(1).name));
    scatter3(ax, Pc(:,1), Pc(:,2), Pc(:,3), 1, [0.25 0.25 0.3], 'filled', 'MarkerFaceAlpha', 0.25);
end

% 蓝色系三层等值面 (速度类, alpha 由高到低)
cm = [linspace(0.25,0.45,256)', linspace(0.55,0.70,256)', linspace(0.95,0.85,256)'];
lv = [0.90 0.55 0.30] * Veff;
alv = [0.35 0.18 0.08];
for q = 1:numel(lv)
    pv = isosurface(X3g, Y3g, Z3g, Wd, lv(q));
    if isempty(pv.vertices), continue; end
    hs = patch(ax, pv, 'FaceColor', cm(128,:), 'EdgeColor', 'none', 'FaceAlpha', alv(q));
    % 层间微差色: 高速层偏白蓝, 低速层偏深蓝 (读数仍由切片板承担)
    hs.FaceColor = interp1([1 256], [cm(1,:) ; cm(end,:)], 256*(lv(q)/(0.90*Veff)));
end

% 双轴锚点: 绿=叶轮轴(+y), 蓝=风洞轴(x)
plot3(ax, [xC xC], [min(ye) max(ye)], [zF zF], 'g-', 'LineWidth', 3);
plot3(ax, [xe(1) xe(end)], [0 0], [0 0], 'b-', 'LineWidth', 3);
axis(ax, 'equal');  axis(ax, 'off');
view(ax, -42, 16);
ttl = sprintf('MRF 1180rpm steady |U| — orbit (red=impeller, blue iso=%.1f/%.1f/%.1f m/s)', lv(1), lv(2), lv(3));
title(ax, ttl);

% ---- 相机环绕 36 帧 → mp4 + gif ----
azs = 0:10:350;
tmpd = fullfile(tempdir, sprintf('orbitof_%d', matlabProcessID));
if exist(tmpd, 'dir'), rmdir(tmpd, 's'); end
mkdir(tmpd);
for k = 1:numel(azs)
    view(ax, azs(k) - 42, 16);
    exportgraphics(fig, fullfile(tmpd, sprintf('f%03d.png', k)), 'Resolution', 100);
end
close(fig);

mp4p = fullfile(outDir, 'orbit_of.mp4');
v = VideoWriter(mp4p, 'MPEG-4');  v.FrameRate = 8;  v.Quality = 92;
open(v);
tgt = [];   % exportgraphics 按内容紧裁剪, 视角变化导致帧尺寸±1px — 统一画布后喂 writeVideo
for k = 1:numel(azs)
    im = imread(fullfile(tmpd, sprintf('f%03d.png', k)));
    if isempty(tgt)
        tgt = [2*floor(size(im,1)/2), 2*floor(size(im,2)/2)];   % 首帧尺寸取偶 (H.264 要求)
    end
    im = fix_canvas(im, tgt(1), tgt(2));
    writeVideo(v, im);
    [A, map] = rgb2ind(im, 128);   % 必须 双输出! 单输出丢 map → 灰度 GIF (2026-10-06 翻过车)
    if k == 1
        imwrite(A, map, fullfile(outDir, 'orbit_of.gif'), 'gif', 'LoopCount', Inf, 'DelayTime', 0.125);
    else
        imwrite(A, map, fullfile(outDir, 'orbit_of.gif'), 'gif', 'WriteMode', 'append', 'DelayTime', 0.125);
    end
end
close(v);
delete(fullfile(tmpd, 'f*.png'));  rmdir(tmpd, 's');
fprintf('[orbit_of] %d frames -> %s + .gif\n', numel(azs), mp4p);
end

function im = fix_canvas(im, H, W)
% 白底 pad/crop 到固定 HxW (帧尺寸不一致会让 writeVideo 中途炸)
out = 255*ones(H, W, 3, 'uint8');
h = min(H, size(im,1));  w = min(W, size(im,2));
out(1:h, 1:w, :) = im(1:h, 1:w, :);
im = out;
end

function pts = read_patch_pts(f)
% legacy patch vtk 顶点 (与 render_of.read_patch_pts 同款实证实现, 10 floats/行)
s = readlines(f);
ip = find(contains(s, 'POINTS '), 1);
np = sscanf(s{ip}, 'POINTS %d');
nPL = ceil(np*3/10);
d = sscanf(strjoin(string(s(ip+1:ip+nPL)), ' '), '%f');
pts = reshape(d, 3, [])';
pts = pts(1:np, :);
end
