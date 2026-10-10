% transient_mov — 瞬态 MRF 时序帧 → 动态视频 (风机"工作"演示: 叶轮旋转 + 流动脉动)
% 用法: transient_mov('D:/.../VTK', 1180)                 → 同目录 transient_mov.mp4 + .gif
%       transient_mov(vtkDir, rpm, outDir)
% 输入: vtkDir 内 windtunnel-mrf_*.vtk 时序 (foamToVTK -ascii 导出的多时刻)
% 物理: 叶轮几何在 MRF 网格里静止 — 本工具按真实角速度 ω=2π·rpm/60 逐帧合成叶轮旋转相位
%       (负 omega = 叶片 +φ 扫, 2026-10-04 实证; +φ = 自 +x 向 +z), 流动场用瞬态真解
% 语义: 黑底 / 蓝青白流动 y=0 切片 / 红叶轮 / 白圈 MRF 区 / 绿线叶轮轴 / 蓝线风洞轴
% 坑位: rgb2ind 必须双输出(丢 map=灰度 GIF); exportgraphics 紧裁剪帧尺寸漂移(统一画布);
%       writeVideo 首帧锁尺寸(H.264 偶数); -batch 下禁 getframe
function transient_mov(vtkDir, rpm, outDir)
if nargin < 2, rpm = 1180; end
if nargin < 3, outDir = vtkDir; end
d = dir(fullfile(vtkDir, 'windtunnel-mrf_*.vtk'));
d = d(~[d.isdir]);
assert(~isempty(d), 'no windtunnel-mrf_*.vtk under %s', vtkDir);
% 按解析时刻排序 (datenum=文件 mtime, 会把基帧挤到末尾打乱时序)
tt = zeros(numel(d),1);
for k = 1:numel(d)
    tok = regexp(d(k).name, '_(\d+\.?\d*)\.vtk', 'tokens', 'once');
    tt(k) = str2double(tok{1});
end
[~, order] = sort(tt);  d = d(order);
nF = numel(d);
fprintf('[transient_mov] %d frames from %s\n', nF, vtkDir);

xC = -0.5556;  zF = 0.3726;
umaxClip = 25;
w = 2*pi*abs(rpm)/60;            % rad/s (大小)
cmap = [linspace(0.05,0.45,86)', linspace(0.10,0.80,86)', linspace(0.45,1.00,86)';
        linspace(0.45,0.95,85)', linspace(0.80,1.00,85)', linspace(1.00,1.00,85)';
        linspace(0.95,1.00,85)', linspace(1.00,1.00,85)', linspace(1.00,0.90,85)'];

fig = figure('Position', [20 20 1500 560], 'Color', 'k', 'Visible', 'off');
ax = axes('Parent', fig);  hold(ax, 'on');
hS = scatter(ax, nan, nan, 6, 'filled');            % 流动切片 (每帧换数据)
hI = scatter(ax, nan, nan, 3, 'r', 'filled');       % 叶轮旋转点云
plot(ax, [xC xC], [-0.30 0.95], 'g-', 'LineWidth', 2);
plot(ax, [-1.15 2.5], [0 0], 'b-', 'LineWidth', 2);
th = linspace(0, 2*pi, 180);
plot(ax, xC + 0.232*cos(th), zF + 0.232*sin(th), 'w-', 'LineWidth', 1);
axis(ax, 'equal');  axis(ax, [-1.15 2.5 -0.30 0.95]);
colormap(ax, cmap);  cb = colorbar(ax);  ylim(cb, [0 umaxClip]);  ylabel(cb, '|U| m/s');
xlabel(ax, 'x (m)');  ylabel(ax, 'z (m)');

tmpd = fullfile(tempdir, sprintf('transmov_%d', matlabProcessID));
if exist(tmpd, 'dir'), rmdir(tmpd, 's'); end
mkdir(tmpd);

t0 = [];
for k = 1:nF
    f = fullfile(d(k).folder, d(k).name);
    [P, ~, magU, ~] = render_of(f);
    % 时刻: 从文件名 windtunnel-mrf_<t>.vtk 提取
    tok = regexp(d(k).name, '_(\d+\.?\d*)\.vtk', 'tokens', 'once');
    t = str2double(tok{1});
    if isempty(t0), t0 = t; end
    tr = max(t - t0, 0);           % 相对瞬态起点的物理时间 (s)
    dphi = mod(w*tr, 2*pi);        % 本帧叶轮相位 (rad, +φ 扫)

    sl = abs(P(:,2)) < 0.025;
    set(hS, 'XData', P(sl,1), 'YData', P(sl,3), 'CData', min(magU(sl), umaxClip));

    % 叶轮点云旋转 (+φ: x'=xC+dx cos−dz sin, z'=zF+dx sin+dz cos)
    ipt = solid_pts_impeller(f);
    if ~isempty(ipt)
        dx = ipt(:,1) - xC;  dz = ipt(:,3) - zF;
        xr = xC + dx*cos(dphi) - dz*sin(dphi);
        zr = zF + dx*sin(dphi) + dz*cos(dphi);
        keep = abs(xr) < 2.5 & zr > -0.3 & zr < 0.95;
        set(hI, 'XData', xr(keep), 'YData', zr(keep));
    end
    nBPF = tr*abs(rpm)/60*16;      % 叶片通过周期数 (16 叶)
    title(ax, sprintf('transient MRF %drpm | t = %.1f ms (%.1f blade passes) | \\phi = %.0f\\circ', ...
        rpm, tr*1e3, nBPF, dphi*180/pi));
    drawnow;
    exportgraphics(fig, fullfile(tmpd, sprintf('f%03d.png', k)), 'Resolution', 100);
    fprintf('[transient_mov] frame %d/%d  t=%.4f  phi=%.0f deg\n', k, nF, tr, dphi*180/pi);
end
close(fig);

% ---- 合成 mp4 + gif (统一画布 + 双输出 rgb2ind) ----
mp4p = fullfile(outDir, 'transient_mov.mp4');
v = VideoWriter(mp4p, 'MPEG-4');  v.FrameRate = 8;  v.Quality = 92;
open(v);
tgt = [];
for k = 1:nF
    im = imread(fullfile(tmpd, sprintf('f%03d.png', k)));
    if isempty(tgt), tgt = [2*floor(size(im,1)/2), 2*floor(size(im,2)/2)]; end
    out = 255*ones(tgt(1), tgt(2), 3, 'uint8');
    h = min(tgt(1), size(im,1));  wd = min(tgt(2), size(im,2));
    out(1:h, 1:wd, :) = im(1:h, 1:wd, :);
    writeVideo(v, out);
    [A, map] = rgb2ind(out, 128);
    if k == 1
        imwrite(A, map, fullfile(outDir, 'transient_mov.gif'), 'gif', 'LoopCount', Inf, 'DelayTime', 0.125);
    else
        imwrite(A, map, fullfile(outDir, 'transient_mov.gif'), 'gif', 'WriteMode', 'append', 'DelayTime', 0.125);
    end
end
close(v);
delete(fullfile(tmpd, 'f*.png'));  rmdir(tmpd, 's');
fprintf('[transient_mov] %d frames -> %s + .gif\n', nF, mp4p);
end

function pts = solid_pts_impeller(f)
% 同目录 impeller/impeller_<同时刻>.vtk 顶点 (与 render_of 同款解析)
[~, base] = fileparts(f);
tok = regexp(base, '_(\d+\.?\d*)$', 'tokens', 'once');
if isempty(tok), pts = []; return; end
cand = fullfile(fileparts(f), 'impeller', ['impeller_' tok{1} '.vtk']);
pts = [];
if exist(cand, 'file')
    s = readlines(cand);
    ip = find(contains(s, 'POINTS '), 1);
    np = sscanf(s{ip}, 'POINTS %d');
    nPL = ceil(np*3/10);
    dd = sscanf(strjoin(string(s(ip+1:ip+nPL)), ' '), '%f');
    pts = reshape(dd, 3, [])';
    pts = pts(1:min(np, size(pts,1)), :);
end
end
