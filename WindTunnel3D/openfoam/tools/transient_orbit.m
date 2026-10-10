% transient_orbit — 风机+风洞整体 3D 视场的动态过程 (对标旧线 orbit_fan_tunnel 的新线版)
% 用法: transient_orbit(vtkDir, rpm)                       → 同目录 transient_orbit.mp4 + .gif
%       transient_orbit(vtkDir, rpm, outDir)
% 结构: 片头环绕段 36 帧 (流场固定=第一帧, 相机转一圈, 便宜) + 演化段 nF 帧
%       (固定机位缓漂 0.8°/帧, 流场逐帧演化, 叶轮按真实 ω 合成旋转相位)
% 数据: windtunnel-mrf_*.vtk 时序 (foamToVTK -ascii -useTimeName, 须含 C 格心场)
% 粒度: 0.012m bin (orbit_of 曾用 0.02 — 与旧线 4mm 体素观感差异, 加密)
% 坑位: discretize 分箱 / accumarray 下标矩阵 / surface 'Parent' 形式 / 画布统一 / rgb2ind 双输出
function transient_orbit(vtkDir, rpm, outDir)
if nargin < 2, rpm = 1180; end
if nargin < 3, outDir = vtkDir; end
d = dir(fullfile(vtkDir, 'windtunnel-mrf_*.vtk'));
d = d(~[d.isdir]);
assert(~isempty(d), 'no windtunnel-mrf_*.vtk under %s', vtkDir);
tt = zeros(numel(d),1);
for k = 1:numel(d)
    tok = regexp(d(k).name, '_(\d+\.?\d*)\.vtk', 'tokens', 'once');
    tt(k) = str2double(tok{1});
end
[~, order] = sort(tt);  d = d(order);  tt = tt(order);
nF = numel(d);
fprintf('[transient_orbit] %d frames\n', nF);

xC = -0.5556;  zF = 0.3726;
umaxClip = 25;
w = 2*pi*abs(rpm)/60;
cm = [linspace(0.25,0.45,256)', linspace(0.55,0.70,256)', linspace(0.95,0.85,256)'];

% ---- 0.012m bin 网格 ----
h = 0.012;
xe = -1.14:h:2.54;  ye = -0.41:h:0.55;  ze = -0.29:h:0.95;
nb = [numel(xe)-1, numel(ye)-1, numel(ze)-1];
[X3g, Y3g, Z3g] = ndgrid(xe(1:end-1)+h/2, ye(1:end-1)+h/2, ze(1:end-1)+h/2);
[Xq, Zq] = ndgrid(xe(1:end-1)+h/2, ze(1:end-1)+h/2);

fig = figure('Color', 'w', 'Position', [40 40 1280 720], 'Visible', 'off');
ax = axes('Parent', fig);  hold(ax, 'on');

hIso = gobjects(0);  hImp = [];  hSurf = [];  hShell = [];
Veff = [];  ipt0 = [];  ipt0c = [];
inited = false;

tmpd = fullfile(tempdir, sprintf('trorb_%d', matlabProcessID));
if exist(tmpd, 'dir'), rmdir(tmpd, 's'); end
mkdir(tmpd);

fi = 0;
for k = 1:nF
    f = fullfile(d(k).folder, d(k).name);
    [P, ~, magU, ~] = render_of(f);
    if isempty(Veff)
        tsm = P(:,1)>1.56 & P(:,1)<1.74 & abs(P(:,2))<0.145 & abs(P(:,3))<0.095;
        Veff = median(magU(tsm));
        fprintf('[transient_orbit] Veff=%.2f, grid %dx%dx%d\n', Veff, nb(1), nb(2), nb(3));
    end
    ix = discretize(P(:,1), xe);  iy = discretize(P(:,2), ye);  iz = discretize(P(:,3), ze);
    ok = ~(isnan(ix) | isnan(iy) | isnan(iz));
    subs = [ix(ok), iy(ok), iz(ok)];
    sumU = accumarray(subs, magU(ok), nb);
    cnt  = accumarray(subs, 1, nb);
    Wd = sumU ./ max(cnt, 1);
    Wd(cnt == 0) = NaN;
    jy = round((0 - ye(1))/h);
    Usl = squeeze(Wd(:, jy, :));
    tok = regexp(d(k).name, '_(\d+\.?\d*)\.vtk', 'tokens', 'once');
    ipt0 = impeller_pts(fullfile(fileparts(f), 'impeller', ['impeller_' tok{1} '.vtk']));

    % ---- 静态图元 (一次) ----
    if ~inited
        hSurf = surface('XData', Xq, 'YData', zeros(size(Xq)), 'ZData', Zq, ...
            'CData', Usl, 'FaceColor', 'interp', 'EdgeColor', 'none', 'Parent', ax);
        colormap(ax, turbo);  clim(ax, [0 umaxClip]);
        % 手绘固定色标: colorbar 对象在离屏 figure + view 旋转下渲染不稳定
        % (实测同对象部分帧整条消失), 固定像素位渐变条+刻度绕开全部布局玄学
        axc = axes('Parent', fig, 'Position', [0.90 0.28 0.022 0.44]);
        imagesc(axc, [0 umaxClip], [0 1], repmat(linspace(0, umaxClip, 256)', 1, 8));
        colormap(axc, turbo);  axis(axc, 'off');
        for vlab = 0:5:umaxClip
            text(axc, 1.3, 1 - vlab/umaxClip, sprintf('%d', vlab), ...
                'Units', 'data', 'FontSize', 8, 'Color', 'k', 'Clipping', 'off', 'Parent', axc);
        end
        text(axc, 1.15, 1.09, '|U| m/s', 'Units', 'data', 'FontSize', 9, 'Color', 'k', ...
            'Clipping', 'off', 'Parent', axc);
        sd = dir(fullfile(fileparts(f), 'skin_casing', 'skin_casing_*.vtk'));
        if ~isempty(sd)
            Pc = patch_pts(fullfile(sd(1).folder, sd(1).name));
            ipt0c = Pc;   % 暂存, 每帧在等值面之后重画 (顺序保证不被半透明壳吞掉)
        end
        th = linspace(0, 2*pi, 180);
        plot3(ax, xC + 0.234*cos(th), zeros(size(th)), zF + 0.234*sin(th), 'w-', 'LineWidth', 0.8);
        plot3(ax, [xC xC], [min(ye) max(ye)], [zF zF], 'g-', 'LineWidth', 3);
        plot3(ax, [xe(1) xe(end)], [0 0], [0 0], 'b-', 'LineWidth', 3);
        axis(ax, 'equal');  axis(ax, 'off');
        inited = true;
    else
        set(hSurf, 'CData', Usl);
    end

    % ---- 等值面更新 (delete 重建) ----
    delete(hIso(isgraphics(hIso)));  hIso = gobjects(0);
    lv = [0.90 0.55 0.30] * Veff;  alv = [0.35 0.18 0.08];
    for q = 1:3
        pv = isosurface(X3g, Y3g, Z3g, Wd, lv(q));
        if isempty(pv.vertices), continue; end
        fc = interp1([1 256], [cm(1,:); cm(end,:)], 256*(lv(q)/(0.90*Veff)));
        hIso(end+1) = patch(ax, pv, 'FaceColor', fc, 'EdgeColor', 'none', 'FaceAlpha', alv(q)); %#ok<AGROW>
    end
    % 壳体/叶轮点云必须画在等值面之后 (半透明 patch 会洗淡先画的点 — mini 实验实锤)
    tr = max(tt(k) - tt(1), 0);
    dphi = mod(w*tr, 2*pi);
    dx = ipt0(:,1) - xC;  dz = ipt0(:,3) - zF;
    xr = xC + dx*cos(dphi) - dz*sin(dphi);
    zr = zF + dx*sin(dphi) + dz*cos(dphi);
    if isgraphics(hShell), delete(hShell); end
    if isgraphics(hImp), delete(hImp); end
    if ~isempty(ipt0c)
        hShell = scatter3(ax, ipt0c(:,1), ipt0c(:,2), ipt0c(:,3), 4, [0.15 0.15 0.2], 'filled');
    end
    hImp = scatter3(ax, xr, zeros(size(xr)), zr, 3, 'r', 'filled', 'MarkerFaceAlpha', 0.7);

    % pass A: 环绕段 — 仅第一帧的场重复用于 36 帧相机 (只在 k==1 时)
    if k == 1
        for ka = 1:36
            view(ax, -42 + (ka-1)*10, 16);
            title(ax, sprintf('MRF %drpm — 3D orbit (blue iso %.0f/%.0f/%.0f m/s)', ...
                rpm, lv(1), lv(2), lv(3)), 'FontWeight', 'bold', 'Color', 'k');
            fi = fi + 1;
            print(fig, fullfile(tmpd, sprintf('f%03d', fi)), '-dpng', '-r100');
        end
        fprintf('[transient_orbit] orbit segment done (36 frames)\n');
    end
    % pass B: 演化段 — 固定机位缓漂
    view(ax, -42 + 350 + 0.8*(k-1), 16);
    title(ax, sprintf('MRF %drpm — transient | t=%.1f ms (%.1f blade passes)', ...
        rpm, tr*1e3, tr*abs(rpm)/60*16), 'FontWeight', 'bold', 'Color', 'k');
    fi = fi + 1;
    print(fig, fullfile(tmpd, sprintf('f%03d', fi)), '-dpng', '-r100');
    fprintf('[transient_orbit] evolve frame %d/%d\n', k, nF);
end
close(fig);

% ---- 合成 ----
nf = fi;
mp4p = fullfile(outDir, 'transient_orbit.mp4');
v = VideoWriter(mp4p, 'MPEG-4');  v.FrameRate = 8;  v.Quality = 92;
open(v);  tgt = [];
for k = 1:nf
    im = imread(fullfile(tmpd, sprintf('f%03d.png', k)));
    if isempty(tgt), tgt = [2*floor(size(im,1)/2), 2*floor(size(im,2)/2)]; end
    out = 255*ones(tgt(1), tgt(2), 3, 'uint8');
    hh = min(tgt(1), size(im,1));  wd = min(tgt(2), size(im,2));
    out(1:hh, 1:wd, :) = im(1:hh, 1:wd, :);
    writeVideo(v, out);
    [A, map] = rgb2ind(out, 256);
    if k == 1
        imwrite(A, map, fullfile(outDir, 'transient_orbit.gif'), 'gif', 'LoopCount', Inf, 'DelayTime', 0.125);
    else
        imwrite(A, map, fullfile(outDir, 'transient_orbit.gif'), 'gif', 'WriteMode', 'append', 'DelayTime', 0.125);
    end
end
close(v);
delete(fullfile(tmpd, 'f*.png'));  rmdir(tmpd, 's');
fprintf('[transient_orbit] %d frames -> %s + .gif\n', nf, mp4p);
end

function pts = impeller_pts(f)
pts = [];
if exist(f, 'file')
    s = readlines(f);
    ip = find(contains(s, 'POINTS '), 1);
    np = sscanf(s{ip}, 'POINTS %d');
    nPL = ceil(np*3/10);
    dd = sscanf(strjoin(string(s(ip+1:ip+nPL)), ' '), '%f');
    pts = reshape(dd, 3, [])';
    pts = pts(1:min(np, size(pts,1)), :);
end
end

function pts = patch_pts(f)
s = readlines(f);
ip = find(contains(s, 'POINTS '), 1);
np = sscanf(s{ip}, 'POINTS %d');
nPL = ceil(np*3/10);
dd = sscanf(strjoin(string(s(ip+1:ip+nPL)), ' '), '%f');
pts = reshape(dd, 3, [])';
pts = pts(1:np, :);
end
