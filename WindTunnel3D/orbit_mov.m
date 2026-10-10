function orbit_mov()
% orbit_mov — 风机-风洞全链路 3D 整体环绕渲染 (相机绕场景轴转一圈)
% 内容: 涵道壁轮廓 + 蜗壳螺旋/外廓 + 叶轮体素(红) + y=0 速度切片板 + 等值面
% 输出: orbit_fan_tunnel.mp4 + .gif
% 用法: orbit_mov  (需 gpu_meta.mat + windtunnel.raw + 至少一个 u_*.vtk)
RT = fileparts(mfilename('fullpath'));  addpath(RT);
M = load(fullfile(RT,'gpu_meta.mat'));  meta = M.meta;
nx = meta.nx; ny = meta.ny; nz = meta.nz; dx = meta.dx; Sv = meta.velScale;
if isfield(meta,'y0'), y0 = meta.y0; else, y0 = -(ny*dx)/2; end
if isfield(meta,'z0'), z0 = meta.z0; else, z0 = -(nz*dx)/2; end
fid = fopen(fullfile(RT,'fluidx3d','windtunnel.raw'),'rb'); raw = fread(fid,inf,'uint8=>uint8'); fclose(fid);
raw = reshape(raw, nx, ny, nz);
d = dir(fullfile(RT,'fluidx3d','vtk','u_*.vtk'));
assert(~isempty(d), '无 VTK 快照 — 先跑 windtunnel_fan.exe');
[~,imx] = max([d.datenum]);
A = read_fluidx3d_vtk(fullfile(d(imx).folder, d(imx).name))*Sv;
fprintf('orbit_mov: 场数据 %s\n', d(imx).name);

xv = meta.x0 + ((1:nx)-0.5)'*dx;
yv = y0 + ((1:ny)-0.5)*dx;
zv = z0 + ((1:nz)-0.5)*dx;
xs3 = xv;  ys3 = reshape(yv,1,ny);  zs3 = reshape(zv,1,1,nz);

fig = figure('Color','w','Position',[40 40 1280 720],'Visible','off');
ax = axes('Parent',fig); hold(ax,'on');

% y=0 速度切片板 (贴在 3D 场中间)
jy = min(ny, max(1, round((0-y0)/dx+0.5)));
Usl = squeeze(sqrt(sum(A(:,jy,:,:).^2,4)));  Usl(squeeze(raw(:,jy,:))>=1) = NaN;
hI = surface('XData', [xv(1) xv(end) xv(end) xv(1)], ...
             'YData', [0 0 0 0], ...
             'ZData', [zs3(1) zs3(1) zs3(end) zs3(end)], ...
             'CData', Usl', 'FaceColor','texturemap', 'EdgeColor','none');
colormap(ax, turbo); caxis(ax, [0 28]);
hc = colorbar(ax); ylabel(hc, '|U| (m/s)');

% 涵道壁轮廓 (Gext 包络素线, 与 explore_gpu 同源)
G = meta.Gext;
for sg = [1 -1]
    plot3(ax, G.x,  sg*G.a(:),  sg*G.a(:)*0, '-', 'Color',[0.15 0.15 0.2], 'LineWidth', 0.8);
    plot3(ax, G.x,  sg*G.a(:),  sg*G.b(:), '-', 'Color',[0.15 0.15 0.2], 'LineWidth', 1.2);
    plot3(ax, G.x,  sg*G.a(:)*0, sg*G.b(:), '-', 'Color',[0.15 0.15 0.2], 'LineWidth', 0.8);
end

% 蜗壳解析线 (螺旋 + 外廓圆, xoz 平面, y=0 前后双份)
if isfield(meta,'fanLayout')
    fl = meta.fanLayout;  vf = meta.vol;
    thV = linspace(0, 359, 120);
    rS = vf.rTongue + vf.a*mod(thV-300,360)*pi/180;
    ph = sign(meta.impSpin)*thV;
    for ye = [-0.125 0.125]
        plot3(ax, fl.xC + rS.*cosd(ph), ye*ones(size(thV)), fl.zF + rS.*sind(ph), '-', ...
            'Color',[0.2 0.2 0.3], 'LineWidth', 1.5);
    end
end

% 叶轮体素 (红)
rot = double(raw==2);
[Xg,Yg,Zg] = ndgrid(xv, yv, zv);
pv = isosurface(Xg, Yg, Zg, rot, 0.5);
if ~isempty(pv.vertices)
    patch(ax, pv, 'FaceColor',[0.85 0.25 0.1], 'FaceAlpha',0.95, 'EdgeColor','none');
end

% 试验段射流等值面 (半透明, 增强流动观感)
U3 = sqrt(sum(A.^2,4));  U3(raw>=1) = NaN;
Wd = U3(1:4:end,1:4:end,1:4:end);
[X3g,Y3g,Z3g] = ndgrid(xv(1:4:end), yv(1:4:end), zv(1:4:end));
pv2 = isosurface(X3g, Y3g, Z3g, Wd, 20);
if ~isempty(pv2.vertices)
    patch(ax, pv2, 'FaceColor',[0.1 0.4 0.9], 'FaceAlpha',0.25, 'EdgeColor','none');
end

axis(ax,'equal'); grid(ax,'on');
xlim(ax, [meta.x0-0.05, meta.x0+nx*dx+0.05]);
zlim(ax, [z0-0.15, z0+(nz*dx)+0.35]);
camlight(ax,'headlight'); material(ax,'dull'); lighting(ax,'gouraud');
view(ax, -42, 16);
title(ax, sprintf('风机-风洞全链路整体 | %s | 红=叶轮 板=|U| y=0 蓝=20m/s射流', strrep(d(imx).name,'_','\_')), 'FontSize', 10);

out = fullfile(RT,'orbit_fan_tunnel.mp4');
v = VideoWriter(out, 'MPEG-4');  v.FrameRate = 8;  v.Quality = 92;
open(v);  cleaner = onCleanup(@() close(v));
frames = {};
azs = 0:10:350;
for q = 1:numel(azs)
    view(ax, azs(q), 16);
    drawnow;
    fr = frame2im(getframe(fig));
    writeVideo(v, fr);
    frames{q} = fr;
    fprintf('orbit %d/%d (az=%d°)\n', q, numel(azs), azs(q));
end
close(v);
try
    im = cat(4, frames{:});   % → gif (索引色逐帧)
    [Imap] = rgb2ind(frames{1}, 128);
    gifOut = fullfile(RT,'orbit_fan_tunnel.gif');
    for q = 1:numel(frames)
        [A2, Imap] = rgb2ind(frames{q}, 128);
        if q==1, imwrite(A2, Imap, gifOut, 'gif', 'Loopcount', inf, 'DelayTime', 0.125);
        else,    imwrite(A2, Imap, gifOut, 'gif', 'WriteMode','append', 'DelayTime', 0.125);
        end
    end
    fprintf('orbit_fan_tunnel.gif 已写\n');
catch e
    fprintf('[orbit] gif 失败: %s\n', e.message);
end
fprintf('--- orbit_mov 完成: %s (%d 帧) ---\n', out, numel(azs));
end
