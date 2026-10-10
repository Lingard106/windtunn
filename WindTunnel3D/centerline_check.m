%% centerline_check — 风洞中心线 (y=0, z=0) 流速沿程定量 (时均 vs 瞬时)
% 固体区一律置 NaN (瞬时快照固体格保留动壁给定值, 无物理意义, 必须掩掉)
% 用法: centerline_check  (需 gpu_meta.mat + mean_u.vtk + 任一 u_*.vtk)
RT = fileparts(mfilename('fullpath'));  addpath(RT);
M = load(fullfile(RT,'gpu_meta.mat'));  m = M.meta;
fid = fopen(fullfile(RT,'fluidx3d','windtunnel.raw'),'rb'); raw = fread(fid,inf,'uint8=>uint8'); fclose(fid);
raw = reshape(raw, m.nx, m.ny, m.nz);
mask = raw >= 1;
xv = m.x0 + ((1:m.nx)-0.5)*m.dx;
jy0 = min(m.ny, max(1, round((0-m.y0)/m.dx+0.5)));
kz0 = min(m.nz, max(1, round((0-m.z0)/m.dx+0.5)));

Um = read_fluidx3d_vtk(fullfile(RT,'fluidx3d','vtk','mean_u.vtk'))*m.velScale;
uM = squeeze(sqrt(sum(Um(:,jy0,kz0,:).^2,3)));  uM(mask(:,jy0,kz0)) = NaN;  clear Um
dU = dir(fullfile(RT,'fluidx3d','vtk','u_*.vtk'));
[~,imx] = max([dU.datenum]);
Ui = read_fluidx3d_vtk(fullfile(dU(imx).folder, dU(imx).name))*m.velScale;
uI = squeeze(sqrt(sum(Ui(:,jy0,kz0,:).^2,3)));  uI(mask(:,jy0,kz0)) = NaN;  clear Ui
fprintf('瞬时快照: %s\n', dU(imx).name);

fig = figure('Color','w','Position',[40 40 1360 500],'Visible','off');
subplot(1,2,1); hold on; grid on;
h1 = plot(xv*1e3, uM, 'b-', 'LineWidth', 1.8);
h2 = plot(xv*1e3, uI, 'r-', 'LineWidth', 0.8);
xline(500, 'k--', '整流网 x=0.50m');
xlabel('x (mm)'); ylabel('|U| (m/s)');
legend([h1 h2], {'时均 mean_u', sprintf('瞬时 %s', dU(imx).name)}, 'Location','northwest', 'Interpreter','none');
title('中心线 (y=0, z=0) 流速沿程 — 固体区已掩码');
subplot(1,2,2); hold on; grid on;
plot(xv*1e3, uM, 'b-', 'LineWidth', 1.4);
for xq = [0.30 0.44 0.48 0.52 0.60 0.80]
    [~,iq] = min(abs(xv-xq));
    plot(xq*1e3, uM(iq), 'bo', 'MarkerFaceColor','b', 'MarkerSize', 5);
    text(xq*1e3, uM(iq)+1.2, sprintf('%.1f', uM(iq)), 'HorizontalAlignment','center', 'FontSize', 8);
    fprintf('x=%.2fm: 时均 %.1f | 瞬时 %.1f m/s\n', xq, uM(iq), uI(iq));
end
xline(500, 'k--', '整流网');
axis([0 1100 0 32]);   % mm 单位 (此前误写 [0 1.1] 裁掉了全部数据)
xlabel('x (mm)'); ylabel('|U| (m/s)'); title('渐扩段—整流网—稳定段 关键站点');
exportgraphics(fig, fullfile(RT,'centerline_check.png'), 'Resolution', 130);
fprintf('centerline_check.png 已更新\n');
