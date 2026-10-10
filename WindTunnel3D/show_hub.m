% show_hub — 纵剖 y=0 展示: 侧置风机布局 (毂封轴/蜗壳/底部切向出口 + 全隧道)
% 套件相对定位; 读最新 u_*.vtk 快照
RT = fileparts(mfilename('fullpath'));
addpath(RT);
M = load(fullfile(RT,'gpu_meta.mat'));  meta = M.meta;
dx = meta.dx;  nx = meta.nx;  ny = meta.ny;  nz = meta.nz;
dU = dir(fullfile(RT,'fluidx3d','vtk','u_*.vtk'));
assert(~isempty(dU), '无 VTK 快照 — 先跑 windtunnel_fan.exe');
[~, imx] = max([dU.datenum]);
A = read_fluidx3d_vtk(fullfile(dU(imx).folder, dU(imx).name));
fid = fopen(fullfile(RT,'fluidx3d','windtunnel.raw'),'rb');
raw = fread(fid, inf, 'uint8');  fclose(fid);
raw = reshape(raw, nx, ny, nz);
if isfield(meta,'y0'), y0 = meta.y0; else, y0 = -(ny*dx)/2; end
if isfield(meta,'z0'), z0 = meta.z0; else, z0 = -(nz*dx)/2; end
U = sqrt(sum(A.^2,4))*meta.velScale;
U(raw >= 1) = NaN;
xs = meta.x0 + ((1:nx)-0.5)'*dx;
zv = z0 + ((1:nz)-0.5)*dx;
jy = min(ny,max(1,round((0-y0)/dx+0.5)));   % 过 y=0 (蜗壳腔/叶轮中面)
figure('Color','w','Position',[40 40 1500 520]);
imagesc(xs*1000, zv*1000, squeeze(U(:,jy,:))'); axis xy;
set(gca,'YDir','normal'); colormap(parula); caxis([0 25]); colorbar;
hold on;
contour(xs*1000, zv*1000, double(reshape(raw(:,jy,:)~=0,nx,nz))', [0.5 0.5], 'k-', 'LineWidth', 1.2);
% 标注 (侧置布局: 蜗壳心 impCenterX/Z, 进气 +y, 切向出口 +x)
xC = meta.impCenterX*1000;  zC = meta.impCenterZ*1000;
text(xC, zC, '  ← 叶轮/毂 (绕 y 轴旋转, 红橙=被带动气流)', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.8 0.1 0.1]);
text(xC-80, zC-430, '蜗壳底部切向出口 → 渐扩短管 (+x)', 'FontSize', 10, 'Color', [0 0.4 0.8]);
text(80, -200, '前置扩散段 → 稳定段 → 收缩段 → 试验段', 'FontSize', 10);
xlabel('x (mm)'); ylabel('z (mm)');
title(sprintf('风机耦合域纵剖 y=0 (侧置布局, %s)', dU(imx).name));
saveas(gcf, fullfile(RT,'hub_section.png'));
fprintf('hub_section.png 已存\n');
