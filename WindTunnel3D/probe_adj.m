% probe_adj — 叶片贴壁流体速度核验 (动壁耦合生效的最直接证据)
% 侧置布局 (2026-09-30): 叶轮绕 y 轴旋转, 截面在 xoz 平面, 蜗壳心 (impCenterX/Z)
RT = fileparts(mfilename('fullpath'));   % 项目根 (套件内, 无盘符硬编码)
addpath(RT);                             % read_fluidx3d_vtk 等
M = load(fullfile(RT,'gpu_meta.mat'));  meta = M.meta;
nx = meta.nx;  ny = meta.ny;  nz = meta.nz;  dx = meta.dx;
xC = meta.impCenterX;  zC = meta.impCenterZ;  wLx = meta.impOmega*meta.dt;  % 晶格角速度 (绕 +y, 带符号)
if isfield(meta,'y0'), y0 = meta.y0; else, y0 = -(ny*dx)/2; end
if isfield(meta,'z0'), z0 = meta.z0; else, z0 = -(nz*dx)/2; end
dU = dir(fullfile(RT,'fluidx3d','vtk','u_*.vtk'));
assert(~isempty(dU), '无 VTK 快照 — 先跑 windtunnel_fan.exe');
[~, imx] = max([dU.datenum]);
A = read_fluidx3d_vtk(fullfile(dU(imx).folder, dU(imx).name));
fid = fopen(fullfile(RT,'fluidx3d','windtunnel.raw'),'rb');
raw = fread(fid, inf, 'uint8');  fclose(fid);
raw = reshape(raw, nx, ny, nz);
rot = raw == 2;
% 贴壁流体: rot 膨胀一层 ∩ 流体
adj = rot;
adj(2:end,:,:)   = adj(2:end,:,:)   | rot(1:end-1,:,:);
adj(1:end-1,:,:) = adj(1:end-1,:,:) | rot(2:end,:,:);
adj(:,2:end,:)   = adj(:,2:end,:)   | rot(:,1:end-1,:);
adj(:,1:end-1,:) = adj(:,1:end-1,:) | rot(:,2:end,:);
adj(:,:,2:end)   = adj(:,:,2:end)   | rot(:,:,1:end-1);
adj(:,:,1:end-1) = adj(:,:,1:end-1) | rot(:,:,2:end);
adj = adj & rot == 0 & raw == 0;
% 绕 +y 涡旋速度: s = (ux*z' - uz*x')/r (叶片格应 ≈ wLx·r_lx)
[ii, jj, kk] = ind2sub(size(rot), find(rot));
swB = zeros(numel(ii),1);  rB = swB;
for t = 1:numel(ii)
    xr = meta.x0 + (ii(t)-0.5)*dx - xC;  zr = z0 + (kk(t)-0.5)*dx - zC;
    swB(t) = (A(ii(t),jj(t),kk(t),1)*zr - A(ii(t),jj(t),kk(t),3)*xr)/max(hypot(xr,zr),eps);
    rB(t) = hypot(xr,zr);
end
% 贴壁流体涡旋
[i2, j2, k2] = ind2sub(size(adj), find(adj));
swF = zeros(numel(i2),1);  rF = swF;
for t = 1:numel(i2)
    xr = meta.x0 + (i2(t)-0.5)*dx - xC;  zr = z0 + (k2(t)-0.5)*dx - zC;
    swF(t) = (A(i2(t),j2(t),k2(t),1)*zr - A(i2(t),j2(t),k2(t),3)*xr)/max(hypot(xr,zr),eps);
    rF(t) = hypot(xr,zr);
end
wExp = wLx*median(rB)/dx;   % 期望叶片格涡旋 (lx)
fprintf('叶片格 n=%d: 涡旋速度中位 %+.4f lx (期望 ≈ %+.4f, wLx=%+.5f) | r 中位 %.3f m\n', ...
    numel(ii), median(swB), wExp, wLx, median(rB));
fprintf('贴壁流体 n=%d: 涡旋速度中位 %+.4f lx = %+.2f m/s | 方向一致性 %+.0f%%\n', ...
    numel(i2), median(swF), median(swF)*meta.velScale, ...
    100*mean(sign(swF)==sign(wLx)));
% 沿径向的旋流衰减 (xoz 平面环带, 叶轮 x 带内)
xv = meta.x0 + ((1:nx)-0.5)'*dx;
zv = z0 + ((1:nz)-0.5)*dx;
X3 = reshape(xv - xC, nx, 1, 1);  Z3 = reshape(zv - zC, 1, 1, nz);
RC = sqrt(X3.^2 + Z3.^2);
vtAll = (A(:,:,:,1).*Z3 - A(:,:,:,3).*X3);
cav = abs(xv - xC) <= 0.23;
for rr = {[0.24 0.28], [0.28 0.35], [0.35 0.45]}
    m = reshape(cav, [], 1, 1) & RC >= rr{1}(1) & RC <= rr{1}(2) & raw == 0 & isfinite(vtAll);
    fprintf('环带 r %.2f-%.2f: 涡旋 %+.3f m/s\n', rr{1}(1), rr{1}(2), ...
        mean(vtAll(m))*meta.velScale);
end
