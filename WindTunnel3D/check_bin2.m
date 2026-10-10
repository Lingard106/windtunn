% check_bin2 — bin/raw 一致性核验 (侧置布局: 叶轮截面在 xoz 平面, 轴=y, 心=impCenterX/Z)
RT = fileparts(mfilename('fullpath'));   % 项目根 (套件内, 无盘符硬编码)
fid = fopen(fullfile(RT,'fan_blade.bin'),'rb');  D = fread(fid, inf, 'float32');  fclose(fid);
D = reshape(D, 3, []);
M = load(fullfile(RT,'gpu_meta.mat'));  m = M.meta;
fprintf('bin N=%d blade0: n=%d r=%.1f lx=%.3f m (dx=%.1fmm)\n', size(D,2), D(1,1)+1, D(2,1), D(2,1)*m.dx, m.dx*1e3);
fid = fopen(fullfile(RT,'fluidx3d','windtunnel.raw'),'rb');  raw = fread(fid, inf, 'uint8');  fclose(fid);
fprintf('磁盘 raw numel=%d vs meta %d\n', numel(raw), m.nx*m.ny*m.nz);
raw = reshape(raw, m.nx, m.ny, m.nz);
n0 = D(1,1) + 1;
[ix, iy, iz] = ind2sub(size(raw), n0);
xc = m.impCenterX;  zc = m.impCenterZ;
if isfield(m,'y0'), y0 = m.y0; else, y0 = -(m.ny*m.dx)/2; end
if isfield(m,'z0'), z0 = m.z0; else, z0 = -(m.nz*m.dx)/2; end
x = m.x0 + (ix-0.5)*m.dx;  z = z0 + (iz-0.5)*m.dx;  y = y0 + (iy-0.5)*m.dx;
fprintf('raw(n0)=%d 位置 x=%.3f y=%.3f z=%.3f r=%.3f m\n', raw(n0), x, y, z, hypot(x-xc, z-zc));
xx = m.x0 + ((1:m.nx)-0.5)*m.dx;
zz = z0 + ((1:m.nz)-0.5)*m.dx;
R2 = zeros(m.nx, m.nz);
for a = 1:m.nx, R2(a,:) = hypot(xx(a)-xc, zz-zc); end
rr = repmat(reshape(R2, m.nx, 1, m.nz), [1, m.ny, 1]);
v = rr(raw == 2);
fprintf('raw==2 半径 (xoz 系): min=%.3f p50=%.3f max=%.3f (叶片应 0.138~0.230)\n', min(v), median(v), max(v));
% 旋向自检: bin 存物理角 (xoz 系), 与蜗壳螺旋/出口扇区配对
ph = D(3,:);
fprintf('叶片格角域: %.0f°~%.0f° (应覆盖全圆) | omega=%.1f rad/s (绕+y, spinS=%+d)\n', ...
    min(ph)*180/pi, max(ph)*180/pi, m.impOmega, m.impSpin);
