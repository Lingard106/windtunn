function explore_fan()
% explore_fan — 风机 GPU 结果交互浏览器 (切片 + 流量时程)
% 用法: explore_fan  (需先跑 export_fan_fluidx3d → build_fan.bat → windfan.exe)
% 套件相对定位: 无盘符硬编码 (FanSim3D 与 WindTunnel3D 同级; 环境变量 WT3D_DIR 可覆盖)
FAN = fileparts(mfilename('fullpath'));
WT = getenv('WT3D_DIR');  if isempty(WT), WT = fullfile(FAN,'..','WindTunnel3D'); end
addpath(WT);
M = load(fullfile(FAN,'fan_meta.mat'));  meta = M.meta;
nx = meta.nx;  ny = meta.ny;  nz = meta.nz;  x0 = meta.x0;  Sv = meta.velScale;

d = dir(fullfile(FAN,'fluidx3d','vtk','u_*.vtk'));
if isempty(d)
    error('无 VTK 快照 — 先跑 windfan.exe 求解');
end
ts = zeros(numel(d),1);
for k = 1:numel(d)
    tok = regexp(d(k).name, 'u_(\d+)u-(\d+)\.vtk', 'tokens', 'once');
    ts(k) = str2double(tok{2});
end
[ts, ord] = sort(ts);  d = d(ord);

T = readmatrix(fullfile(FAN,'fluidx3d','fan_diag.csv'));
tD = T(:,1);  QD = T(:,2);
iQ = tD >= meta.nAvgStart;
Qmean = mean(QD(iQ));  Qstd = std(QD(iQ));

xs = x0 + ((1:nx)-0.5)*meta.dx;
yv = ((1:ny)-0.5-ny/2)*meta.dx;
Lx = meta.Lx;

F = figure('Color','w','Name','FanSim3D explore','Position',[60 60 1360 780]);
axA = subplot(2,2,[1 2]);  hold on; axis equal;
axB = subplot(2,2,3);  hold on; axis equal;
axC = subplot(2,2,4);  hold on; grid on;

yline(axA,  Lx.W_out/2, ':'); yline(axA, -Lx.W_out/2, ':');
yline(axA,  Lx.W_sett/2, ':'); yline(axA, -Lx.W_sett/2, ':');
xline(axA, Lx.xFlange, 'k:');
xline(axA, Lx.xBlade(1), 'r:');  xline(axA, Lx.xBlade(2), 'r:');
hIA = imagesc(axA, xs, yv, zeros(ny, nx)');
set(axA, 'YDir', 'normal');  colormap(axA, turbo);
hcA = colorbar(axA);  ylabel(hcA, '|U| (m/s)');
xlabel(axA, 'x (m)');  ylabel(axA, 'y (m)');
title(axA, '水平切片 z=mid (俯视蜗壳出口链路)');

hIB = imagesc(axB, xs, yv, zeros(nz, nx)');
set(axB, 'YDir', 'normal');  colormap(axB, turbo);
hcB = colorbar(axB);  ylabel(hcB, '|U| (m/s)');
xlabel(axB, 'x (m)');  ylabel(axB, 'z (m)');
title(axB, '竖直切片 y=0 (过轴线: 钟形口→叶轮→出口)');

plot(axC, tD, QD, 'b-', 'LineWidth', 1.2);
plot(axC, tD(iQ), QD(iQ), 'b.', 'MarkerSize', 8);
plot(axC, [tD(1) tD(end)], [meta.opQ meta.opQ], 'r--', 'LineWidth', 1.2);
ylabel(axC, 'Q (m³/s)');  xlabel(axC, 't (步)');
legend(axC, {'Q(t)', '时均段', '1D 工况点'}, 'Location', 'southeast');
title(axC, sprintf('流量时程 | 后半段均值 %.3f±%.3f m³/s (1D 设计: %.2f)', Qmean, Qstd, meta.opQ));

sl = uicontrol('Style','slider','Min',1,'Max',numel(ts),'Value',numel(ts), ...
    'Units','normalized','Position',[0.05 0.01 0.55 0.04], 'BackgroundColor','w');
txt = uicontrol('Style','text','Units','normalized','Position',[0.62 0.01 0.36 0.04], ...
    'String','','BackgroundColor','w','HorizontalAlignment','left');

cache = struct('k',{},'U',{},'mx',{});   % LRU 环形缓存 (上限 4 帧, 防拖动吃满内存)

    function [U, mx] = loadU(idx, t)
        for q = 1:numel(cache.k)
            if cache.k(q) == t
                U = cache.U(q);  mx = cache.mx(q);
                if q < numel(cache.k)   % 命中移到末尾
                    cache.k(end+1) = cache.k(q);  cache.U(end+1) = cache.U(q);  cache.mx(end+1) = cache.mx(q);
                    cache.k(q) = [];  cache.U(q) = [];  cache.mx(q) = [];
                end
                return;
            end
        end
        A = read_fluidx3d_vtk(fullfile(FAN,'fluidx3d','vtk', d(idx).name));
        U = sqrt(sum(A.^2, 4))*Sv;   % 单表达式一次成形 (省 3 个全网格临时)
        clear A
        mx = max(U(:));
        cache.k(end+1) = t;  cache.U(end+1) = U;  cache.mx(end+1) = mx;
        while numel(cache.k) > 4
            cache.k(1) = [];  cache.U(1) = [];  cache.mx(1) = [];
        end
    end

    function upd(~, ~)
        idx = round(get(sl,'Value'));
        t = ts(idx);
        [U, mx] = loadU(idx, t);
        set(hIA, 'CData', squeeze(U(:,:,round(nz/2)))');
        set(hIB, 'CData', squeeze(U(:,round(ny/2),:))');
        set(txt, 'String', sprintf('t=%d (%.0f%%) max|U|=%.1f m/s', t, 100*t/meta.nSteps, mx));
        drawnow;
    end
addlistener(sl, 'ContinuousValueChanged', @upd);
upd();
end
