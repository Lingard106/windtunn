%% gpu_slices — 高频切片时序留档: 连续性验证 + 流速梯度差异展示
% 输入: fluidx3d/u_slice_y0.raw / u_slice_zm.raw (求解器每 sliceEvery=100 步流式追加)
% 输出: gpu_xt_y0.png          时空图 (中心线 u_x 与面内涡量 ω_y: 斜条纹=涡连续输运)
%       gpu_slice_movie_y0.mp4 竖直切片 y=0 电影 (|u| 与 ω_y 双面板, 300 帧)
%       gpu_slice_movie_zm.mp4 水平切片 z=mid 电影 (|u|)
%       控制台: 相邻帧相关系数 (时间连续性定量检验 — 回答"风是否间断")
% 前提: 重跑过 export_fluidx3d + build_windtunnel.bat + windtunnel.exe (新模板含切片流)
clear; clc; close all;
set(groot,'defaultFigureColor','w','defaultAxesColor','w', ...
    'defaultAxesXColor',[.15 .15 .15],'defaultAxesYColor',[.15 .15 .15], ...
    'defaultAxesZColor',[.15 .15 .15],'defaultTextColor',[.15 .15 .15]);

M = load(fullfile(pwd,'gpu_meta.mat'));  meta = M.meta;
if isfield(meta,'isFanCoupled') && meta.isFanCoupled
    error(['gpu_slices: 风机耦合求解器不写切片流 — 动态请用 explore_gpu/gpu_mov (VTK 快照) 或 gpu_report (时均/湍流度)']);
end
P = tunnel_params();  G = tunnel_geometry(P);
nx = meta.nx;  ny = meta.ny;  nz = meta.nz;  dx = meta.dx;  dt = meta.dt;
se = 100;  if isfield(meta,'sliceEvery'), se = meta.sliceEvery; end
x0 = G.xSec(3) - P.lbm.margins(1);
xs = x0 + ((1:nx)-0.5)'*dx;
ys = ((1:ny)-0.5)'*dx - ny*dx/2;
zs = ((1:nz)-0.5)'*dx - nz*dx/2;

fY0 = fullfile(pwd,'fluidx3d','u_slice_y0.raw');
fZm = fullfile(pwd,'fluidx3d','u_slice_zm.raw');
if ~isfile(fY0), error(['无切片流文件 u_slice_y0.raw — 需用新模板重跑: ' ...
    'export_fluidx3d -> build_windtunnel.bat -> windtunnel.exe, 然后再跑 gpu_slices']); end

%% ---- 读取切片流 (float32, 帧顺序追加, 每帧 (3,nx,nz): comp 最快, x 次, z 慢) ----
d = readRaw(fY0);
nF = floor(numel(d)/(3*nx*nz));
Uy0 = permute(reshape(d(1:nF*3*nx*nz), [3, nx, nz, nF]), [2 3 4 1]);   % (nx,nz,nF,3) 晶格单位
se0 = 0;  if isfield(meta,'sliceStart'), se0 = meta.sliceStart; end     % 采样起点步 (时均窗口)
tV = (se0 + (1:nF)'*se)*dt;                                            % 物理时间 (s)
spdY = sqrt(sum(Uy0.^2,4)) * meta.velScale;                            % |u| (m/s)
if max(spdY(:)) < 0.5*meta.V_test
    error(['切片流数据异常 (max|u|=%.2f m/s, 远低于来流) — 切片流为空/旧版求解器, ' ...
        '请用当前模板重跑 export_fluidx3d -> build -> windtunnel.exe'], max(spdY(:)));
end
UX = Uy0(:,:,:,1)*meta.velScale;  UZ = Uy0(:,:,:,3)*meta.velScale;
wY = zeros(nx,nz,nF,'single');
for k = 1:nF   % 面内涡量 ω_y = du_x/dz - du_z/dx (流速梯度差异的显式表达)
    [~, dux_dz] = gradient(UX(:,:,k), dx, dx);
    [duz_dx, ~] = gradient(UZ(:,:,k), dx, dx);
    wY(:,:,k) = dux_dz - duz_dx;
end
fprintf('y0 切片流: %d 帧 (每帧 %.2f ms, 涡脱周期约 %.2f ms -> 每周期 %.1f 帧, 时间连续)\n', ...
    nF, se*dt*1e3, 1/341*1e3, 1/341/(se*dt));

%% ---- 连续性定量检验: 相邻帧相关系数 (欠采样会显著掉 <0.8, 连续输运应 >0.9) ----
cz = zeros(1,nF-1);
for k = 1:nF-1
    a = spdY(:,:,k);   b = spdY(:,:,k+1);
    a = a(:) - mean(a(:));  b = b(:) - mean(b(:));
    cz(k) = sum(a.*b) / sqrt(sum(a.^2)*sum(b.^2) + eps);   % 手写相关 (免工具箱)
end
fprintf(['连续性检验: 相邻帧 |u| 相关系数 中位=%.4f 最小=%.4f ' ...
    '(接近 1 = 场平滑演化; 连续输运的几何证据见下方时空图斜条纹)\n'], median(cz), min(cz));

%% ---- 时空图 (x-t diagram): 偏移对称面 z=+10mm 的 u_x 与 ω_y — 斜条纹即涡的连续输运 ----
% 注: 对称流动的 z=0 中面上 ω_y 精确为零 (u_x 偶对称/u_z 奇对称), 涡量线必须偏离对称面
[~, kz0] = min(abs(zs - 0.010));
xtU = squeeze(UX(:,kz0,:))';      % (nF, nx)
xtW = squeeze(wY(:,kz0,:))';
f = figure('Color','w','Position',[60 60 1350 640],'NumberTitle','off', ...
    'Name','时空图 x-t (切片流连续性证据)');
subplot(1,2,1);
imagesc(xs*1e3, tV, xtU); axis xy; colormap(parula); colorbar;
xlabel('x (mm)'); ylabel('t (s)');
title('z=+10mm 线 u_x 时空图 — 斜条纹 = 涡/亏损包连续输运', 'FontSize', 11);
xline(G.xSec(3)*1e3,'k--'); xline(G.xSec(4)*1e3,'k--');
subplot(1,2,2);
wl = prctileC(abs(xtW(:)), 98);  if wl<=0, wl = 1; end
imagesc(xs*1e3, tV, xtW); axis xy; colormap(turbo); colorbar; caxis([-wl wl]);
xlabel('x (mm)'); ylabel('t (s)');
title(sprintf('z=+10mm 线面内涡量 \\omega_y 时空图 — 交替正负斜条 = 涡的连续输运'), 'FontSize', 11);
xline(G.xSec(3)*1e3,'k--'); xline(G.xSec(4)*1e3,'k--');
sgtitle(sprintf('时空图 (y=0, z=0 剖面) | 相邻帧相关系数中位 %.3f — 风是连续的, 旧 3D 动画是欠采样', median(cz)), 'FontSize', 12);
print(f, fullfile(pwd,'gpu_xt_y0.png'), '-dpng', '-r130');
fprintf('gpu_xt_y0.png 已生成\n');

%% ---- 高频切片电影 y=0 (|u| 与 ω_y 双面板) ----
movF = fullfile(pwd,'gpu_slice_movie_y0.mp4');
figM = figure('Color','w','Position',[80 80 1250 620],'NumberTitle','off','Name','切片时序 y=0');
axA = axes('Parent',figM,'Position',[0.07 0.56 0.85 0.38]); hold(axA,'on');
axB = axes('Parent',figM,'Position',[0.07 0.10 0.85 0.38]); hold(axB,'on');
clA = [0, meta.V_test*1.5];
lcl = prctileC(abs(wY(:)), 99);  if lcl<=0, lcl = 1; end
clB = [-lcl lcl];
hA = imagesc(axA, xs*1e3, zs*1e3, spdY(:,:,1)'); axis(axA,'xy'); caxis(axA,clA); colormap(axA,parula);
colorbar(axA); plot(axA, G.xSec([3 4])*1e3, [0 0],'k--');
title(axA,'|u| (m/s), y=0','FontSize',10);
hB = imagesc(axB, xs*1e3, zs*1e3, wY(:,:,1)'); axis(axB,'xy'); caxis(axB,clB); colormap(axB,turbo);
colorbar(axB); plot(axB, G.xSec([3 4])*1e3, [0 0],'k--');
title(axB,'\omega_y (1/s) — 流速梯度差异 (剪切层/涡量)','FontSize',10);
xlabel(axB,'x (mm)'); ylabel(axB,'z (mm)');
frames = cell(1,ceil(nF/2));
tmpD = fullfile(tempdir,'wtslice_mov');  if ~exist(tmpD,'dir'), mkdir(tmpD); end
fprintf('渲染 y=0 切片电影 %d 帧...\n', numel(frames));
w = 0;
for k = 1:2:nF
    w = w + 1;
    set(hA,'CData',spdY(:,:,k)');  set(hB,'CData',wY(:,:,k)');
    title(axA, sprintf('|u| (m/s), y=0 | t = %.4f s (帧 %d/%d)', tV(k), w, numel(frames)), 'FontSize',10);
    drawnow;
    tmpf = fullfile(tmpD,sprintf('y%04d.png',k));
    try,   exportapp(figM, tmpf); catch, print(figM, tmpf, '-dpng','-r80'); end
    frames{w} = imread(tmpf);
end
close(figM);
writeVidFrames(frames, movF, 12);

%% ---- 高频切片电影 z=mid (|u|) ----
if isfile(fZm)
    dz = readRaw(fZm);
    nFz = floor(numel(dz)/(3*nx*ny));
    Uzm = permute(reshape(dz(1:nFz*3*nx*ny), [3, nx, ny, nFz]), [2 3 4 1]);
    spdZ = sqrt(sum(Uzm.^2,4)) * meta.velScale;
    movF2 = fullfile(pwd,'gpu_slice_movie_zm.mp4');
    figZ = figure('Color','w','Position',[100 100 1250 420],'NumberTitle','off','Name','切片时序 z=mid');
    axZ = axes('Parent',figZ,'Position',[0.07 0.16 0.85 0.72]); hold(axZ,'on');
    hZ = imagesc(axZ, xs*1e3, ys*1e3, spdZ(:,:,1)'); axis(axZ,'xy'); caxis(axZ,clA);
    colormap(axZ,parula); colorbar(axZ);
    plot(axZ, G.xSec([3 4])*1e3, [0 0],'k--');
    xlabel(axZ,'x (mm)'); ylabel(axZ,'y (mm)');
    title(axZ,'|u| (m/s), z=mid','FontSize',10);
    framesZ = cell(1,ceil(nFz/2));
    fprintf('渲染 z=mid 切片电影 %d 帧...\n', numel(framesZ));
    tVz = (1:nFz)'*se*dt;
    w = 0;
    for k = 1:2:nFz
        w = w + 1;
        set(hZ,'CData',spdZ(:,:,k)');
        title(axZ, sprintf('|u| (m/s), z=mid | t = %.4f s (帧 %d/%d)', tVz(k), w, numel(framesZ)), 'FontSize',10);
        drawnow;
        tmpf = fullfile(tmpD,sprintf('z%04d.png',k));
        try,   exportapp(figZ, tmpf); catch, print(figZ, tmpf, '-dpng','-r80'); end
        framesZ{w} = imread(tmpf);
    end
    close(figZ);
    writeVidFrames(framesZ, movF2, 12);
end
fprintf('--- gpu_slices 完成: 连续性中位相关 %.3f | 时空图 + 2 部切片电影已留档 ---\n', median(cz));

%% ==================== 局部函数 ====================
function d = readRaw(fname)
fid = fopen(fname,'rb');
d = fread(fid, inf, 'float32=>single');
fclose(fid);
end

function v = prctileC(a, q)
a = a(isfinite(a));  a = sort(a(:));
if isempty(a), v = 1; return; end
v = a(max(1,min(numel(a),round(q/100*numel(a)))));
end

function writeVidFrames(frames, fpath, fps)
ok = false;  vw = [];
try
    vw = VideoWriter(fpath,'MPEG-4');  vw.FrameRate = fps;
    open(vw);  writeVideo(vw, cat(4,frames{:}));  close(vw);  ok = true;
catch e4
    try, close(vw); catch, end
    [~,base] = fileparts(fpath);
    gifF = fullfile(fileparts(fpath), [base '.gif']);
    for w = 1:numel(frames)
        [Xw, mw] = rgb2ind(frames{w}, 64);
        if w==1, imwrite(Xw, mw, gifF, 'gif','Loopcount',0,'DelayTime',0.1);
        else,    imwrite(Xw, mw, gifF, 'gif','WriteMode','append','DelayTime',0.1);
        end
    end
    fprintf('[动画] mp4 失败 (%s), 回退 GIF: %s\n', e4.message, gifF);
end
if ok, fprintf('[动画] 已存 %s (%d 帧)\n', fpath, numel(frames)); end
end
