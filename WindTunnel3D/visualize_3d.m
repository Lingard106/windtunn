%% visualize_3d — 风洞 3D 体渲染视场 (基于体素求解器 flow3d_solve)
% 图1: 空间立体网格体渲染 — 逐单元流场切片墙 + 试验段核心流等值面 + 360° GIF
% 图2: 最小方格单元放大 — 体素立方体着色 + 邻居耦合矢量
% 图3: 边界层/湍流验证 — 壁面律 (粘性底层/对数律/尾迹) 对比 + 竖直截面分层
% 图4: 可压缩性/激波模块 — 马赫数与密度比沿程 + 正激波关系曲线
clear; clc; close all;
set(groot,'defaultFigureColor','w','defaultAxesColor','w', ...
    'defaultAxesXColor',[.15 .15 .15],'defaultAxesYColor',[.15 .15 .15], ...
    'defaultAxesZColor',[.15 .15 .15],'defaultTextColor',[.15 .15 .15]);

config = 'balanced';
if ~isempty(getenv('WT_CONFIG')), config = getenv('WT_CONFIG'); end
P = tunnel_params(config);
if ~isempty(getenv('WT_QUICK'))                     % 秒级冒烟档: 单元 ×6
    P.flow.nx = max(60, round(P.flow.nx/6));
    P.flow.ny = max(32, round(P.flow.ny/6));
    P.flow.nz = max(32, round(P.flow.nz/6));
elseif ~isempty(getenv('WT_MID'))                   % 中档: 单元 ×2, 快速查看
    P.flow.nx = max(60, round(P.flow.nx/2));
    P.flow.ny = max(32, round(P.flow.ny/2));
    P.flow.nz = max(32, round(P.flow.nz/2));
end   % 不设环境变量 = 默认最高档 (tunnel_params 按目标单元尺寸派生)
G = tunnel_geometry(P);

fprintf('=== 运行 3D 体素求解器 (%dx%dx%d, %d sweeps/站) ===\n', P.flow.nx, P.flow.ny, P.flow.nz, P.flow.sweeps);
toc0 = tic;
S = flow3d_solve(P, G);
fprintf('=== 求解耗时 %.1f s ===\n', toc(toc0));

spdN = S.spd;  spdN(~S.mask) = NaN;      % NaN 处不渲染
[Xg, Yg] = meshgrid(S.x, S.y);
[Xg2, Zg] = meshgrid(S.x, S.z);
cmap = turbo(256);
MAKE_GIF = isempty(getenv('WT_QUICK')) && isempty(getenv('WT_NOGIF'));
hIn = max(G.a);

%% ==================== 图 1: 体渲染 3D 视场 ====================
f1 = figure('Position',[40 40 1220 700]); hold on;
axF1 = gca;   % GIF 循环用 (不抢前台焦点)

% (a) 半透明壁面 (分段着色) + 四角棱线 — 空间参照
wallCols = [0.85 0.65 0.25; 0.30 0.50 0.85; 0.90 0.45 0.20; 0.55 0.55 0.60];
Ek = linspace(-1,1,25);
for k = 1:4
    mk = S.x >= G.xSec(k)-1e-9 & S.x <= G.xSec(k+1)+1e-9;
    xk = S.x(mk); ak = interp1(G.x,G.a,xk,'pchip','extrap'); bk = interp1(G.x,G.b,xk,'pchip','extrap');
    [XX, EE] = ndgrid(xk, Ek);
    fc = wallCols(k,:);
    surf(XX, EE.*ak(:),  repmat(bk(:),1,numel(Ek)), 'FaceColor',fc,'FaceAlpha',0.08,'EdgeColor','none');
    surf(XX, EE.*ak(:), -repmat(bk(:),1,numel(Ek)), 'FaceColor',fc,'FaceAlpha',0.08,'EdgeColor','none');
    surf(XX,  repmat(ak(:),1,numel(Ek)), EE.*bk(:), 'FaceColor',fc,'FaceAlpha',0.08,'EdgeColor','none');
    surf(XX, -repmat(ak(:),1,numel(Ek)), EE.*bk(:), 'FaceColor',fc,'FaceAlpha',0.08,'EdgeColor','none');
end
for sy_ = [-1 1], for sz_ = [-1 1]
    plot3(G.x, sy_*G.a, sz_*G.b, '-', 'Color',[0.2 0.2 0.25], 'LineWidth', 1.0);
end, end

% (a2) PLA 粗糙壁边界层壳层 (δ 外缘, 摩擦效应可视化)
if isfield(P,'wall') && P.wall.on
    aI = interp1(G.x,G.a,S.x,'pchip','extrap');
    bI = interp1(G.x,G.b,S.x,'pchip','extrap');
    offA = max(aI(:) - S.delta(:), 0.002);
    offB = max(bI(:) - S.delta(:), 0.002);
    sub = 1:4:numel(S.x);
    [XXs, EEs] = ndgrid(S.x(sub), linspace(-1,1,20));
    aS = offA(sub);  bS = offB(sub);
    blc = [0.98 0.30 0.08];
    surf(XXs, EEs.*aS(:),  repmat(bS(:),1,size(XXs,2)), 'FaceColor',blc,'FaceAlpha',0.20,'EdgeColor','none');
    surf(XXs, EEs.*aS(:), -repmat(bS(:),1,size(XXs,2)), 'FaceColor',blc,'FaceAlpha',0.20,'EdgeColor','none');
    surf(XXs,  repmat(aS(:),1,size(XXs,2)), EEs.*bS(:), 'FaceColor',blc,'FaceAlpha',0.20,'EdgeColor','none');
    surf(XXs, -repmat(aS(:),1,size(XXs,2)), EEs.*bS(:), 'FaceColor',blc,'FaceAlpha',0.20,'EdgeColor','none');
    text(S.x(sub(end)), 0, hIn*1.24, ...
        sprintf('PLA 边界层外缘 (试验段末端 \\delta=%.1fmm)', ...
        S.delta(find(S.x<=G.xSec(4),1,'last'))*1000), ...
        'Color',[0.75 0.15 0.05], 'FontWeight','bold', 'HorizontalAlignment','center');
end

% (b) 体填充: 三向正交薄层致密堆叠 (任意视角无缝) + 逐面元透明度
%      快速核心不透明"发光", 慢速区半透渐隐, 整体呈连续体积
cl0 = [0, P.V_test*1.25];
alph = @(C) 0.07 + 0.28*min(max((C-cl0(1))/diff(cl0),0),1);
% z 向层 (俯视连续)
kz = unique(round(linspace(1, numel(S.z), 48)));
for kk = kz
    Cd = spdN(:,:,kk)';   Al = alph(Cd);
    Al(isnan(Cd)) = 0;  Cd(isnan(Cd)) = 0;
    surf(Xg, Yg, S.z(kk)*ones(size(Xg)), Cd, 'FaceColor','interp', ...
        'FaceAlpha','interp', 'AlphaData', Al, 'EdgeColor','none');
end
% y 向层 (正/侧视连续)
ky = unique(round(linspace(2, numel(S.y)-1, 36)));
[XgZ2, ZgZ2] = meshgrid(S.x, S.z);
for jj = ky
    Cd = squeeze(spdN(:,jj,:))';   Al = alph(Cd);
    Al(isnan(Cd)) = 0;  Cd(isnan(Cd)) = 0;
    surf(XgZ2, S.y(jj)*ones(size(XgZ2)), ZgZ2, Cd, 'FaceColor','interp', ...
        'FaceAlpha','interp', 'AlphaData', Al, 'EdgeColor','none');
end
% x 向层 (横断面连续)
kx = unique(round(linspace(3, numel(S.x)-2, 30)));
[YgX, ZgX] = meshgrid(S.y, S.z);
for ii2 = kx
    Cd = squeeze(spdN(ii2,:,:))';   Al = alph(Cd);
    Al(isnan(Cd)) = 0;  Cd(isnan(Cd)) = 0;
    surf(S.x(ii2)*ones(size(YgX)), YgX, ZgX, Cd, 'FaceColor','interp', ...
        'FaceAlpha','interp', 'AlphaData', Al, 'EdgeColor','none');
end
% 竖直纵剖面 (y=0): 展示竖直界面分层
km = round((numel(S.y)+1)/2);
surf(Xg2, S.y(km)*ones(size(Xg2)), Zg, squeeze(spdN(:,km,:))', ...
    'FaceColor','interp', 'FaceAlpha',0.55, 'EdgeColor','none');

% (c) 试验段核心流等值面 (>=22 m/s 的连通体)
its = S.x >= G.xSec(3)-1e-9 & S.x <= G.xSec(4)+1e-9;
spdTs = S.spd(its,:,:);  spdTs(~S.mask(its,:,:)) = 0;
[Xt, Yt, Zt] = ndgrid(S.x(its), S.y, S.z);
fv = isosurface(Xt, Yt, Zt, spdTs, P.V_test*0.88);
if ~isempty(fv.vertices)
    patch(fv, 'FaceColor',[0.95 0.55 0.1], 'FaceAlpha',0.30, 'EdgeColor','none');
end

% (d) 标注
caxis([0 P.V_test*1.25]); colormap(cmap);
cb = colorbar; cb.Label.String = '流速 (m/s)';
axis equal; xlim([-0.05 G.x(end)+0.05]); ylim([-hIn-0.06, hIn+0.06]); zlim([-hIn-0.06, hIn+0.18]);
view(-42, 16); grid on;
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
title(sprintf('风洞 3D 体渲染视场 [%s] | 体素 %dx%dx%d | 试验段 %.0fx%.0f mm @ %.0f m/s', ...
    P.config, P.flow.nx, P.flow.ny, P.flow.nz, P.ts.W*1000, P.ts.H*1000, P.V_test));
for k = 1:4
    text(mean(G.xSec([k k+1])), 0, hIn*1.30, G.secNames{k}, ...
        'HorizontalAlignment','center', 'FontSize', 11, 'FontWeight','bold');
end

%% ==================== 图 2: 最小方格单元放大 ====================
figure('Position',[70 70 1080 560]); hold on;
% 选取试验段入口子域 (5x4x4 体素, 紧贴下壁面以展示边界层梯度)
i0 = find(S.x >= G.xSec(3), 1);
ii = i0:min(i0+4, numel(S.x));
jw = find(S.y(:) >= -interp1(G.x,G.a,S.x(i0)) & S.y(:) < 0);
jj = jw(1:min(4,numel(jw)));
kk = round((numel(S.z)-3)/2) + (0:3);
drawVoxels(S.x, S.y, S.z, ii, jj, kk, S.spd, S.mask, cmap, [0 P.V_test*1.25]);
% 邻居耦合: 交叉矢量 (放大 25 倍显示)
[Vc, Wc] = deal(zeros(numel(ii),numel(jj),numel(kk)));
for a = 1:numel(ii), for b = 1:numel(jj), for c = 1:numel(kk)
    Vc(a,b,c) = S.V(ii(a),jj(b),kk(c));
    Wc(a,b,c) = S.W(ii(a),jj(b),kk(c));
end, end, end
[Xc, Yc, Zc] = ndgrid(S.x(ii), S.y(jj), S.z(kk));
quiver3(Xc, Yc, Zc, zeros(size(Vc)), Vc*25, Wc*25, 0.4, 'b', 'LineWidth', 1.2, 'MaxHeadSize', 0.6);
axis equal; grid on; view(-38, 14);
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
colormap(cmap); caxis([0 P.V_test*1.25]); colorbar;
title(sprintf(['最小方格单元放大 (试验段入口 5\\times4\\times4 体素) | ' ...
    '单元尺寸 %.1f\\times%.1f\\times%.1f mm | 蓝箭头=邻居耦合的横向速度 (\\times25)'], ...
    S.diag.dx*1000, S.diag.dy*1000, S.diag.dz*1000));

%% ==================== 图 3: 边界层/湍流/分层验证 ====================
figure('Position',[100 100 1150 520]);

% (a) 壁面律对比: 试验段下游站位 (δ 最厚) 的内层剖面 + 子网格模型线
subplot(1,2,1); hold on;
imid = find(S.x >= G.xSec(3) & S.x <= G.xSec(4));
xim = imid(round(numel(imid)*0.9));
profU = squeeze(S.spd(xim,:,:));                 % (ny,nz)
line_ = profU(:, round((numel(S.z)+1)/2));       % z=0 剖面
d = interp1(G.x, G.a, S.x(xim)) - abs(S.y)';      % 到侧壁距离
Re_x = P.V_test*(S.x(xim)-G.xSec(3)+0.08)/P.nu;
cfw = 0.0592*Re_x^(-0.25);  uT = P.V_test*sqrt(cfw/2);
dp = d*uT/P.nu;  up = line_/uT;
mk = d > 0 & d < 8*S.delta(xim);                 % 边界层及其外一环的体素中心
semilogx(dp(mk), up(mk), 'o', 'MarkerSize', 6, 'LineWidth', 1.2);
% 子网格模型剖面 (体素内部按壁面律解析填充)
dl = logspace(0.3, 5, 80);
upModel = min(2.44*log(dl)+5.0, P.V_test*1.05/uT);
plot(dl, upModel, 'm-', 'LineWidth', 1.0);
plot(dl, dl, 'k:', dl, 2.44*log(dl)+5.0, 'k--', 'LineWidth', 1.1);
xlabel('d^+'); ylabel('u^+'); grid on; ylim([0 30]);
legend('体素中心', '子网格壁面律模型', 'u^+=d^+ (粘性底层)', ...
    'u^+=2.44 ln(d^+)+5 (对数律)', 'Location','southeast');
title(sprintf('壁面律验证 @ x=%.0fmm (Re_x=%.2e, \\delta=%.1fmm)', ...
    S.x(xim)*1000, Re_x, S.delta(xim)*1000));

% (b) 竖直界面速度分层: 三个站位全高剖面
subplot(1,2,2); hold on;
cols = lines(3);
stas = [G.xSec(3)+0.05, G.xSec(3)+P.ts.L*0.5, G.xSec(4)-0.02];
for si = 1:3
    [~, is] = min(abs(S.x - stas(si)));
    uProf = squeeze(S.spd(is,:,round((numel(S.z)+1)/2)));
    plot(uProf, S.y*1000, '-', 'Color', cols(si,:), 'LineWidth', 1.6);
end
yline(-P.ts.W/2*1000, 'k:'); yline(P.ts.W/2*1000, 'k:');
grid on; xlabel('u (m/s)'); ylabel('y (mm)');
legend({sprintf('x=%.0fmm', stas(1)*1000), sprintf('x=%.0fmm', stas(2)*1000), ...
    sprintf('x=%.0fmm', stas(3)*1000)}, 'Location','southeast');
title(sprintf('竖直界面速度分层 (双侧壁面层, 中部分层度 %.1f%%)', S.diag.stratMid*100));

%% ==================== 图 4: 可压缩性 / 激波模块 ====================
figure('Position',[120 120 1150 500]);
uCore = interp1(G.x, G.U, S.x, 'pchip', 'extrap');
subplot(1,2,1); hold on;
plot(S.x, interp1(G.x, G.U, S.x, 'pchip', 'extrap')/P.aSound, 'b-', 'LineWidth', 1.5);
yline(0.3, 'r--', 'M=0.3 可压缩门槛');
yline(1.0, 'k-', 'M=1 (激波形成)');
grid on; xlabel('x (m)'); ylabel('核心区马赫数 M');
title(sprintf('沿程马赫数 (当前 M_{max}=%.3f — 不可压区, 无激波)', max(uCore)/P.aSound));

subplot(1,2,2); hold on;
M1 = linspace(1.001, 3, 80);
ns = normalShock(M1, P.gamma);
plot(M1, ns.p02p01, 'r-', 'LineWidth', 1.5); grid on;
plot(M1, ns.p2p1/20, 'b--', 'LineWidth', 1.2);
plot(M1, ns.rho2p1/8, 'm-.', 'LineWidth', 1.2);
xlabel('波前马赫数 M_1'); ylabel('比值 (归一化显示)');
legend('总压恢复 p_{02}/p_{01}', '静压比 p_2/p_1 (\div20)', '密度比 \rho_2/\rho_1 (\div8)');
title('正激波模块 (高速工况启用, 含粘性可压缩扩展接口)');

fprintf('\n=== 激波说明: 设计风速 %.0f m/s → M=%.3f, 不可压缩流场, 无激波。 ===\n', ...
    P.V_test, P.V_test/P.aSound);
fprintf('=== 可压缩/激波模块已内置于 flow3d_solve (M>0.3 密度修正, normalShock 关系式), ===\n');
fprintf('=== 提高 tunnel_params 中 V_test>120 m/s 即自动启用密度修正演示。 ===\n');

%% ==================== 图 1 的 360° 旋转 GIF (最后执行, 不阻塞分析图) ====================
if MAKE_GIF && isgraphics(f1)
    gifFile = fullfile(pwd, 'tunnel_3d_orbit.gif');
    tmpD = fullfile(tempdir, 'wt3d_frames');
    if ~exist(tmpD, 'dir'), mkdir(tmpD); end
    nF = 16;
    fprintf('生成旋转 GIF (%d 帧, 按 Ctrl+C 可跳过)...\n', nF);
    for k = 1:nF
        if ~isgraphics(f1) || ~isvalid(f1)
            fprintf('图窗已关闭, GIF 中止 (其他结果不受影响)\n');
            break;
        end
        set(0,'CurrentFigure',f1);          % 后台渲染: 不抢占当前窗口
        set(f1,'CurrentAxes',axF1);
        camorbit(22.5, 0); drawnow;
        tmpf = fullfile(tmpD, sprintf('f%02d.png', k));
        try
            exportgraphics(f1, tmpf, 'Resolution', 40);
        catch
            print(f1, tmpf, '-dpng', '-r40');   % 回退
        end
        A = imread(tmpf);
        if size(A,3) == 4, A = A(:,:,1:3); end
        [Xind, map] = quantize64(A);
        if k == 1
            imwrite(Xind, map, gifFile, 'gif', 'Loopcount', 0, 'DelayTime', 0.2);
        else
            imwrite(Xind, map, gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', 0.2);
        end
        if mod(k,4)==0, fprintf('  GIF 帧 %d/%d\n', k, nF); end
    end
    if isgraphics(f1), view(-42, 16); end
    if exist(gifFile,'file'), fprintf('已生成 %s\n', gifFile); end
end

%% ==================== 局部函数 ====================
function drawVoxels(xv, yv, zv, ii, jj, kk, C, mask, cmap, clim)
% 逐个体素绘制立方体 (最小方格单元渲染), 颜色=单元平均流速
xv = xv(:)';  yv = yv(:)';  zv = zv(:)';
xe = [xv-(xv(2)-xv(1))/2, xv(end)+(xv(2)-xv(1))/2];
ye = [yv-(yv(2)-yv(1))/2, yv(end)+(yv(2)-yv(1))/2];
ze = [zv-(zv(2)-zv(1))/2, zv(end)+(zv(2)-zv(1))/2];
hold on;
ii = ii(:)';  jj = jj(:)';  kk = kk(:)';   % for 循环要求行向量 (列向量会整列代入)
for a = ii, for b = jj, for c = kk
    if b+1 > numel(ye) || c+1 > numel(ze) || a+1 > numel(xe), continue; end
    if ~mask(a,b,c), continue; end
    val = C(a,b,c);
    if ~isfinite(val), continue; end
    ci = double(max(1, min(256, round((double(val)-clim(1))/diff(clim)*255)+1)));
    v8 = [xe(a) ye(b) ze(c); xe(a+1) ye(b) ze(c); xe(a+1) ye(b+1) ze(c); xe(a) ye(b+1) ze(c); ...
          xe(a) ye(b) ze(c+1); xe(a+1) ye(b) ze(c+1); xe(a+1) ye(b+1) ze(c+1); xe(a) ye(b+1) ze(c+1)];
    f6 = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    patch('Vertices', v8, 'Faces', f6, 'FaceColor', cmap(ci,:), ...
        'FaceAlpha', 0.88, 'EdgeColor', [0.15 0.15 0.15], 'LineWidth', 0.5);
end, end, end
end

function [X, map] = quantize64(RGB)
    Aq = min(round(double(RGB)/64)*64, 255);
    [m, n, c] = size(Aq);
    [cols, ~, ic] = unique(reshape(Aq, [], c), 'rows');
    map = cols/255;
    X = reshape(ic, m, n);
end
