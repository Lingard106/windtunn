%% lbm_run — 动态风场联合仿真驱动: 涵道 + 测试物 (LBM)
% 流程: 体素化涵道+测试物 → D3Q19 LBM 瞬态求解 → 气动力/涡脱落分析 → 3D 渲染 + GIF
% 测试物: P.lbm.object = 'sphere'|'cylinder'|'plate'|'naca'|'stl' (风源视为均匀来流)
% 环境变量: WT_QUICK=1 快速档; WT_NOGIF=1 跳过 GIF; WT_OBJ=<类型> 覆盖测试物
clear; clc; close all;
set(groot,'defaultFigureColor','w','defaultAxesColor','w', ...
    'defaultAxesXColor',[.15 .15 .15],'defaultAxesYColor',[.15 .15 .15], ...
    'defaultAxesZColor',[.15 .15 .15],'defaultTextColor',[.15 .15 .15]);

P = tunnel_params();
if getenv('WT_OBJ'), P.lbm.object = getenv('WT_OBJ'); end
if strcmpi(P.lbm.object,'stl') && ~exist(P.lbm.objFile,'file')
    fprintf(['[提示] 未找到测试物 STL (%s) — 请把你的 CAD 模型导出为 STL 放到该路径\n' ...
        '        (包围盒中心会自动对齐试验段中心)。本次先用 sphere 演示。\n'], P.lbm.objFile);
    P.lbm.object = 'sphere';
end
hasGPU = false;
try
    hasGPU = license('test','Distrib_Computing_Toolbox') && gpuDeviceCount > 0;
catch
    hasGPU = false;
end
if ~isempty(getenv('WT_QUICK'))
    P.lbm.dx = 0.008;  P.lbm.steps = 3000;  P.lbm.snapEvery = 150;  P.lbm.Re = 250;  P.lbm.margins = [0.06 0.12];
elseif ~hasGPU
    P.lbm.dx = 0.006;  P.lbm.steps = 2400;  P.lbm.snapEvery = 150;
    fprintf('[提示] 未检测到 GPU, 已自动降档 (dx=6mm, 2400 步); 装 Parallel Computing Toolbox 可跑全档\n');
end

%% ---- 体素化涵道 + 测试物 ----
G = tunnel_geometry(P);
[solid, objMask, gridG] = tunnel_voxelize(P, G);
fprintf('涵道体素: %dx%dx%d, 测试物 %s (%.0fmm) 占 %d 格\n', ...
    gridG.nx, gridG.ny, gridG.nz, P.lbm.object, P.lbm.objSize*1000, nnz(objMask));;

%% ---- LBM 瞬态求解 ----
S = lbm3d_solve(P, solid, objMask, gridG);
save(fullfile(pwd,'lbm_result.mat'), 'S', 'P');

%% ==================== 图 1: 气动力/探针时程 + 客观性基准 ====================
figure('Color','w','Position',[40 40 1150 520]);
subplot(1,2,1); hold on;
t = S.probeT(:,1);
plot(t, smoothdata(S.Cd_t,'movmean',50), 'b-', 'LineWidth', 1.4);
plot(t, smoothdata(S.Cl_t,'movmean',50), 'r-', 'LineWidth', 1.1);
grid on; xlabel('t (s)'); ylabel('系数');

% --- 客观性基准: 文献相关式 + 分辨率/堵塞比诊断 ---
nuP = P.nu;  dO = P.lbm.objSize;
ReObj = P.V_test*dO/nuP;                          % 测试物物理雷诺数 (背景参考)
ReDemo = S.diag.Re_demo;                          % 演示雷诺数 (LBM 实际运行值)
ReLit  = ReDemo;                                  % 文献相关式用演示 Re (同 Re 对比才有意义)
nAcross = dO/P.lbm.dx;                            % 直径上的晶格数
blockage = pi*(dO/2)^2/(P.ts.W*P.ts.H)*100;       % 面积堵塞比 (%)
switch lower(P.lbm.object)
    case {'sphere','cylinder'}, CdLit = sphereDragLGW(ReLit);
    otherwise,                     CdLit = NaN;
end
if ~isnan(CdLit)
    yline(CdLit, 'k--', sprintf('文献 C_d=%.2f @Re(演示)=%.0f', CdLit, ReLit));
    dev = (S.Cd-CdLit)/CdLit*100;
    lgStr = sprintf('C_d (LBM %.2f | 文献 %.2f, 偏差 %+.0f%%)', S.Cd, CdLit, dev);
else
    lgStr = sprintf('C_d (均值 %.2f)', S.Cd);
end
legend(lgStr, sprintf('C_l (均值 %.2f)', S.Cl), 'Location','best');
title(sprintf('%s d=%.0fmm | 演示Re=%d | 直径 %.1f 格 | 堵塞 %.1f%%', ...
    P.lbm.object, P.lbm.objSize*1000, P.lbm.Re, nAcross, blockage));
if nAcross < 10
    text(0.02, 0.9, sprintf('警告: 直径仅 %.0f 格, Cd 受体素阶梯影响偏大\n(改进: 降低 dx 或提高 objSize)', ...
        nAcross), 'Units','normalized','Color',[0.8 0.1 0], 'FontSize', 8);
end

% 涡脱频率 (Cl 频谱峰 → Strouhal)
% 只取后半段信号 (剔除来流爬坡瞬态), 去趋势后加 Hann 窗做频谱
iW = round(numel(t)*0.4):numel(t);  dtS = mean(diff(t(iW)));
ClD = detrend(S.Cl_t(iW));  ClD = ClD - mean(ClD);
NF = 2^nextpow2(numel(ClD));
wn = 0.5 - 0.5*cos(2*pi*(1:numel(ClD))'/max(numel(ClD),1));  % Hann 窗(手写)
SP = abs(fft(ClD.*wn, NF));
fr = (0:NF-1)'/(numel(ClD)*dtS);
half = 2:floor(NF/2);
[~, ipk] = max(SP(half));
fShed = fr(half(ipk));
St = fShed*dO/P.V_test;

subplot(1,2,2); hold on;
for q = 1:min(3, size(S.probeT,2)-4)
    plot(t, S.probeT(:,1+q), 'LineWidth', 1.1);
end
grid on; xlabel('t (s)'); ylabel('u (m/s)');
legend('探针1 (下游中心)','探针2','探针3','Location','best');
title('尾流探针速度时程 (动态风场)');

%% ==================== 图 2: 瞬态涡结构 (最后切片) ====================
figure('Color','w','Position',[70 60 1150 560]);
wS = S.vort;  wS(S.mask) = NaN;      % S.mask=固体 (lbm 求解器约定)
wS(repmat(S.x<S.x(1)+0.06,[size(wS,2),1,size(wS,3)]) | ...
   repmat(S.x>S.x(end)-0.06,[size(wS,2),1,size(wS,3)])) = NaN;   % 出入口过渡区
lvl = prctile2(wS, 97);
[kk, jj] = deal(round(size(wS,3)/2), round(size(wS,2)/2));
subplot(1,2,1); hold on;
surf(S.x*1000, S.y*1000, squeeze(wS(:,:,kk))', 'FaceColor','interp','EdgeColor','none');
title(sprintf('水平切面涡量 |ω| (z=中面), 97%% 分位 %.0f 1/s', lvl));
xlabel('x (mm)'); ylabel('y (mm)'); colorbar; colormap(turbo); caxis([0 lvl]);
axis tight;

subplot(1,2,2); hold on;
surf(S.x*1000, S.z*1000, squeeze(wS(:,jj,:))', ...
    'FaceColor','interp','EdgeColor','none');
title('竖直切面涡量 |ω| (y=中面)');
xlabel('x (mm)'); zlabel('z (mm)'); colorbar; colormap(turbo); caxis([0 lvl]);
axis tight;

%% ==================== 图 2b: 总压切片时序 (视频同款渲染) ====================
% y=中面总压 p0 = p + ½ρu² 的时序蒙太奇 — 测试物尾流的压力波/亏损随时间演化
fM = figure('Color','w','Position',[60 60 1250 620]);
nT = size(S.p0Sn, 3);
idx = unique(round(linspace(1, nT, min(6, nT))));
xp = S.x(1:2:end)*1000;  zp = S.z(1:2:end)*1000;
pmax = prctile2(S.p0Sn, 99);
objC = squeeze(double(max(objMask(1:2:end, 1:2:end, 1:2:end), [], 2)));                 % 测试物在 x-z 面投影
for q = 1:numel(idx)
    subplot(2, 3, q); hold on;
    imagesc(xp, zp, S.p0Sn(:,:,idx(q))'); axis image;
    colormap(jet); caxis([-0.3*pmax 1.15*pmax]);
    contour(xp, zp, objC', [0.5 0.5], 'b-', 'LineWidth', 1.2);
    title(sprintf('t = %.3f s (步 %d)', S.tSnap(idx(q)), round(S.tSnap(idx(q))/S.diag.dt)));
    xlabel('x (mm)'); ylabel('z (mm)');
end
colormap(jet);
sgtitle('总压切片时序 p_0 (y=中面) — 测试物尾流压力场演化', 'FontSize', 12);
print(fM, fullfile(pwd,'lbm_p0_slices.png'), '-dpng', '-r90');
fprintf('总压切片时序图已存 lbm_p0_slices.png\n');

%% ==================== 图 3: 3D 风洞涵道 + 涡结构 + 测试物 ====================
f3 = figure('Color','w','Position',[90 70 1250 700]); hold on;
camlight('headlight');  material dull;  lighting gouraud;  % 涡面深度可读
% 涵道内壁 (薄壁壳, 半透明) + 四角棱线 — 让轮廓一眼可辨为风洞
aI = interp1(G.x, G.a, S.x, 'pchip', 'extrap');
bI = interp1(G.x, G.b, S.x, 'pchip', 'extrap');
[XXw, EEw] = ndgrid(S.x, linspace(-1,1,25));
ak = aI(:);  bk = bI(:);  nW = size(XXw,1);
fcW = [0.72 0.76 0.82];
surf(XXw, EEw.*ak,  repmat(bk(:),1,25), 'FaceColor',fcW, 'FaceAlpha',0.10, 'EdgeColor','none');
surf(XXw, EEw.*ak, -repmat(bk(:),1,25), 'FaceColor',fcW, 'FaceAlpha',0.10, 'EdgeColor','none');
surf(XXw,  repmat(ak(:),1,25), EEw.*bk, 'FaceColor',fcW, 'FaceAlpha',0.10, 'EdgeColor','none');
surf(XXw, -repmat(ak(:),1,25), EEw.*bk, 'FaceColor',fcW, 'FaceAlpha',0.10, 'EdgeColor','none');
for sy_ = [-1 1], for sz_ = [-1 1]
    plot3(S.x, sy_*aI, sz_*bI, '-', 'Color',[0.2 0.2 0.25], 'LineWidth', 0.8);
end, end
text(S.x(round(nW*0.05)), 0, bI(round(nW*0.05))+0.02, '风洞内壁 (PLA)', ...
    'Color',[0.3 0.3 0.35], 'FontWeight','bold');
% 入口截面标注
plot3(S.x(1)*[1 1 1 1 1], [aI(1) -aI(1) -aI(1) aI(1) aI(1)], ...
    [bI(1) bI(1) -bI(1) -bI(1) bI(1)], 'k-', 'LineWidth', 1.0);
text(S.x(1), 0, -bI(1)-0.02, '入口', 'HorizontalAlignment','center');
text(S.x(end), 0, -bI(end)-0.02, '出口', 'HorizontalAlignment','center');

% 测试物表面
if nnz(S.obj) > 0
    p = isosurface(single(S.obj), 0.5);
    if ~isempty(p.vertices)
        p.vertices(:,1) = S.x(1) + (p.vertices(:,1)-0.5)*S.grid.dx;
        p.vertices(:,2) = S.y(1) + (p.vertices(:,2)-0.5)*S.grid.dx;
        p.vertices(:,3) = S.z(1) + (p.vertices(:,3)-0.5)*S.grid.dx;
        patch(p, 'FaceColor',[0.2 0.5 0.9], 'FaceAlpha',0.9, 'EdgeColor','none');
    end
end
% 涡量等值面: 剔除贴壁 2 层/测试物邻域/出入口过渡区 + 轻度平滑 → 连续涡环结构
wF = S.vort;  wF(S.mask) = 0;
dil = S.mask;
for r = 1:2
    d = dil;
    d(2:end,:,:)   = d(2:end,:,:)   | dil(1:end-1,:,:);
    d(1:end-1,:,:) = d(1:end-1,:,:) | dil(2:end,:,:);
    d(:,2:end,:)   = d(:,2:end,:)   | dil(:,1:end-1,:);
    d(:,1:end-1,:) = d(:,2:end,:)   | dil(:,1:end-1,:);
    d(:,:,2:end)   = d(:,:,2:end)   | dil(:,:,1:end-1);
    d(:,:,1:end-1) = d(:,:,2:end)   | dil(:,:,1:end-1);
    dil = d;
end
wF(dil) = 0;
dilO = objMask;
d = dilO;
d(2:end,:,:)   = d(2:end,:,:)   | dilO(1:end-1,:,:);
d(1:end-1,:,:) = d(1:end-1,:,:) | dilO(2:end,:,:);
d(:,2:end,:)   = d(:,2:end,:)   | dilO(:,1:end-1,:);
d(:,1:end-1,:) = d(:,2:end,:)   | dilO(:,1:end-1,:);
d(:,:,2:end)   = d(:,:,2:end)   | dilO(:,:,1:end-1);
d(:,:,1:end-1) = d(:,:,2:end)   | dilO(:,:,1:end-1);
wF(d) = 0;                                              % 测试物邻域角点奇性剔除
xIn  = S.x(1)+0.06;  xOut = S.x(end)-0.06;
wF(repmat(S.x<xIn,[1,size(wF,2),size(wF,3)]) | repmat(S.x>xOut,[1,size(wF,2),size(wF,3)])) = 0;
% 轻度平滑 (两轮 7 点邻域均值) — 消除体素噪声, 呈现连续涡结构
for qs = 1:2
    s6 = 0*wF;
    s6(2:end,:,:) = s6(2:end,:,:) + wF(1:end-1,:,:);  s6(1:end-1,:,:) = s6(1:end-1,:,:) + wF(2:end,:,:);
    s6(:,2:end,:) = s6(:,2:end,:) + wF(:,1:end-1,:);  s6(:,1:end-1,:) = s6(:,2:end,:) + wF(:,1:end-1,:);
    s6(:,:,2:end) = s6(:,:,2:end) + wF(:,:,1:end-1);  s6(:,:,1:end-1) = s6(:,:,1:end-1) + wF(:,:,2:end);
    wF = (wF + s6/6)/2;
end
[X3, Y3, Z3] = ndgrid(S.x, S.y, S.z);
lvl3 = 0.45*(P.V_test/P.lbm.objSize);                 % 物理电平: 0.45×(U/d) 转折频率
if max(wF(:)) < lvl3, lvl3 = prctile2(wF, 99.5); end
fv = isosurface(X3, Y3, Z3, wF, lvl3);
if ~isempty(fv.vertices)
    patch(fv, 'FaceColor',[0.9 0.35 0.1], 'FaceAlpha',0.32, 'EdgeColor','none');
end
plot3(S.x, 0, 0, 'k:');
axis equal; grid on; view(-45, 14);
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
title(sprintf('动态风场 3D: 测试物 (%s) + 涡量等值面 |ω|=%.0f 1/s | C_d=%.2f, C_l=%.2f', ...
    P.lbm.object, lvl3, S.Cd, S.Cl));
fprintf('图 1-3 已生成 (主界面 + 分析图)\n');

% 图窗存 PNG (批量模式下也能查看)
fs = findall(0,'Type','figure');
for q = 1:numel(fs)
    try
        print(fs(q), '-dpng', '-r85', sprintf('lbm_fig%d.png', fs(q).Number));
    catch
    end
end

%% ==================== 旋转 GIF (最后, 可跳过) ====================
if isempty(getenv('WT_NOGIF'))
    fG = f3;   % 3D 图
    gifFile = fullfile(pwd,'lbm_transient.gif');
    tmpD = fullfile(tempdir,'lbm_frames');
    if ~exist(tmpD,'dir'), mkdir(tmpD); end
    nF = 14;
    fprintf('生成 LBM 瞬态 GIF (%d 帧, Ctrl+C 可跳过)...\n', nF);
    axG = findall(fG,'Type','axes');
    for k = 1:nF
        if ~isgraphics(fG) || ~isvalid(fG), fprintf('图窗关闭, GIF 中止\n'); break; end
        set(0,'CurrentFigure',fG);  set(fG,'CurrentAxes',axG(1));
        camorbit(25.7,0); drawnow;
        tmpf = fullfile(tmpD, sprintf('f%02d.png',k));
        try,   exportgraphics(fG, tmpf, 'Resolution', 40);
        catch, print(fG, tmpf, '-dpng','-r40'); end
        A = imread(tmpf);  if size(A,3)==4, A=A(:,:,1:3); end
        [Xq, map] = quantize64(A);
        if k==1, imwrite(Xq,map,gifFile,'gif','Loopcount',0,'DelayTime',0.2);
        else,    imwrite(Xq,map,gifFile,'gif','WriteMode','append','DelayTime',0.2);
        end
    end
    if isgraphics(fG), view(-45,14); end
    fprintf('已生成 %s\n', gifFile);
end

savefig(gcf, 'lbm_3d_final.fig');
fprintf('--- lbm_run 完成 (lbm_result.mat 已保存) ---\n');

%% ==================== 客观性验证打印 (涡脱频率 vs 文献) ====================
fprintf('\n===== 客观性验证 =====\n');
fprintf('测试物: %s d=%.0fmm | 物理Re=%.0f | 直径 %.1f 晶格 | 堵塞比 %.1f%%\n', ...
    P.lbm.object, dO*1000, ReObj, nAcross, blockage);
if ~isnan(CdLit)
    fprintf('Cd: LBM=%.2f vs 文献=%.2f (偏差 %+.0f%%) — 体素阶梯与低分辨率使阻力偏大\n', ...
        S.Cd, CdLit, (S.Cd-CdLit)/CdLit*100);
end
if any(strcmpi(P.lbm.object, {'cylinder','sphere'}))
    StLit = strouhalCyl(ReObj);
    fprintf('涡脱: LBM St=%.3f (f=%.1f Hz) vs 圆柱文献 St≈%.3f\n', St, fShed, StLit);
    fprintf('(球体绕流为非规则脱落, St 仅作量级参考)\n');
end
fprintf('分辨率建议: 直径晶格数<10 时 Cd 偏大; 正式测量请 dx≤%.2fmm\n', dO/12*1000);
fprintf('======================\n');

%% ==================== 局部函数 ====================
function Cd = sphereDragLGW(Re)
% 圆球阻力系数 — Clift, Grace & Weber (1978) 分段相关式
Re = max(Re, 0.05);
if Re <= 0.01
    Cd = 24/Re;
elseif Re <= 20
    Cd = 24/Re.*(1 + 0.1315*Re.^(0.82 - 0.05*log10(Re)));
elseif Re <= 260
    Cd = 24/Re.*(1 + 0.1935*Re.^0.6305);
elseif Re <= 1500
    Cd = 10.^(1.6435 - 1.1242*log10(Re) + 0.1558*log10(Re).^2);
elseif Re <= 12000
    Cd = 10.^(0.9135 - 0.4887*log10(Re) + 0.04622*log10(Re).^2);
else
    Cd = 0.4;
end
end

function St = strouhalCyl(Re)
% 圆柱涡脱 Strouhal 数 (Williamson 低压段 + 高雷诺常数段)
if Re < 47,      St = 0.2;
elseif Re <= 150, St = 0.212*(1 - 21.2/max(Re,47));
else,             St = 0.212;
end
end
function v = prctile2(A, q)
v = A(isfinite(A));  v = sort(v(:));
if isempty(v), v = 0; return; end
v = v(max(1,min(numel(v),round(q/100*numel(v)))));
end

function [Xq, map] = quantize64(RGB)
Aq = min(round(double(RGB)/64)*64, 255);
[m,n,c] = size(Aq);
[cols,~,ic] = unique(reshape(Aq,[],c),'rows');
map = cols/255;  Xq = reshape(ic,m,n);
end
