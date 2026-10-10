%% gpu_report — 风洞 + 被测物 双重测算一页仪表盘 (GPU 管线结果)
% 输入: gpu_meta.mat + windtunnel.raw + fluidx3d/vtk/{mean_u,tu}.vtk + fluidx3d/forces.csv
% 输出: gpu_report.png (3×2 六联: 流场质量 / 边界层 / 气动力 / 涡脱频谱 / 汇总)
% 前提: 已跑完 export_fluidx3d → build → windtunnel.exe; 参数与导出时一致
clear; clc; close all;
set(groot,'defaultFigureColor','w','defaultAxesColor','w', ...
    'defaultAxesXColor',[.15 .15 .15],'defaultAxesYColor',[.15 .15 .15], ...
    'defaultAxesZColor',[.15 .15 .15],'defaultTextColor',[.15 .15 .15]);

R = pwd;
meta = load(fullfile(R,'gpu_meta.mat'));  meta = meta.meta;
P = tunnel_params();  G = tunnel_geometry(P);
[x0, G, isFan, objVal, rotVal] = gpu_fan_compat(G, P, meta);   % 风机耦合兼容
dx = meta.dx;  velS = meta.velScale;  nx = meta.nx;  ny = meta.ny;  nz = meta.nz;

% 网格坐标 (与 explore_gpu 同口径; 风机耦合域 y/z 不对称, 用 meta 原点)
xs = x0 + ((1:nx)-0.5)'*dx;
if isfield(meta,'y0'), y0 = meta.y0; else, y0 = -(ny*dx)/2; end
if isfield(meta,'z0'), z0 = meta.z0; else, z0 = -(nz*dx)/2; end
ys = y0 + ((1:ny)-0.5)'*dx;
zs = z0 + ((1:nz)-0.5)'*dx;

% 涵道/被测物 mask
fid = fopen(fullfile(R,'windtunnel.raw'),'rb');
raw = reshape(fread(fid, inf, 'uint8=>uint8'), [nx,ny,nz]);  fclose(fid);
mask = raw >= 1;  objM = raw == objVal;
if isFan
    fprintf('[风机耦合模式] 报告口径: 试验段指标照常; 风源=叶轮 (Cfd 工况点见 fan_diag.csv)\n');
end

% 时均场与湍流度
U3 = read_fluidx3d_vtk(fullfile(R,'fluidx3d','vtk','mean_u.vtk'))*velS;
UM = sqrt(sum(U3.^2,4));                    % 时均流速 |Ū| (m/s)
TU = read_fluidx3d_vtk(fullfile(R,'fluidx3d','vtk','tu.vtk'));   % Tu (%)
UM(mask) = NaN;  TU(mask) = NaN;

% 力时程
Fc = readmatrix(fullfile(R,'fluidx3d','forces.csv'));
Fc(Fc(:,1)==0,:) = [];
tF = Fc(:,1)*meta.dt;
qRef = 0.5*meta.rho*meta.V_test^2*meta.Aref;
Cd_t = Fc(:,2)/qRef;  Cl_t = Fc(:,4)/qRef;

%% ---- 风洞侧测算 (试验段流场质量; 有物体时取上游无物体区 = 风洞自身品质) ----
aI = interp1(G.x, G.a, xs, 'pchip', 'extrap');
bI = interp1(G.x, G.b, xs, 'pchip', 'extrap');
[XX, YY, ZZ] = ndgrid(xs, ys, zs);
xUpH = G.xSec(4);
if any(objM(:))
    iF = find(any(objM, [2 3]), 1, 'first');
    if iF > 2, xUpH = min(G.xSec(4), xs(iF) - 2*dx); end   % 物体前缘前 2 格
end
core = ~mask & xs(:)>=G.xSec(3) & xs(:)<=xUpH ...
     & abs(YY) < aI(:)-3*dx & abs(ZZ) < bI(:)-3*dx;         % 核心区 (壁面 3 格外)
uCore = UM(core);  uCore = uCore(isfinite(uCore));
uMed  = median(uCore);  cv = std(uCore)/mean(uCore)*100;     % 核心均匀度 (风洞自身)
tuCore = TU(core);  tuCore = tuCore(isfinite(tuCore));
tuMeas = median(tuCore);
jj0 = find(min(abs(ys)) == abs(ys), 1);                      % y=0 列
dxF = xUpH - G.xSec(3);
if dxF < 0.15   % 上游窗口小 (大物体): 按窗口比例取站, 避免落入物体影响区
    stas = G.xSec(3) + [0.25 0.60 0.95]*dxF;
else
    stas = [G.xSec(3)+0.05, G.xSec(3)+0.5*dxF, xUpH-0.03];
end
d99 = zeros(1,3);  profs = cell(1,3);
for si = 1:3
    [~, is] = min(abs(xs - stas(si)));
    uCol = squeeze(UM(is, jj0, :));                          % 沿 z 剖面 (底壁→顶壁)
    profs{si} = uCol(:);
    uLoc = median(uCore(abs(xs - stas(si)) < 0.05));         % 该站核心速度
    iz = find(uCol >= 0.99*uLoc & zs < 0, 1);                % 首次达 99% → δ99
    if isempty(iz), iz = 1; end
    d99(si) = zs(iz) - zs(1) + dx;
end
% 物体所在截面的全域核心 (含加速区) — 展示堵塞加速效应
coreAll = ~mask & xs(:)>=G.xSec(3) & xs(:)<=G.xSec(4) ...
     & abs(YY) < aI(:)-3*dx & abs(ZZ) < bI(:)-3*dx;
uAll = UM(coreAll);  uAll = uAll(isfinite(uAll));
uMedAll = median(uAll);

%% ---- 空风洞基线对照 (WT_NOOBJ 运行结果存在时自动加载) ----
hasBase = isfile(fullfile(R,'fluidx3d','vtk','mean_u_base.vtk')) && ...
          isfile(fullfile(R,'fluidx3d','vtk','tu_base.vtk')) && isfile(fullfile(R,'gpu_meta_base.mat'));
cvB = NaN;  tuB = NaN;  d99B = NaN(1,3);  UMb = [];
if hasBase
    Bm = load(fullfile(R,'gpu_meta_base.mat'));
    mb = Bm.metaB;
    if mb.nx==nx && mb.ny==ny && mb.nz==nz && abs(mb.dx-dx)<1e-12
        U3b = read_fluidx3d_vtk(fullfile(R,'fluidx3d','vtk','mean_u_base.vtk'))*mb.velScale;
        UMb = sqrt(sum(U3b.^2,4));
        TUb = read_fluidx3d_vtk(fullfile(R,'fluidx3d','vtk','tu_base.vtk'));
        coreB = ~mask & xs(:)>=G.xSec(3) & xs(:)<=xUpH ...
              & abs(YY) < aI(:)-3*dx & abs(ZZ) < bI(:)-3*dx;
        ucB = UMb(coreB);  ucB = ucB(isfinite(ucB));
        cvB = std(ucB)/mean(ucB)*100;
        tb = TUb(coreB);  tb = tb(isfinite(tb));  tuB = median(tb);
        stasB = G.xSec(3) + [0.25 0.60 0.95]*P.ts.L;
        for si = 1:3
            [~, isb] = min(abs(xs - stasB(si)));
            uc = squeeze(UMb(isb, jj0, :));
            uLocB = median(ucB(abs(xs - stasB(si)) < 0.06));
            izb = find(uc >= 0.99*uLocB & zs < 0, 1);
            if isempty(izb), izb = 1; end
            d99B(si) = zs(izb) - zs(1) + dx;
        end
        fprintf('基线对照已加载: CV=%.2f%%, Tu=%.2f%%\n', cvB, tuB);
    else
        hasBase = false;
        fprintf('基线结果网格不匹配, 跳过对照\n');
    end
end

%% ---- 被测物侧测算 ----
iS = max(1, floor(numel(tF)/2));
Cd = mean(Cd_t(iS:end));  Cl = mean(Cl_t(iS:end));
iW = max(2, floor(numel(tF)/2)):numel(tF);
ClD = detrend(Cl_t(iW));  ClD = ClD - mean(ClD);
NF = 2^nextpow2(numel(ClD));
wn = 0.5 - 0.5*cos(2*pi*(1:numel(ClD))'/numel(ClD));
SP = abs(fft(ClD.*wn, NF));
fr = (0:NF-1)'/(numel(ClD)*mean(diff(tF(iW))));
hIdx = 2:floor(NF/2);
[~, ipk] = max(SP(hIdx));
fPk = fr(hIdx(ipk));  St = fPk*meta.objSize/meta.V_test;
blk = meta.blockage;
if isnan(blk), blk = meta.Aref/(P.ts.W*P.ts.H)*100; end
resCell = min(meta.objBBox)/dx;
objN = 'sphere';  if isfield(meta,'objType'), objN = meta.objType; end
CdLit = NaN;  StLit = NaN;
if strcmpi(objN,'sphere'),    CdLit = 0.5;  end
if strcmpi(objN,'cylinder'),  CdLit = 1.0;  StLit = 0.212;  end

%% ==================== 仪表盘 ====================
f = figure('Color','w', 'Position',[40 40 1500 860], ...
    'Name','WindTunnel3D 双重测算报告', 'NumberTitle','off');
cm = turbo(256);

% (1) 时均流速 竖直中截面
subplot(3,2,1); hold on;
imagesc(xs*1e3, zs*1e3, squeeze(UM(:,jj0,:))'); axis xy;
colormap(cm); caxis([0 meta.V_test*1.3]); colorbar;
plot(xs*1e3,  bI*1e3, 'k-', xs*1e3, -bI*1e3, 'k-', 'LineWidth', 1.2);
if any(objM(:,jj0,:)), contour(xs*1e3, zs*1e3, double(squeeze(objM(:,jj0,:)))', [0.5 0.5], 'b-', 'LineWidth', 1.4); end
xlabel('x (mm)'); ylabel('z (mm)');
title(sprintf('时均流速 |U| (y=0 截面) | 核心中位 %.2f m/s', uMed), 'FontSize', 10);

% (2) 湍流度 同截面
subplot(3,2,2); hold on;
imagesc(xs*1e3, zs*1e3, squeeze(TU(:,jj0,:))'); axis xy;
colormap(cm); caxis([0 max(5, prctile(TU(isfinite(TU)),99))]); colorbar;
plot(xs*1e3,  bI*1e3, 'k-', xs*1e3, -bI*1e3, 'k-', 'LineWidth', 1.2);
if any(objM(:,jj0,:)), contour(xs*1e3, zs*1e3, double(squeeze(objM(:,jj0,:)))', [0.5 0.5], 'b-', 'LineWidth', 1.4); end
xlabel('x (mm)'); ylabel('z (mm)');
title(sprintf('湍流度 Tu (%%) | 外流实测 %.2f%% vs Tu 链设计 %.2f%%', tuMeas, meta.tuInlet*100), 'FontSize', 10);

% (3) 底壁边界层发展 (风洞侧)
subplot(3,2,3); hold on; grid on;
cols = lines(3);
for si = 1:3
    plot(profs{si}, zs*1e3, '-', 'Color', cols(si,:), 'LineWidth', 1.5);
end
lg = {sprintf('x=%.0fmm, \\delta_{99}=%.1fmm', stas(1)*1e3, d99(1)*1e3), ...
      sprintf('x=%.0fmm, \\delta_{99}=%.1fmm', stas(2)*1e3, d99(2)*1e3), ...
      sprintf('x=%.0fmm, \\delta_{99}=%.1fmm', stas(3)*1e3, d99(3)*1e3)};
if hasBase   % 基线 (空风洞) 剖面: 全长站位, 灰虚线
    for si = 1:3
        [~, isb] = min(abs(xs - stasB(si)));
        plot(squeeze(UMb(isb, jj0, :)), zs*1e3, '--', 'Color',[.45 .45 .45], 'LineWidth', 1.1);
    end
    lg{end+1} = '基线 (空风洞, 全长站位)';
end
yline(0, 'k:');
xlabel('u (m/s)'); ylabel('z (mm)');
legend(lg, 'Location','northwest');
title('试验段速度剖面 (底壁边界层发展)', 'FontSize', 10);
ylim([-0.12 0.02]*1e3);

% (4) Cd/Cl 时程
subplot(3,2,4); hold on; grid on;
plot(tF, smoothdata(Cd_t,'movmean',9), 'b-', 'LineWidth', 1.2);
plot(tF, smoothdata(Cl_t,'movmean',9), 'r-', 'LineWidth', 1.0);
if isfinite(CdLit), yline(CdLit, 'k--', sprintf('文献 C_d\\approx%.2f', CdLit)); end
xlabel('t (s)'); ylabel('系数');
legend({sprintf('C_d=%.3f', Cd), sprintf('C_l=%.3f', Cl)}, 'Location','northeast');
title(sprintf('被测物气动力时程 (%s)', objN), 'FontSize', 10);

% (5) Cl 频谱 → St
subplot(3,2,5); hold on; grid on;
semilogx(fr(hIdx), SP(hIdx), 'b-', 'LineWidth', 1.0);
xline(fPk, 'r-', sprintf('f_{peak}=%.0f Hz, St=%.3f', fPk, St), 'LineWidth', 1.2);
if isfinite(StLit), xline(StLit*meta.V_test/meta.objSize, 'k--', sprintf('文献 St=%.3f', StLit)); end
xlabel('f (Hz)'); ylabel('|Cl(f)|');
title('升力频谱与涡脱 (Cl 后半段 FFT)', 'FontSize', 10);
xlim([fr(hIdx(1)) fr(hIdx(end))]);

% (6) 双重测算汇总
subplot(3,2,6); axis off;
txt = [];
fmt = [...
    '被测物: %s   包围盒 %.0f\\times%.0f\\times%.0f mm   最窄特征 %.1f 格\n' ...
    '迎风面积 %.1f cm²   堵塞比 %.1f%%   分辨率建议: 最窄特征 >= 12~15 格\n' ...
    '--------------------------------------------------------------\n' ...
    'C_d = %.3f    C_l = %.3f    St = %.3f (f_{peak} = %.0f Hz)\n' ...
    '名义晶格 Re = %d (LES 有效高 Re)    V_{test} = %.0f m/s\n' ...
    '--------------------------------------------------------------\n' ...
    '风洞侧(上游无物体区): 核心中位 |U| = %.2f m/s (%.0f%% 设计)\n' ...
    '核心非均匀度 CV = %.2f%%    外流 Tu = %.2f%% (设计 %.2f%%)\n' ...
    '边界层 \\delta_{99}: %.1f / %.1f / %.1f mm (试验段入口/中/末端)\n' ...
    '物体区加速: 全试验段核心中位 %.1f m/s (堵塞 %.1f%%)\n' ...
    '注: 有物体时上游区亦受堵塞前影响, 风洞自身品质建议空风洞对照运行\n'];
txt = sprintf(fmt, ...
    objN, meta.objBBox(1)*1e3, meta.objBBox(2)*1e3, meta.objBBox(3)*1e3, resCell*dx*1e3, ...
    meta.Aref*1e4, blk, ...
    Cd, Cl, St, fPk, meta.ReLattice, meta.V_test, ...
    uMed, uMed/meta.V_test*100, cv, tuMeas, meta.tuInlet*100, d99(1)*1e3, d99(2)*1e3, d99(3)*1e3, ...
    uMedAll, blk);
if hasBase && isfinite(cvB)
    txt = [txt, sprintf('基线(空风洞)对照: CV=%.2f%%, Tu=%.2f%%, \\delta_{99}=%.1f/%.1f/%.1f mm (全长站位)\n', ...
        cvB, tuB, d99B(1)*1e3, d99B(2)*1e3, d99B(3)*1e3)];
end
if isfinite(Cd) && Cd>0 && isfinite(blk) && blk>0 && blk<50
    eS = 0.25*(blk/100)*Cd;   % Maskell III 尾流堵塞近似
    txt = [txt, sprintf('C_d 堵塞修正 (Maskell III 近似): %.3f (\\epsilon_S=%.3f, 修正前 %.3f)\n', ...
        Cd/(1+eS)^2, eS, Cd)];
end
text(0.0, 1.0, txt, 'VerticalAlignment','top', 'FontSize', 10.5, 'Interpreter','tex', ...
    'FontName','Consolas', 'Color',[.15 .15 .15]);
title('双重测算汇总', 'FontSize', 10);

sgtitle(sprintf('WindTunnel3D 双重测算报告 | %s | 试验段 %.0f\\times%.0f mm @ %.0f m/s', ...
    objN, P.ts.W*1e3, P.ts.H*1e3, P.V_test), 'FontSize', 13);
print(f, fullfile(R,'gpu_report.png'), '-dpng', '-r130');
fprintf('gpu_report.png 已生成\n');
