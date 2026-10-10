%% fan_sweep — 叶片角度/长度/尺寸比 参数扫描 → 风速·效率·稳定裕度图谱与选型建议
% 核心变量: beta2A (出口安装角) × D1/D2 (决定叶片径向长度 (D2-D1)/2)
% 每个组合: 先在 4 极转速 1440rpm 求工况点, 可行域内再二变转速精确命中 25 m/s
% 输出: fan_sweep.png (风速图/效率图/功率图/裕度图) + fan_sweep_result.mat + 控制台推荐表
clear; clc; close all;
P0 = fan_params();

%% ---- 主扫描网格 (前弯家族为主, 兼顾径向/后弯) ----
beta2A_v = 60:5:165;           % 出口安装角 (deg): 覆盖后弯/径向/前弯全家族
D1D2_v   = 0.60:0.05:0.90;     % 进口直径比 → 叶片长度 = (1-D1D2)·D2/2
nB = numel(beta2A_v);  nD = numel(D1D2_v);
[Vmap, Emap, Pmap, Mmap, RPMmap] = deal(NaN(nD,nB));

for iD = 1:nD
    for iB = 1:nB
        P = P0;
        P.imp.D1 = D1D2_v(iD)*P.imp.D2;
        P.imp.beta2A = beta2A_v(iB);
        try
            R = fan_performance(P, [], 1440, [], true);
        catch
            continue;   % 圆弧无解等几何失败组合
        end
        if ~isfinite(R.op.Q), continue; end
        RPMmap(iD,iB) = 1440;
        Vmap(iD,iB)  = R.op.Vts;
        Emap(iD,iB)  = R.op.etaS;
        Pmap(iD,iB)  = R.op.P;
        Mmap(iD,iB)  = R.stab.surgeMargin;
    end
    fprintf('扫描进度: D1/D2=%.2f 完成\n', D1D2_v(iD));
end

%% ---- 全组合变速精调 (命中 25 m/s, 记录精调后指标; 可行性在精调后判定) ----
Vt = Vmap; Et = Emap; Pt = Pmap; Mt = Mmap; RPMt = RPMmap;
for iD = 1:nD
    for iB = 1:nB
        if ~isfinite(Vmap(iD,iB)), continue; end
        P = P0;
        P.imp.D1 = D1D2_v(iD)*P.imp.D2;
        P.imp.beta2A = beta2A_v(iB);
        [Vt(iD,iB), Et(iD,iB), Pt(iD,iB), Mt(iD,iB), RPMt(iD,iB)] = ...
            trimRpm(P, 1440, Vmap(iD,iB));
    end
end
feas = isfinite(Vt) & abs(Vt - P0.load.V_test) <= 1.0;

%% ---- 推荐: 精调可行 (命中25) + 稳定裕度>15% + 功率<2.2kW 中效率最高 ----
ok = isfinite(Vt) & abs(Vt-P0.load.V_test) < 0.4 & isfinite(Mt) & Mt > 0.15 ...
     & isfinite(Pt) & Pt < 2200;
score = Et;  score(~ok) = -Inf;
[~, im] = max(score(:));
[iD0, iB0] = ind2sub(size(score), im);

fprintf('\n===== 扫描完成: %d 组合, 精调可行 %d, 满足约束 %d =====\n', nB*nD, nnz(feas), nnz(ok));
if isfinite(score(iD0))
    fprintf('推荐设计点(全域): beta2A=%.0f° | D1/D2=%.2f (叶片长 %.0f mm) | 转速 %.0f rpm\n', ...
        beta2A_v(iB0), D1D2_v(iD0), (1-D1D2_v(iD0))*P0.imp.D2/2*1e3, RPMt(iD0,iB0));
    fprintf('  → 试验段 %.1f m/s | η静 %.1f%% | 轴功率 %.0f W | 喘振裕度 %.0f%%\n', ...
        Vt(iD0,iB0), Et(iD0,iB0)*100, Pt(iD0,iB0), Mt(iD0,iB0)*100);
end
% 前弯家族最优 (beta2A>=120, 参考图的前弯多叶构型)
okF = ok & repmat(beta2A_v, nD, 1) >= 120;
scoreF = Et;  scoreF(~okF) = -Inf;
[~, imF] = max(scoreF(:));
[iDf, iBf] = ind2sub(size(scoreF), imF);
if isfinite(scoreF(imF))
    fprintf('前弯家族最优(beta2A>=120°): beta2A=%.0f° | D1/D2=%.2f (叶片长 %.0f mm) | 转速 %.0f rpm\n', ...
        beta2A_v(iBf), D1D2_v(iDf), (1-D1D2_v(iDf))*P0.imp.D2/2*1e3, RPMt(iDf,iBf));
    fprintf('  → 试验段 %.1f m/s | η静 %.1f%% | 轴功率 %.0f W | 喘振裕度 %.0f%%\n', ...
        Vt(iDf,iBf), Et(iDf,iBf)*100, Pt(iDf,iBf), Mt(iDf,iBf)*100);
end
% Top-8 表 (按效率)
score2 = score; top = zeros(8,6);  r = 0;
for rr = 1:8
    [~, im2] = max(score2(:));
    if ~isfinite(score2(im2)), break; end
    [a, b] = ind2sub(size(score2), im2);
    r = r + 1;
    top(r,:) = [beta2A_v(b), D1D2_v(a), RPMt(a,b), Vt(a,b), Et(a,b)*100, Pt(a,b)];
    score2(im2) = -Inf;
end
fprintf('\n%-8s %-9s %-9s %-8s %-8s %-8s\n', 'beta2A', 'D1/D2', 'rpm', 'V_ts', 'η静%', 'P_W');
for k = 1:r
    fprintf('%-8.0f %-9.2f %-9.0f %-8.1f %-8.1f %-8.0f\n', top(k,:));
end

%% ---- 图谱 (4 联) ----
figure('Color','w','Position',[60 60 1240 780]);
set(groot,'defaultAxesColor','w','defaultTextColor',[.15 .15 .15]);
subplot(2,2,1);
imagesc(beta2A_v, D1D2_v, Vt); set(gca,'YDir','normal'); colorbar; hold on;
contour(beta2A_v, D1D2_v, Vt, [20 25 30], 'w-','LineWidth',1.2);
plot(beta2A_v(iB0), D1D2_v(iD0), 'rp','MarkerSize',14,'MarkerFaceColor','r');
plot(beta2A_v(iBf), D1D2_v(iDf), 'kd','MarkerSize',10,'MarkerFaceColor','k');
plot(P0.imp.beta2A, P0.imp.D1/P0.imp.D2, 'wo','MarkerSize',12,'LineWidth',2);
caxis([15 35]); colormap(parula);
legend({'','设计风速等值线','全域最优(后弯)','前弯家族最优','当前默认(130°)'}, 'Location','best');
xlabel('\beta_{2A} (deg)'); ylabel('D_1/D_2'); title('试验段风速 V_{ts} (m/s), 转速精调后');
subplot(2,2,2);
imagesc(beta2A_v, D1D2_v, Et*100); set(gca,'YDir','normal'); colorbar; hold on;
plot(beta2A_v(iB0), D1D2_v(iD0), 'rp','MarkerSize',14,'MarkerFaceColor','r');
plot(beta2A_v(iBf), D1D2_v(iDf), 'kd','MarkerSize',10,'MarkerFaceColor','k');
plot(P0.imp.beta2A, P0.imp.D1/P0.imp.D2, 'wo','MarkerSize',12,'LineWidth',2);
xlabel('\beta_{2A} (deg)'); ylabel('D_1/D_2'); title('静压效率 \eta_{sf} (%) @ 25 m/s');
subplot(2,2,3);
imagesc(beta2A_v, D1D2_v, Pt); set(gca,'YDir','normal'); colorbar; hold on;
contour(beta2A_v, D1D2_v, Pt, [2200 2200], 'r-','LineWidth',2);
plot(beta2A_v(iB0), D1D2_v(iD0), 'rp','MarkerSize',14,'MarkerFaceColor','r');
plot(beta2A_v(iBf), D1D2_v(iDf), 'kd','MarkerSize',10,'MarkerFaceColor','k');
plot(P0.imp.beta2A, P0.imp.D1/P0.imp.D2, 'wo','MarkerSize',12,'LineWidth',2);
xlabel('\beta_{2A} (deg)'); ylabel('D_1/D_2'); title('轴功率 (W), 红圈=2.2kW 预算');
subplot(2,2,4);
imagesc(beta2A_v, D1D2_v, Mt*100); set(gca,'YDir','normal'); colorbar; hold on;
contour(beta2A_v, D1D2_v, Mt*100, [15 15], 'r-','LineWidth',2);
plot(beta2A_v(iB0), D1D2_v(iD0), 'rp','MarkerSize',14,'MarkerFaceColor','r');
plot(beta2A_v(iBf), D1D2_v(iDf), 'kd','MarkerSize',10,'MarkerFaceColor','k');
plot(P0.imp.beta2A, P0.imp.D1/P0.imp.D2, 'wo','MarkerSize',12,'LineWidth',2);
xlabel('\beta_{2A} (deg)'); ylabel('D_1/D_2'); title('喘振流量裕度 (%), 红圈=15% 下限');
sgtitle(sprintf('抽风式风机参数扫描 | D2=%.0fmm Z=%d b2=%.0fmm | 目标试验段 %.0f m/s', ...
    P0.imp.D2*1e3, P0.imp.Z, P0.imp.b2*1e3, P0.load.V_test));
saveas(gcf, fullfile(pwd,'fan_sweep.png'));
fprintf('fan_sweep.png 已保存\n');

save(fullfile(pwd,'fan_sweep_result.mat'), 'beta2A_v','D1D2_v','Vt','Et','Pt','Mt','RPMt','top','P0');
fprintf('fan_sweep_result.mat 已保存\n');

%% ==================== 局部函数 ====================
function [Vts, eta, Pw, sm, rpm] = trimRpm(P, rpm0, V0)
% 二变转速使试验段风速精确命中 25 m/s (已知 rpm0=1440 的偏向, 只在对应半区间找)
target = P.load.V_test;
if isfinite(V0) && V0 < target, lo = rpm0; hi = 2900; else, lo = 500; hi = 1440; end
Vts = NaN;
try
    for it = 1:14
        rpm = (lo+hi)/2;
        R = fan_performance(P, [], rpm, [], true);
        if ~isfinite(R.op.Q), lo = rpm; continue; end
        Vts = R.op.Vts;
        if Vts > target, hi = rpm; else, lo = rpm; end
        if abs(Vts-target) < 0.02, break; end
    end
    R = fan_performance(P, [], rpm, [], true);
end
if exist('R','var') && isfinite(Vts) && isfinite(R.op.Q)
    Vts = R.op.Vts;  eta = R.op.etaS;  Pw = R.op.P;  sm = R.stab.surgeMargin;
else
    Vts = NaN;  eta = NaN;  Pw = NaN;  sm = NaN;
end
end
