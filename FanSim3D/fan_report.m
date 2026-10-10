%% fan_report — 抽风式风机一页设计报告 (1D 性能 + 损失分解 + 扫描图谱 + GPU 时程)
% 输出: fan_report.png + 控制台结论
% 面板原则: 仅看图即可获得信息 — 每格自含单位/参考线/结论
% 若有 GPU 结果 (fan_diag.csv / vtk) 自动纳入; 否则仅 1D 部分
clear; clc; close all;
set(groot,'defaultFigureColor','w','defaultAxesColor','w', ...
    'defaultAxesXColor',[.15 .15 .15],'defaultAxesYColor',[.15 .15 .15], ...
    'defaultAxesZColor',[.15 .15 .15],'defaultTextColor',[.15 .15 .15]);
P = fan_params();
G = fan_geometry(P);
R = G.R;
L = R.load;
hasGPU = isfile(fullfile('D:\FanSim3D','fluidx3d','fan_diag.csv'));

fprintf('==== FanSim3D 设计报告 ====\n');
fprintf('风机: D2=%.0fmm D1=%.0fmm Z=%d beta2A=%.0f° b2=%.0fmm n=%.0frpm | 叶片长 %.0fmm\n', ...
    P.imp.D2*1e3, P.imp.D1*1e3, P.imp.Z, R.tri.beta2A, P.imp.b2*1e3, P.imp.rpm, P.imp.bladeLen*1e3);
fprintf('1D 工况点: Q=%.2f m³/s → 试验段 %.1f m/s | p_sf=%.0f Pa | 轴功率 %.0f W | η静=%.0f%% | 喘振裕度 %.0f%%\n', ...
    R.op.Q, R.op.Vts, R.op.p, R.op.P, R.op.etaS*100, R.stab.surgeMargin*100);
fprintf('蜗壳: c_th=%.1f m/s rEnd=%.0fmm | 出口法兰 %.0fx%.0f mm (= WindTunnel3D 前置扩散段入口)\n', ...
    G.vol.c_th, G.vol.rEnd*1e3, P.fd.W_in*1e3, P.fd.H_in*1e3);
fprintf('风洞阻力: K=%.0f Pa/(m³/s)² | 设计点压降 %.0f Pa\n', L.K, L.dp(R.op.Q));

figure('Color','w','Position',[30 30 1560 1080]);

% ---- (1) p-Q 曲线 + 工况点 ----
subplot(3,2,1); hold on; grid on;
plot(R.Q, R.p_sf, 'b-', 'LineWidth', 1.6);
plot(R.Q, L.dp(R.Q), 'r--', 'LineWidth', 1.4);
plot(R.op.Q, R.op.p, 'ko', 'MarkerFaceColor', 'y', 'MarkerSize', 8);
plot(R.Q, R.p_sf*0+R.stab.peakPs, 'k:', 'LineWidth', 0.8);
xlabel('Q (m³/s)'); ylabel('\Delta p (Pa)');
legend({'风机静压 p_{sf}', '风洞阻力 K·Q²', '工况点', '喘振边界(峰压)'}, 'Location', 'southwest');
title(sprintf('(1) 性能匹配 | η静=%.0f%% 喘振裕度=%.0f%%', R.op.etaS*100, R.stab.surgeMargin*100));

% ---- (2) 系统损失分解 (设计风量下各部件吃掉多少 Pa) ----
subplot(3,2,2); hold on; grid on;
nC = size(L.comp,1);
dpC = zeros(nC,1);
for i = 1:nC
    dpC(i) = L.comp{i,3}*(L.rho/2)*(R.op.Q/L.comp{i,2})^2;
end
[dpS, iS] = sort(dpC, 'ascend');
barh(dpS, 'FaceColor', [0.25 0.5 0.8]);
yticks(1:nC);  yticklabels({L.comp{iS,1}});
for i = 1:nC
    text(dpS(i)+max(dpS)*0.02, i, sprintf('%.0f', dpS(i)), 'FontSize', 8, 'VerticalAlignment','middle');
end
xlabel('\Delta p @ 设计风量 (Pa)');
title(sprintf('(2) 设计点系统损失分解 | 合计 %.0f Pa (排空动能+前置扩散段为大头)', sum(dpC)));
xlim([0, max(dpS)*1.18]);

% ---- (3) 扫描图谱 ----
subplot(3,2,3);
if isfile(fullfile(pwd,'fan_sweep_result.mat'))
    S = load(fullfile(pwd,'fan_sweep_result.mat'));
    imagesc(S.beta2A_v, S.D1D2_v, S.Et*100); set(gca,'YDir','normal'); colorbar; hold on;
    contour(S.beta2A_v, S.D1D2_v, S.Pt, [2200 2200], 'r-', 'LineWidth', 1.5);
    plot(R.tri.beta2A, P.imp.D1/P.imp.D2, 'rp', 'MarkerSize', 13, 'MarkerFaceColor', 'r');
    xlabel('\beta_{2A} (deg)'); ylabel('D_1/D_2');
    title('(3) 扫描: η静% (红圈=2.2kW 功率线, ★=当前设计)');
else
    text(0.5, 0.5, '先跑 fan_sweep', 'HorizontalAlignment', 'center');
end

% ---- (4) 出口速度三角形 ----
subplot(3,2,4); hold on; axis equal; grid on;
tri = R.tri;
quiver(0, 0, tri.u2, 0, 0, 'b', 'LineWidth', 1.8, 'MaxHeadSize', 0.5);
quiver(0, 0, tri.c2u, tri.c2m, 0, 'r', 'LineWidth', 1.8, 'MaxHeadSize', 0.5);
quiver(tri.c2u, tri.c2m, tri.u2-tri.c2u, -tri.c2m, 0, 'g', 'LineWidth', 1.4, 'MaxHeadSize', 0.5);
text(tri.u2*1.02, 0.1, sprintf('u_2=%.1f', tri.u2), 'Color','b');
text(tri.c2u*0.5, tri.c2m*1.12, sprintf('c_2: |c|=%.1f m/s, 气流角 %.0f°', ...
    hypot(tri.c2u,tri.c2m), atand(tri.c2m/tri.c2u)), 'Color','r');
legend({'u_2 (叶尖)', 'c_2 (绝对)', 'w_2 (相对)'}, 'Location', 'southwest');
title(sprintf('(4) 出口速度三角形 \\beta_{2A}=%.0f° (前弯: c_{2u}>u_2)', tri.beta2A));
xlabel('切向 (m/s)'); ylabel('径向 (m/s)');
axis([-0.15, 1.25*max(tri.u2,tri.c2u), -0.08, 1.35*max(tri.c2m,0.5)]);

% ---- (5)(6) GPU 时程 ----
if hasGPU
    T = readmatrix(fullfile('D:\FanSim3D','fluidx3d','fan_diag.csv'));
    tD = T(:,1);  QD = T(:,2);  TQ = T(:,3);
    iA = tD >= meta_avgStart();
    wPhys = 2*pi*P.imp.rpm/60;
    subplot(3,2,5); hold on; grid on;
    plot(tD, QD, 'b-', 'LineWidth', 1.1);
    plot([tD(1) tD(end)], [R.op.Q R.op.Q], 'r--', 'LineWidth', 1.2);
    xlabel('t (步)'); ylabel('Q (m³/s)');
    title(sprintf('(5) GPU 流量 | 末段 %.2f m³/s (1D: %.2f — 粗网格叶片保真度限制, 见 README 已知边界)', ...
        mean(QD(iA),'omitnan'), R.op.Q));
    subplot(3,2,6); hold on; grid on;
    yyaxis left;  plot(tD, -TQ, 'r-', 'LineWidth', 1.0);
    ylabel('轴转矩 (N·m)');
    yyaxis right; plot(tD, -TQ*wPhys, 'm-', 'LineWidth', 1.0);
    plot([tD(1) tD(end)], [R.op.P R.op.P], 'r--', 'LineWidth', 1.2);
    ylabel('轴功率 (W)');
    title(sprintf('(6) GPU 转矩/功率 | 末段 %.0f N·m, %.0f W (1D: %.0f W)', ...
        mean(-TQ(iA),'omitnan'), mean(-TQ(iA),'omitnan')*wPhys, R.op.P));
    xlabel('t (步)');
else
    subplot(3,2,[5 6]);
    text(0.5, 0.5, {'GPU 结果未生成', 'export → build_fan.bat → windfan.exe'}, ...
        'HorizontalAlignment', 'center');
end
sgtitle(sprintf('FanSim3D 抽风式风机 | D_2=%.0fmm Z=%d \\beta_{2A}=%.0f° n=%.0frpm → 试验段 %.1f m/s', ...
    P.imp.D2*1e3, P.imp.Z, R.tri.beta2A, P.imp.rpm, R.op.Vts), 'FontWeight','bold');
saveas(gcf, fullfile(pwd, 'fan_report.png'));
fprintf('fan_report.png 已保存\n');

    function s = meta_avgStart()
        M = load(fullfile('D:\FanSim3D','fan_meta.mat'));
        s = M.meta.nAvgStart;
    end
