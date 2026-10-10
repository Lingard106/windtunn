%% boundary_layer_study — 腔体长度对风场稳定性与摩擦分层的影响
% "摩擦分层": 试验段壁面边界层生长导致核心区与近壁区速度差随轴向增大,
% 以及边界层排挤厚度引起的核心区加速 (blockage effect)。
% 方法: 湍流平板边界层增长 (压差沿程近似为0, 试验段内成立)
clear; clc; close all;
P = tunnel_params();

U  = P.V_test;
nu = P.nu;
Re_x = @(x) U*x/nu;

% ---- 湍流边界层厚度: delta = 0.37 x / Re_x^{1/5} ----
delta   = @(x) 0.37*x ./ Re_x(x).^0.2;                 % 边界层厚度
cf_fun  = @(x) 0.0592 * Re_x(x).^(-0.25);              % 局部摩擦系数
delta_s = @(x) delta(x)/8;                             % 排挤厚度 (1/7 幂律)
theta_m = @(x) 7*delta(x)/72;                          % 动量厚度

%% ---- 关键位置边界层状态 ----
fprintf('===== 试验段边界层发展 (U = %.0f m/s) =====\n', U);
x_chk = [0, P.ts.L*0.25, P.ts.L*0.5, P.ts.L];          % 收缩段出口起算
% 注意: 收缩段强顺压梯度大幅抑制边界层, 保守取收缩出口初始厚度 ~2mm
d0 = 0.002;
fprintf('  站位(mm)   delta(mm)   delta_s(mm)   cf      核心区U修正\n');
x_eff = x_chk + 0.05;                                  % 折算平板当量长度(保守)
for i = 1:numel(x_chk)
    d = sqrt(d0^2 + delta(x_eff(i))^2);                % 初值与增长平方合成(保守)
    ds = d/8;  cf = cf_fun(x_eff(i));
    Uc = U / (1 - 4*ds/P.ts.W);                        % 排挤引起的核心加速
    fprintf('  %-8.0f  %-8.2f    %-8.3f     %.4f  %.1f%%\n', ...
        x_chk(i)*1000, d*1000, ds*1000, cf, (Uc/U-1)*100);
end

%% ---- 腔体(试验段)长度扫描: 分层程度评估 ----
Ls = (0.2:0.05:1.0)';                                  % 试验段长度扫描 (m)
Xg = linspace(0, 1, 200)';                             % 归一化站位
d_end  = zeros(size(Ls));
Ucore  = zeros(size(Ls));                              % 出口核心区风速
dU_wall = zeros(size(Ls));                             % 壁面与核心速度差
for i = 1:numel(Ls)
    x = Xg*Ls(i) + 0.05;
    d = sqrt(d0^2 + delta(x).^2);
    d_end(i) = d(end);
    Ucore(i) = U / (1 - 4*(d_end(i)/8)/P.ts.W);
    % 1/7 幕律: 壁面处 u=0, 边界层外缘 u=U_core -> 速度分层幅度
    dU_wall(i) = Ucore(i);
end

% 均匀区判定: 核心区中 |y| < W/2 - delta 的范围
fprintf('\n===== 腔体长度对可用均匀流场的影响 =====\n');
fprintf('  L(mm)   delta_end(mm)   均匀区宽度(mm)   核心加速%%   适合测量的模型直径(mm)\n');
for i = 1:numel(Ls)
    Wu = P.ts.W - 2*d_end(i);
    fprintf('  %-6.0f  %-10.2f     %-10.1f      %5.2f      %.1f\n', ...
        Ls(i)*1000, d_end(i)*1000, Wu*1000, (Ucore(i)/U-1)*100, Wu*0.6*1000);
end

%% ---- 可视化 ----
figure('Color','w','Position',[60 60 1150 760]);

subplot(2,2,1);
x = linspace(0, P.ts.L, 100)';
d = sqrt(d0^2 + delta(x+0.05).^2);
yy = linspace(-P.ts.W/2, P.ts.W/2, 200)';
% 速度剖面云图 (1/7 幂律)
Ufield = repmat(1, numel(x), numel(yy));
for k = 1:numel(x)
    u_prof = min(max(1 - abs(yy)/max(d(k),eps), 0), 1).^(1/7) * U;
    Ufield(k,:) = u_prof';
end
contourf(x*1000, yy*1000, Ufield', 20, 'LineColor','none'); colormap(jet); colorbar;
xlabel('试验段 x (mm)'); ylabel('y (mm)'); title('速度剖面发展 (摩擦分层)');

subplot(2,2,2);
plot(x*1000, d*1000, 'b-', 'LineWidth', 1.6); grid on;
xlabel('试验段 x (mm)'); ylabel('\delta (mm)'); title('边界层厚度增长');

subplot(2,2,3);
plot(Ls*1000, (Ucore/U-1)*100, 'r-o'); grid on;
xlabel('试验段长度 (mm)'); ylabel('核心区加速 (%)'); title('堵塞引起的核心区加速');

subplot(2,2,4);
plot(Ls*1000, (P.ts.W - 2*d_end)*1000, 'b-o'); grid on; hold on;
yline(P.ts.W*1000, 'k--', '壁面');
xlabel('试验段长度 (mm)'); ylabel('均匀区宽度 (mm)'); title('可用均匀流场宽度');

%% ---- 壁面摩擦阻力 (评估驱动功率) ----
x = linspace(0.001, P.ts.L, 100);
tau_w = cf_fun(x) * 0.5*P.rho*U^2;
F_fric = 4 * mean(tau_w) * (P.ts.W * P.ts.L);          % 四壁
fprintf('\n试验段四壁摩擦阻力: %.2f N (单位展长深度)\n', F_fric);

% ---- PLA 粗糙壁对比 (与 flow3d_solve 同源公式) ----
ks = P.wall.ks;  xe = P.ts.L + 0.08;  ReE = U*xe/nu;
cfS = 0.0592*ReE^(-0.25);
cfR = (1.89 + 1.62*log10(max(xe/ks, 30)))^(-2.5);
fprintf(['PLA 粗糙壁 (ks=%.2fmm, Re_ks=%.0f>5 全粗糙): 末端 cf=%.4f vs 光滑壁 %.4f ' ...
    '(+%.0f%%), 转捩提前至入口, 边界层全程湍流\n'], ...
    ks*1e3, ks*U*sqrt(cfS/2)/nu, cfR, cfS, (cfR/cfS-1)*100);
idxL = find(abs(Ls - P.ts.L) < 1e-9, 1);
fprintf('提示: 核心加速由边界层排挤引起, 设计长度 %.0f mm 下壁面加 ~%.1f° 微扩张角可补偿 (见 tunnel_params ts.growth)\n', ...
    P.ts.L*1000, atand((Ucore(idxL)/U-1)/2*P.ts.W/2/P.ts.L));

%% ---- 扩散段逆压梯度与边界层分离分析 (参考: 逆压→剖面 reversed→分离点) ----
% 判据: R = (dp/dx·δ*)/τw — 壁面摩擦对抗逆压的能力比; R→1 预示分离
G = tunnel_geometry(P);
xd = linspace(0, P.df.L, 60)';
aD = interp1(G.x, G.a, G.xSec(4)+xd, 'pchip', 'extrap');
bD = interp1(G.x, G.b, G.xSec(4)+xd, 'pchip', 'extrap');
AD = (2*aD).*(2*bD);  ATs = P.ts.W*P.ts.H;
Ud = P.V_test*ATs./AD;                             % 一维核心速度 (沿程减速)
dUd = gradient(Ud, xd);
dpdx = P.rho*Ud.*(-dUd);                           % 逆压梯度 (Pa/m, >0=逆压)
xeD = P.ts.L + 0.08 + xd;
ReD = Ud.*xeD/nu;
cfD = 0.0592*max(ReD,1).^(-0.25);
twD = 0.5*P.rho*Ud.^2.*cfD;
dsD = 0.046*xeD./max(ReD,1).^0.2;                % 湍流排挤厚度 δ* (含扩散段逆压增厚)
Rsep = (dpdx.*dsD)./max(twD, 1e-6);
sepX = xd(Rsep >= 1);
if isempty(sepX), sMsg = '无分离 (全段 R<1, 摩擦可克服逆压)'; else, sMsg = '扩散段末端进入分离风险区 (定性)'; end
fprintf('\n扩散段逆压分析: dp/dx_max=%.0f Pa/m | 分离判据 R_max=%.2f → %s\n', max(dpdx), max(Rsep), sMsg);

% 假想陡扩散对比 (长度减半 → 扩张角翻倍) 演示"大逆压→分离"
xd2 = xd*0.5;  Lh = P.df.L/2;
A2  = ATs + (AD(end)-ATs)*min(xd2/max(Lh,eps), 1);
Ud2 = P.V_test*ATs./max(A2, eps);
dUd2 = gradient(Ud2, xd2);
dp2 = P.rho*Ud2.*(-dUd2);
Re2 = Ud2.*(xeD(1)+xd2)/nu;
tw2 = 0.5*P.rho*Ud2.^2.*(0.0592*max(Re2,1).^(-0.25));
ds2 = 0.046*(xeD(1)+xd2)./max(Re2,1).^0.2/8;
R2 = (dp2.*ds2)./max(tw2,1e-6);

% --- 耙图: 扩散段轮廓 + 各站速度剖面 (参考图形式: 剖面水平展开=速度大小) ---
figure('Color','w','Position',[80 80 1150 520]);
subplot(1,2,1); hold on;
Xp = [G.xSec(4); G.xSec(4)+xd; G.xSec(4)+xd(end); G.xSec(4)+xd(end:-1:1); G.xSec(4)];
Yp = [bD(1);   bD;   bD(end);   -bD(end:-1:1);   -bD(1)];
fill(Xp, Yp, [0.82 0.9 0.97], 'EdgeColor','none');
for st = round(linspace(3, numel(xd)-2, 5))
    yyP = linspace(-bD(st), bD(st), 41);
    uP = Ud(st)*min(max((bD(st)-abs(yyP))/max(8*dsD(st), 1e-4), 0), 1).^(1/7);
    uP = min(uP, Ud(st));
    xAbs = G.xSec(4) + xd(st);
    plot(xAbs + uP*0.004, yyP, 'b-', 'LineWidth', 1.2);   % 剖面水平展开 (速度×比例)
    plot(xAbs*[1 1], [-bD(st) bD(st)], 'k:', 'LineWidth', 0.6);
end
plot([G.xSec(4), G.xSec(4)+xd(end)], [bD(1), bD(end)], 'k-', 'LineWidth', 1.4);
plot([G.xSec(4), G.xSec(4)+xd(end)], [-bD(1), -bD(end)], 'k-', 'LineWidth', 1.4);
xlabel('x (m)'); ylabel('y (m)');
title(sprintf('扩散段速度剖面发展 (当前 %.0fmm: %s)', P.df.L*1000, sMsg));
axis equal;

subplot(1,2,2); hold on;
plot(xd*1000, Rsep, 'b-o', 'LineWidth', 1.4, 'MarkerSize', 4);
plot(xd2*1000, R2, 'r--s', 'LineWidth', 1.2, 'MarkerSize', 4);
yline(1, 'k-', 'R=1 分离阈值');
grid on; xlabel('扩散段内 x (mm)'); ylabel('逆压/摩擦比 R');
legend(sprintf('当前设计 %.0fmm (6.8°)', P.df.L*1000), '陡扩散对比 (减半)', 'Location','best');
title('逆压梯度 vs 壁面摩擦 — 分离风险判据');
% 涡脱参考
fprintf('扩散段分离判据: R_max=%.2f (当前) / %.2f (陡扩散对比) — R≥1 分离\n', max(Rsep), max(R2));

% 导出分析图
fs = findall(0,'Type','figure');
for q = 1:numel(fs)
    try
        print(fs(q), '-dpng', '-r85', sprintf('boundary_layer_fig%d.png', fs(q).Number));
    catch
    end
end
