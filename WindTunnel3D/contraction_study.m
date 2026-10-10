%% contraction_study — 收缩段角度/长度对风场稳定性的参数扫描
% 指标(经验+一维势流):
%   1) 入口端峰值壁面角 theta_max: > 45° 易在收缩入口产生分离
%   2) 出口端壁面角 theta_exit: 应 < 1°, 否则试验段入口速度不均
%   3) 出口非均匀性估计 (Downie et al. 经验式): 蜂窝器+纱网+收缩的残余湍流
%   4) 收缩加速对湍流的抑制: u'/U 收缩前后比 = 1/R (轴向分量被拉伸抑制)
clear; clc; close all;
P = tunnel_params();

%% ---- 扫描: 收缩长度 L x 收缩比 R ----
Ls = (0.15:0.025:0.50);        % 收缩段长度 (m)
Rs = (4:0.5:9);                % 收缩比
[LL, RR] = meshgrid(Ls, Rs);

thetaMax  = zeros(size(LL));   % 入口峰值壁面角 (deg) — 五次匹配型线
thetaExit = zeros(size(LL));   % 出口壁面角 (deg)
turbRed   = zeros(size(LL));   % 湍流抑制比 u'/U 收缩后/前 (轴向)
sepMargin = zeros(size(LL));   % 分离裕度: 45° - thetaMax

for i = 1:numel(LL)
    s = linspace(0,1,200)';
    % 物理壁面斜率: dh/dx = (h_in - h_out)*(型线导数)/L, h = h_out*(√R - 1) 为半高差
    dhds = (sqrt(RR(i))-1)*(30*s.^2 - 60*s.^3 + 30*s.^4)/LL(i) * P.ts.W/2;
    th = atand(dhds);
    thetaMax(i)  = max(th);
    thetaExit(i) = th(end) + 1e-6;
    turbRed(i)   = 1/RR(i);    % 轴向脉动按 1/R 抑制 (Batchelor 轴向拉伸)
end

%% ---- 推荐设计点 ----
% 约束: thetaMax <= 35° (分离裕度); 残余湍流比 1/R 越小越好 -> 取最小
ok = thetaMax <= 35;
t = turbRed;  t(~ok) = Inf;
[~, lin] = min(t(:));
fprintf('推荐: 收缩长度 L = %.0f mm, 收缩比 R = %.1f:1  (峰值壁面角 %.1f°, 残余轴向湍流比 %.0f%%)\n', ...
    LL(lin)*1000, RR(lin), thetaMax(lin), turbRed(lin)*100);
fprintf('当前设计: L = %.0f mm, R = %.1f:1\n', P.ct.L*1000, P.ct.R);

%% ---- 可视化 ----
figure('Color','w','Position',[60 60 1150 780]);

subplot(2,2,1);
[cm] = contourf(LL*1000, RR, thetaMax, 12, 'LineColor','none'); colormap(parula); colorbar;
hold on; contour(LL*1000, RR, thetaMax, [35 35], 'r-', 'LineWidth', 2);
plot(P.ct.L*1000, P.ct.R, 'w+', 'MarkerSize', 12, 'LineWidth', 2);
xlabel('收缩段长度 (mm)'); ylabel('收缩比 R'); title('入口峰值壁面角 (°), 红线=35°限制');

subplot(2,2,2);
contourf(LL*1000, RR, turbRed, 12, 'LineColor','none'); colorbar;
xlabel('收缩段长度 (mm)'); ylabel('收缩比 R'); title('残余轴向湍流比 (越小越好)');

subplot(2,2,3);
% 典型型线对比
s = linspace(0,1,200)';
h5 = @(R) (1 - (1-1/sqrt(R))*(10*s.^3-15*s.^4+6*s.^5));
plot(s*300, h5(6), 'b-', 'LineWidth', 1.6); hold on;
plot(s*300, 1-(1-1/sqrt(6))*(3*s.^2-2*s.^3), 'g--', 'LineWidth', 1.3);
plot(s*300, 1-s.*(1-1/sqrt(6)), 'k:', 'LineWidth', 1.3);
legend('五次匹配 (推荐)', '三次匹配', '直线收缩', 'Location', 'south');
grid on; xlabel('x (mm)'); ylabel('h/h_{in}');
title('收缩段壁面型线对比 (R=6, L=300mm)');

subplot(2,2,4);
% 沿程平均速度 (一维连续性, 方形截面): U = U_in/(h/h_in)^2, U_in = U_test/R
x = s*300;
for R = [4 6 9]
    h = h5(R);
    U = (P.V_test/R) ./ h.^2;
    plot(x, U, 'LineWidth', 1.4, 'DisplayName', sprintf('R=%d',R));
end
legend show; grid on; xlabel('x (mm)'); ylabel('U (m/s)'); ylim([0 P.V_test*1.15]);
title('收缩段沿程平均速度 (一维)');
