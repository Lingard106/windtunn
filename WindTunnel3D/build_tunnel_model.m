%% build_tunnel_model — 参数化风洞 3D 建模 + STL 导出 + 侧视/角度图
% 流路: 稳定段(蜂窝器+纱网) -> 收缩段 -> 试验段 -> 扩散段
% 构型: tunnel_params('balanced'|'lowTurb'|'compact'|'aniso'), 各段截面独立可调
% 输出: tunnel_body.stl (流向为 STL +Z 轴, 可导入任意 CAD 与 STEP 模型装配)
clear; clc; close all;
set(groot,'defaultFigureColor','w','defaultAxesColor','w', ...
    'defaultAxesXColor',[.15 .15 .15],'defaultAxesYColor',[.15 .15 .15], ...
    'defaultAxesZColor',[.15 .15 .15],'defaultTextColor',[.15 .15 .15]);

config = 'balanced';
if ~isempty(getenv('WT_CONFIG')), config = getenv('WT_CONFIG'); end
P = tunnel_params(config);
G = tunnel_geometry(P);

% ---- 设计检查 ----
fprintf('[%s 构型] 总长 %.3f m (含前置扩散段 %.0fmm) | 风机法兰 %.0fx%.0f mm | 收缩比 %.1f:1 (%s型线)\n', ...
    P.config, G.x(end), P.fd.L*1000, P.fd.W_in*1000, P.fd.H_in*1000, P.ct.R, P.ct.curve);
fprintf('试验段 %.0fx%.0f mm | 收缩段峰值壁面角 %.1f° | 扩散段出口 %.0fx%.0f mm, 等效锥角 %.1f°\n', ...
    P.ts.W*1000, P.ts.H*1000, G.theta_ct_max, P.df.W_out*1000, P.df.H_out*1000, G.theta_df_eq);
if G.theta_df_eq > 7
    fprintf('警告: 扩散段等效锥角 > 7°, 有分离风险, 建议加长扩散段或减小出口面积\n');
end
fprintf('前置扩散段: 面积比 %.1f:1, 等效锥角 %.1f° (宽角扩散, 整流网:%s) — 法兰风速 %.1f m/s → 稳定段 %.1f m/s\n', ...
    G.AR_fd, G.theta_fd_eq, tern(P.fd.screen,'带整流网','无网'), G.U(1), G.U(find(G.x>=P.fd.L,1)));
fprintf('各段风速: 法兰 %.1f | 稳定段 %.1f | 试验段 %.1f | 扩散段出口 %.1f m/s | Re_ts=%.2e\n', ...
    G.U(1), G.U(find(G.x>=P.fd.L,1)), P.V_test, G.U(end), P.V_test*P.ts.L/P.nu);

%% ---- STL: 矩形截面 (a,b 独立) 沿流向扫掠 ----
nSide = 5; prof = squareProfile(nSide);
nProf = size(prof, 1);
pts = []; tris = [];
for k = 1:numel(G.x)-1
    a1=G.a(k); b1=G.b(k); a2=G.a(k+1); b2=G.b(k+1);
    for e = 1:nProf
        e2 = mod(e, nProf) + 1;
        p1 = [a1*prof(e,1),  b1*prof(e,2),  G.x(k)];
        p2 = [a1*prof(e2,1), b1*prof(e2,2), G.x(k)];
        p3 = [a2*prof(e2,1), b2*prof(e2,2), G.x(k+1)];
        p4 = [a2*prof(e,1),  b2*prof(e,2),  G.x(k+1)];
        pts = [pts; p1; p2; p3; p1; p3; p4]; %#ok<AGROW>
        n = size(pts, 1);
        tris = [tris; n-5, n-4, n-3; n-2, n-1, n]; %#ok<AGROW>
    end
end
writeAsciiSTL(pts, tris, fullfile(pwd, 'tunnel_body.stl'));
fprintf('已导出 tunnel_body.stl (%d 三角形)\n', size(tris,1));

%% ---- 图 1: 双视图侧视轮廓 (宽向 a 与高向 b) ----
figure('Color','w','Position',[40 40 1150 640]);
for vv = 1:2
    subplot(2,1,vv); hold on;
    if vv==1, hb = G.a; lab='半宽 a (m)'; else, hb = G.b; lab='半高 b (m)'; end
    fill([G.x, flipud(G.x)], [hb, -flipud(hb)], [0.85 0.9 0.95], 'EdgeColor','none');
    plot(G.x, hb, 'b-', G.x, -hb, 'b-', 'LineWidth', 1.2);
    plot([G.x(1) G.x(end)], [0 0], 'k:');
    if vv == 1
        xFd = P.fd.L;
        y_hn = max(G.a)*0.92;
        fill([0 xFd xFd 0], [y_hn*0.55 y_hn*0.55 -y_hn*0.55 -y_hn*0.55], [0.95 0.85 0.65], ...
            'EdgeColor','none', 'FaceAlpha',0.5);                                % 前置扩散段
        text(xFd/2, y_hn*0.8, '前置扩散段', 'HorizontalAlignment','center');
        rectangle('Position',[xFd+0.02 -y_hn 0.05 2*y_hn], 'FaceColor',[1 .8 .3]);   % 蜂窝器
        for i = 1:P.sc.nScreen
            xx = xFd + 0.11 + (i-1)*0.04;
            plot([xx xx], [-y_hn y_hn], 'g-', 'LineWidth', 2);                 % 纱网
        end
        if P.fd.screen
            plot([xFd xFd], [-y_hn y_hn], 'g-', 'LineWidth', 2);               % 扩散段出口整流网
        end
        text(xFd+0.07, y_hn+0.01, '蜂窝器'); text(xFd+0.16, y_hn+0.01, '纱网');
    end
    for k = 2:4, xline(G.xSec(k), 'k:'); end
    axis equal; ylim([-max(hb)-0.02, max(hb)+0.05]);
    ylabel(lab);
end
subplot(2,1,1); title(sprintf('[%s] 桌面风洞侧视轮廓 | 收缩段 L=%.0fmm R=%.1f:1 %s型线', ...
    P.config, P.ct.L*1000, P.ct.R, P.ct.curve));
subplot(2,1,2);
xlabel('x (m)');

%% ---- 图 2: 壁面倾角沿程分布 (双方向) ----
figure('Color','w','Position',[80 80 900 380]); hold on;
plot(G.x, G.thetaB, 'r-', 'LineWidth', 1.4);
plot(G.x, G.thetaA, 'b--', 'LineWidth', 1.2);
yline(35, 'k--', '35° 分离限制'); yline(-35, 'k--');
for k = 2:4, xline(G.xSec(k), 'k:'); end
for k = 1:4
    text(mean(G.xSec([k k+1])), max(abs(G.thetaB))*0.95, G.secNames{k}, ...
        'HorizontalAlignment','center', 'FontWeight','bold');
end
grid on; xlabel('x (m)'); ylabel('壁面倾角 (deg)');
legend('顶/底壁 (b)', '侧壁 (a)', 'Location','southeast');
title('壁面倾角沿程分布 (收缩段 < 35°, 扩散段 < 7°)');

%% ==================== 局部函数 ====================
function s = tern(c, a, b)
if c, s = a; else, s = b; end
end

function prof = squareProfile(nSide)
    edges = [linspace(-1,1,nSide)' linspace(-1,-1,nSide)' ; ...
             linspace(1,1,nSide)'   linspace(-1,1,nSide)' ; ...
             linspace(1,-1,nSide)'  linspace(1,1,nSide)' ; ...
             linspace(-1,-1,nSide)' linspace(1,-1,nSide)'];
    prof = unique(edges, 'rows', 'stable');
    prof = prof(1:end-1, :);
end

function writeAsciiSTL(pts, tris, fname)
    fid = fopen(fname, 'w');
    fprintf(fid, 'solid tunnel\n');
    for t = 1:size(tris,1)
        p = pts(tris(t,:),:);
        n = cross(p(2,:)-p(1,:), p(3,:)-p(1,:));
        nn = norm(n); if nn == 0, nn = 1; end
        fprintf(fid, '  facet normal %.6f %.6f %.6f\n  outer loop\n', n(1)/nn, n(2)/nn, n(3)/nn);
        fprintf(fid, '    vertex %.6f %.6f %.6f\n', p(1,1), p(1,2), p(1,3));
        fprintf(fid, '    vertex %.6f %.6f %.6f\n', p(2,1), p(2,2), p(2,3));
        fprintf(fid, '    vertex %.6f %.6f %.6f\n', p(3,1), p(3,2), p(3,3));
        fprintf(fid, '  endloop\n  endfacet\n');
    end
    fprintf(fid, 'endsolid tunnel\n');
    fclose(fid);
end
