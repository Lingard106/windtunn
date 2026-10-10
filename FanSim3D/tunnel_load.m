function L = tunnel_load(P, Qs, quiet)
% tunnel_load — 风洞系统阻力曲线 (从风机出口法兰到风洞排空, 含前置扩散段)
% Δp_sys(Q) = K·Q², 各部件损失 ζ_i·(ρ/2)·(Q/A_i)² 求和 → K = Σ ζ_i·ρ/(2·A_i²)
% 风洞侧参数取自 WindTunnel3D/tunnel_params (单一真源); 风机侧弯头取自 fan_params
% L = tunnel_load(P) 返回结构: K, dpFun, 分解表 breakdown, 以及设计点诊断
%   tunnel_load(P, Qs, true) 静音 (fan_performance/sweep 紧循环内用, ~2300 次调用)
if nargin < 2 || isempty(Qs)
    Qs = linspace(0.3, 2.2, 40)';
end
if nargin < 3, quiet = false; end
% ---- 风洞侧参数 (缺风洞项目时用内置备份值, 与 WindTunnel3D balanced 预设一致) ----
T = [];
try
    if exist(fullfile(P.load.path,'tunnel_params.m'),'file')
        if ~any(strcmp(P.load.path, path)), addpath(P.load.path); end   % 幂等 addpath (紧循环降噪)
        T = tunnel_params();
    end
catch
    T = [];
end
if isempty(T)
    fprintf('[tunnel_load] [!] 未找到 WindTunnel3D (load.path=%s), 使用内置备份参数 — 同参数网格可能随环境变化, 请检查套件布局!\n', P.load.path);
    T = struct('V_test',25, 'ts',struct('W',0.30,'H',0.20,'L',0.20), ...
        'ct',struct('R',6,'L',0.60), 'sc',struct('L',0.45,'nScreen',2), ...
        'df',struct('W_out',0.40,'H_out',0.26,'L',0.75), ...
        'fd',struct('W_in',P.fd.W_in,'H_in',P.fd.H_in,'L',0.50,'etaD',0.72,'screen',true));
end
if ~isfield(T,'fd')   % 风洞项目尚未更新前置扩散段参数时, 从风机侧补齐默认
    T.fd = struct('W_in',P.fd.W_in,'H_in',P.fd.H_in,'L',0.50,'etaD',0.72,'screen',true);
end
rho = 1.204;

% ---- 截面积链 (m²) ----
A.flange = P.fd.W_in * P.fd.H_in;            % 风机出口法兰 (弯头内流速≈法兰流速)
A.fdIn   = T.fd.W_in * T.fd.H_in;
A.sett   = T.ct.W_in * T.ct.H_in;            % 稳定段 = 收缩段入口截面
A.ts     = T.ts.W * T.ts.H;
A.dfOut  = T.df.W_out * T.df.H_out;
Vts = @(Q) Q ./ A.ts;

% ---- 部件损失系数 (各自参考流速) ----
c = {};
% 1. 出口 90° 弯头 (风机组件, 但阻力计入系统 — 风机静压须覆盖它)
c(end+1,:) = {'出口弯头', A.flange, P.vol.zetaBend};
% 2. 前置扩散段 (宽角扩散 + 效率模型: 损失 = (1-η_d)·(1-1/AR²) + 摩擦, 参考入口流速)
ARfd = A.sett / A.fdIn;
zetaFd = (1-T.fd.etaD)*(1-1/ARfd^2) + 0.10;
c(end+1,:) = {'前置扩散段', A.fdIn, zetaFd};
% 3. 前置扩散段出口整流网 (稳定宽角扩散边界层, 可选)
if isfield(T.fd,'screen') && T.fd.screen
    c(end+1,:) = {'扩散段出口网', A.sett, 0.80};
end
% 4. 蜂窝器
c(end+1,:) = {'蜂窝器', A.sett, 0.22};
% 5. 纱网 ×nScreen (30 目, 开孔比~0.58: ζ≈(1-β)^1.5/β²≈0.8)
for i = 1:T.sc.nScreen
    c(end+1,:) = {sprintf('纱网#%d',i), A.sett, 0.80};
end
% 6. 收缩段 (quintic 大收缩比, 损失极小)
c(end+1,:) = {'收缩段', A.ts, 0.025};
% 7. 试验段摩擦 (PLA, f≈0.015, L/Dh)
Dh_ts = 4*A.ts/(2*(T.ts.W+T.ts.H));
c(end+1,:) = {'试验段摩擦', A.ts, 0.015*T.ts.L/Dh_ts};
% 8. 试验段测试物 (空风洞基线为 0; 装物体后按堵塞比粗修)
c(end+1,:) = {'测试物(空置)', A.ts, 0.0};
% 9. 后扩散段 (2θ≈5.6°, η_d≈0.85)
ARdf = A.dfOut/A.ts;
zetaDf = (1-0.85)*(1-1/ARdf^2) + 0.03;
c(end+1,:) = {'后扩散段', A.ts, zetaDf};
% 10. 排空动能 (出口射流直排大气, 动能全损)
c(end+1,:) = {'排空动能', A.dfOut, 1.0};

% ---- K 系数与曲线 ----
K = 0;  Kf = 0;  Kr = 0;
for i = 1:size(c,1)
    Ki = c{i,3} * rho / (2 * c{i,2}^2);
    K = K + Ki;
    if i <= 5, Kf = Kf + Ki; else, Kr = Kr + Ki; end   % 1-5=法兰→稳定段出口(前段), 6-10=后段
end
L.K = K;                          % Pa/(m³/s)²
L.Kfront = Kf;                    % 法兰后到稳定段出口
L.Krear = Kr;                     % 稳定段出口后排空
L.Kbend = c{1,3} * rho / (2*c{1,2}^2);   % 出口弯头 (风机域已含, 自适应出口需扣除)
L.KafterFlange = K - L.Kbend;            % GPU 'fan' 域自适应出口用
L.dp = @(Q) K .* Q.^2;
L.Qs = Qs;
L.dpQ = L.dp(Qs);
L.rho = rho;
L.comp = c;                       % {名称, A_i, ζ_i}
L.A = A;
L.Vts = Vts(Qs);

% ---- 设计点分解表 ----
Qd = P.load.V_test * A.ts;        % 设计风量
L.Qdesign = Qd;
L.Vflange = Qd / A.flange;
if ~quiet
    fprintf('系统阻力: K = %.1f Pa/(m³/s)² | 设计风量 %.2f m³/s (%.0f m³/h) @ 试验段 %.0f m/s\n', ...
        K, Qd, Qd*3600, P.load.V_test);
    fprintf('  风机法兰流速 %.1f m/s | 设计点系统压降 %.0f Pa\n', L.Vflange, L.dp(Qd));
end
if nargout > 1 || ~nargout
    fprintf('  %-14s %8s %8s %10s\n', '部件', 'A(m²)', 'ζ', 'Δp(Pa)');
    for i = 1:size(c,1)
        fprintf('  %-14s %8.4f %8.3f %10.1f\n', c{i,1}, c{i,2}, c{i,3}, ...
            c{i,3}*(rho/2)*(Qd/c{i,2})^2);
    end
end
end
