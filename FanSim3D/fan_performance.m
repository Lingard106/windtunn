function R = fan_performance(P, Qs, rpm, L, quiet)
if nargin < 1, P = fan_params(); end
% fan_performance — 抽风式离心风机一维平均流面性能模型
% 物理链: 速度三角形(进口无预旋/出口滑移) -> 欧拉压升 -> 损失模型(冲击/叶表摩擦/
%         弯曲/回流/漏气/盘摩擦/蜗壳回收) -> p_sf(Q) 性能曲线 -> 与风洞阻力曲线
%         tunnel_load 求交点 -> 工况点(风量/风速/轴功率/效率) + 稳定性指标
% 输入: P=fan_params | Qs=风量网格(m³/s, []=自动) | rpm=转速覆盖([]=P.imp.rpm)
%       L=tunnel_load 结果([]=内部计算) | quiet=true 静音(批量扫描用)
% 输出: R.Q/.p_sf/.p_t/.P_shaft/.eta_sf 曲线 + R.op 工况点 + R.stab 稳定性 + R.tri 速度三角形
if nargin < 2 || isempty(Qs),  Qs = []; end
if nargin < 3 || isempty(rpm), rpm = []; end
if nargin < 4 || isempty(L),   L = [];  end
if nargin < 5, quiet = false; end
if isempty(rpm), rpm = P.imp.rpm; end
if isempty(L),   L = tunnel_load(P, [], true); end   % 静音 (本函数已有自己的摘要打印)
rho = L.rho;

% ---- 几何与角参数 ----
r1 = P.imp.D1/2;  r2 = P.imp.D2/2;  b1 = P.imp.b2;  b2 = P.imp.b2;
Z  = P.imp.Z;     tB = P.imp.tB;
beta2A = min(max(P.imp.beta2A, 25), 168);            % 出口安装角 (deg)
A1 = pi*2*r1*b1;  A2 = pi*2*r2*b2;                   % 进/出口圆柱流通面积
u1 = pi*2*r1*rpm/60;  u2 = pi*2*r2*rpm/60;

% 叶片进口角: 自动 = 设计风量无冲击 (+4° 设计冲角, 与叶厚堵塞迭代)
beta1A = P.imp.beta1A;
if isempty(beta1A)
    bA = 20;
    for it = 1:6
        t1 = max(1 - Z*tB/(pi*2*r1*sind(bA)), 0.3);
        bA = atand((L.Qdesign/(A1*t1))/u1) + 4;
    end
    beta1A = min(max(bA, 8), 60);
end

% ---- 滑移系数 (Wiesner + 前弯修正) ----
slipK = P.imp.slipK;
if isempty(slipK)
    if beta2A <= 90, slipK = 1.0; else, slipK = 1.15; end
end
epsN = min(slipK*sqrt(abs(cosd(beta2A)))/Z^0.7, 0.5);% 无量纲滑移 (x u2)

% ---- 叶片圆弧弧长 (与 fan_geometry 同一构造) ----
[~, ~, ~, BL] = fan_blade_arc(r1, r2, beta1A, beta2A);

% ---- 静态中间量 ----
tau2 = max(1 - Z*tB/(pi*2*r2*abs(sind(beta2A))), 0.5);
o1 = pi*2*r1/Z*sind(beta1A);                         % 进口叶道喉宽
o2 = pi*2*r2/Z*abs(sind(beta2A));                    % 出口叶道喉宽
oM = (o1+o2)/2;  Dh = 2*b2*oM/(b2+oM);               % 叶道水力直径
A_out = P.vol.W_out*P.vol.B;                         % 蜗壳出口(法兰)面积
Re2 = u2*P.imp.D2/1.5e-5;
cM  = P.loss.cM0*(Re2/1e6)^(-0.1);
P_df = cM*rho*u2^3*P.imp.D2^2*(pi/2);                % 盘摩擦功率 (W)
fwd = max(beta2A-90, 0);                             % 前弯度 (deg)

% ---- 风量网格 ----
Qd = L.Qdesign;
if isempty(Qs), Qs = linspace(0.12, 1.95, 76)'*Qd; end
nQ = numel(Qs);
[p_sf, p_t, P_sh, etaS, etaT, i1v, w1w2, c2uv] = deal(zeros(nQ,1));

for k = 1:nQ
    Q = Qs(k);
    tau1 = max(1 - Z*tB/(pi*2*r1*sind(beta1A)), 0.3);
    c1m  = Q/(A1*tau1);
    i1   = beta1A - atand(c1m/u1);                   % 进口冲角 (deg, 正=冲击)
    c2m  = Q/(A2*tau2*P.loss.etaVol);
    c2u  = u2 - c2m/tand(beta2A) - u2*epsN;          % 滑移后出口切向分量
    w1   = hypot(u1, c1m);
    w2   = hypot(u2-c2u, c2m);
    c2   = hypot(c2u, c2m);

    % 吸力眼: 物理上限 (眼径 ≤ 0.87·D2, 且面积不超叶道进口环面积需求)
    D0 = min([P.imp.D0, 0.87*P.imp.D2, sqrt(4*2*r1*b2)]);
    A0 = pi/4*D0^2;
    c0 = Q/A0;
    dpEye = 0.5*(1-min(A0/A1,1))^2*(rho/2)*max(c0-c1m,0)^2;  % 眼→叶道突扩损失

    p_th = rho*u2*c2u;                               % 欧拉全压升 (进口无预旋)
    dpInc  = P.loss.zetaInc*(rho/2)*w1^2*sind(min(abs(i1),60))^2;
    dpSf   = P.loss.fBlade*(BL/Dh)*(rho/2)*((w1+w2)/2)^2;
    dpCrv  = P.loss.zetaCurv*(1+fwd/45)*(rho/2)*w1^2;
    dpRec  = P.loss.zetaRec*(1+fwd/90)*(rho/2)*u2^2;
    dpLeak = (1-P.loss.etaVol)*p_th;
    cOut   = Q/A_out;                                % 蜗壳出口(法兰)流速
    dpVol  = (1-P.vol.etaV)*0.5*rho*max(c2^2-cOut^2,0) + P.vol.zetaV*0.5*rho*c2^2;
    dpIn0  = 0.05*(rho/2)*c0^2;                      % 钟形口进口损失

    pT   = p_th - dpInc - dpSf - dpCrv - dpRec - dpLeak - dpVol - dpIn0 - dpEye;
    pS   = pT - 0.5*rho*cOut^2;                      % 风机静压 (法兰处)

    Psh = (rho*Q*u2*c2u + P_df + dpRec*Q)/P.imp.etaMech;
    p_sf(k) = pS;  p_t(k) = pT;  P_sh(k) = Psh;
    etaS(k) = Q*pS/max(Psh, eps);  etaT(k) = Q*pT/max(Psh, eps);
    i1v(k) = i1;  w1w2(k) = w1/max(w2,eps);  c2uv(k) = c2u;
end

% ---- 工况点: p_sf(Q) 与风洞阻力 K·Q² 求交 (取右支稳定交点) ----
fload = L.dp(Qs);
fcross = p_sf - fload;
op = struct('Q',NaN,'Vts',NaN,'p',NaN,'P',NaN,'etaS',NaN,'etaT',NaN,'stable',false);
for k = 1:nQ-1
    if fcross(k) >= 0 && fcross(k+1) < 0             % 正→负穿越 = 右支稳定交点
        t = fcross(k)/(fcross(k)-fcross(k+1));
        op.Q  = Qs(k) + t*(Qs(k+1)-Qs(k));
        op.stable = true;
    end
end
if isfinite(op.Q)
    op.Vts  = op.Q/L.A.ts;
    op.p    = L.dp(op.Q);
    op.P    = interp1(Qs, P_sh, op.Q);
    op.etaS = interp1(Qs, etaS, op.Q);
    op.etaT = interp1(Qs, etaT, op.Q);
end

% ---- 稳定性指标 ----
[~, ipk] = max(p_sf);
stab.Qpeak = Qs(ipk);  stab.peakPs = p_sf(ipk);
stab.surgeMargin = NaN;
if isfinite(op.Q) && op.Q > Qs(ipk)
    stab.surgeMargin = (op.Q - Qs(ipk))/op.Q;        % 工况点对喘振边界(峰压点)的流量裕度
end
stab.diffusion   = interp1(Qs, w1w2, Qd);            % w1/w2 (<1 加速叶道, 前弯无叶道失速)
stab.incidenceDes= interp1(Qs, i1v, Qd);             % 设计冲角 (deg)
stab.slopeOK     = isfinite(op.Q) && op.Q > Qs(ipk); % 工况点在性能曲线下降段
stab.specificNq  = rpm*sqrt(max(op.Q,eps))/max(op.p,1)^0.75;  % 比转速 (m³/s, Pa)
stab.bpf         = Z*rpm/60;                         % 叶片通过频率 (Hz)
stab.powerOK     = isfinite(op.P) && op.P < 2200;    % 电机预算 2.2 kW
stab.u2          = u2;

% ---- 控制台摘要 ----
if ~quiet
fprintf('[fan_performance] %s | D2=%.0f D1=%.0f b2=%.0f mm Z=%d beta2A=%.0f° beta1A=%.1f° n=%.0f rpm\n', ...
    P.preset, P.imp.D2*1e3, P.imp.D1*1e3, P.imp.b2*1e3, Z, beta2A, beta1A, rpm);
fprintf('  叶尖 u2=%.1f m/s | 叶片弧长 %.0f mm | 滑移 σ=%.3f | 设计冲角 %.1f° | w1/w2=%.2f | BPF=%.0f Hz | nq=%.1f\n', ...
    u2, BL*1e3, 1-epsN, stab.incidenceDes, stab.diffusion, stab.bpf, stab.specificNq);
if isfinite(op.Q)
    fprintf('  工况点: Q=%.2f m³/s (%.0f m³/h) → 试验段 %.1f m/s | p_sf=%.0f Pa | 轴功率 %.0f W | η静=%.1f%%\n', ...
        op.Q, op.Q*3600, op.Vts, op.p, op.P, op.etaS*100);
    fprintf('  稳定性: 喘振流量裕度 %s | 曲线右支 %s | 功率预算 %s\n', ...
        marginTxt(stab.surgeMargin), yn(stab.slopeOK,'落在峰压左侧(不稳定区)'), yn(stab.powerOK,'>2.2 kW'));
else
    fprintf('  [!] 无工况点交点: 风机能力与风洞阻力不匹配 (|p_sf-K·Q²| 峰值 %.0f Pa)\n', max(abs(fcross)));
end
end

% ---- 输出打包 ----
tau1d = max(1 - Z*tB/(pi*2*r1*sind(beta1A)), 0.3);
R = struct('P',P, 'rpm',rpm, 'Q',Qs, 'p_sf',p_sf, 'p_t',p_t, 'P_shaft',P_sh, ...
    'eta_sf',etaS, 'eta_t',etaT, 'load',L, 'op',op, 'stab',stab, 'i1',i1v, 'w1w2',w1w2);
R.tri = struct('u1',u1, 'u2',u2, 'beta1A',beta1A, 'beta2A',beta2A, ...
    'c1m',L.Qdesign/(A1*tau1d), ...
    'c2m',Qd/(A2*tau2*P.loss.etaVol), ...
    'c2u',interp1(Qs, c2uv, Qd), 'w1w2',stab.diffusion);
R.bladeLen = BL;
end

%% ==================== 局部工具 ====================
function s = marginTxt(m)
if isnan(m), s = '—(无交点)'; else, s = sprintf('%.0f%%', m*100); end
end
function s = yn(c, bad)
if c, s = '✓'; else, s = ['✗ ' bad]; end
end
