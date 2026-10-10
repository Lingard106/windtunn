function G = tunnel_geometry(P)
% 风洞沿程几何 (各向异性: 半宽 a 与半高 b 独立型线) / 一维核心速度 / 壁面倾角
% 收缩型线由 P.ct.curve 选择; 试验段带微扩张补偿; 全部尺寸可由 tunnel_params 调节

[aIn,  bIn ] = deal(P.ct.W_in/2,  P.ct.H_in/2);    % 收缩段入口半宽/半高
[aOut, bOut] = deal(P.ts.W/2,     P.ts.H/2);       % 试验段入口
Lc = P.ct.L;

s    = linspace(0, 1, 80)';
[f, dfds] = ctCurve(s, P.ct.curve);
x_ct = s * Lc;
a_ct = aOut + (aIn - aOut)*f;          % 半宽型线
b_ct = bOut + (bIn - bOut)*f;          % 半高型线 (同一形状函数)

x_sc = linspace(0, P.sc.L, 20)';
x_ts = linspace(0, P.ts.L, 40)';
g    = tand(P.ts.growth);
a_ts = aOut + g*x_ts;                  % 微扩张补偿
b_ts = bOut + g*x_ts;
a_te = a_ts(end);  b_te = b_ts(end);
x_df = linspace(0, P.df.L, 30)';
a_df = a_te + (P.df.W_out/2 - a_te)*(x_df/P.df.L);
b_df = b_te + (P.df.H_out/2 - b_te)*(x_df/P.df.L);

% 前置扩散段 (风机出口法兰 -> 稳定段入口): 五次型线宽角扩张, 折入稳定段段内
x_fd = linspace(0, P.fd.L, 30)';
sFd  = x_fd / P.fd.L;
fFd  = 10*sFd.^3 - 15*sFd.^4 + 6*sFd.^5;          % quintic (0→1)
a_fd = (P.fd.W_in/2) + (aIn - P.fd.W_in/2)*fFd;
b_fd = (P.fd.H_in/2) + (bIn - P.fd.H_in/2)*fFd;

G.x = [x_fd; P.fd.L + x_sc(2:end); P.fd.L+P.sc.L + x_ct(2:end); P.fd.L+P.sc.L+Lc + x_ts(2:end); P.fd.L+P.sc.L+Lc+P.ts.L + x_df(2:end)];
G.a = [a_fd; repmat(aIn, numel(x_sc)-1, 1); a_ct(2:end); a_ts(2:end); a_df(2:end)];
G.b = [b_fd; repmat(bIn, numel(x_sc)-1, 1); b_ct(2:end); b_ts(2:end); b_df(2:end)];

% 一维核心速度 (连续性): U = U_ts * (a_te*b_te)/(a*b)
U_ts = P.V_test;
G.U  = U_ts * (a_te*b_te) ./ (G.a .* G.b);

G.thetaA = atand(gradient(G.a, G.x));   % 侧壁倾角 (deg)
G.thetaB = atand(gradient(G.b, G.x));   % 顶底壁倾角 (deg)

G.xSec = [0, P.fd.L+P.sc.L, P.fd.L+P.sc.L+Lc, P.fd.L+P.sc.L+Lc+P.ts.L, P.fd.L+P.sc.L+Lc+P.ts.L+P.df.L];
G.secNames = {'前置扩散+稳定段','收缩段','试验段','扩散段'};

% 扩散段等效锥角 (当量直径 = 水力直径 4A/P, 分离检查 2θ<=7°)
De_in  = 4*a_te*b_te/(a_te+b_te);       % 入口: 4ab/(a+b) (a,b 为半宽/半高)
De_out = 2*P.df.W_out*P.df.H_out/(P.df.W_out+P.df.H_out);  % 出口: 2WH/(W+H)
G.theta_df_eq = atand((De_out - De_in)/(2*P.df.L));

% 前置扩散段等效锥角 (宽角扩散: 面积比大, 2θ 远超 7°, 靠出口整流网 + 稳定段纱网镇流)
De_fi = 2*P.fd.W_in*P.fd.H_in/(P.fd.W_in+P.fd.H_in);
De_fs = 4*aIn*bIn/(aIn+bIn);
G.theta_fd_eq = atand((De_fs - De_fi)/(2*P.fd.L));
G.AR_fd = (2*aIn*2*bIn)/(P.fd.W_in*P.fd.H_in);    % 前置扩散段面积比

% 收缩段峰值壁面角
mk = G.x >= G.xSec(2) & G.x <= G.xSec(3);
G.theta_ct_max = max(max(abs(G.thetaA(mk))), max(abs(G.thetaB(mk))));
end

%% ==================== 收缩型线库 ====================
function [f, dfds] = ctCurve(s, type)
% 归一化形状: f(0)=1, f(1)=0 (入口/出口斜率尽量为零)
switch lower(type)
    case 'linear'
        f = 1 - s;  dfds = -ones(size(s));
    case 'cubic'
        f = 1 - (3*s.^2 - 2*s.^3);  dfds = -(6*s - 6*s.^2);
    case 'sine'
        f = 0.5*(1 + cos(pi*s));  dfds = -0.5*pi*sin(pi*s);
    case 'twostage'
        % 两段式: 前 60% 完成 72% 收缩 (快), 后段平缓 — 兼顾长度与出口均匀性
        f = ones(size(s)); dfds = zeros(size(s));
        m1 = 0.6; drop1 = 0.72;
        i1 = s <= m1;  t = s(i1)/m1;
        f(i1) = 1 - drop1*(10*t.^3 - 15*t.^4 + 6*t.^5);
        dfds(i1) = -drop1*(30*t.^2 - 60*t.^3 + 30*t.^4)/m1;
        i2 = ~i1;  t = (s(i2)-m1)/(1-m1);
        f(i2) = (1-drop1)*(1 - (10*t.^3 - 15*t.^4 + 6*t.^5));
        dfds(i2) = -(1-drop1)*(30*t.^2 - 60*t.^3 + 30*t.^4)/(1-m1);
    otherwise   % 'quintic' 五次匹配 (默认, 两端斜率+曲率为零)
        f = 1 - (10*s.^3 - 15*s.^4 + 6*s.^5);
        dfds = -(30*s.^2 - 60*s.^3 + 30*s.^4);
end
end
