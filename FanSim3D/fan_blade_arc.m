function [phi, C, Ra, Lb, th] = fan_blade_arc(r1, r2, beta1A, beta2A, tEval)
% fan_blade_arc — 圆弧叶片构造 (单一真源: fan_performance 用弧长, fan_geometry 用全形)
% 约定: (y,z) 平面, 极角自 +y 向 +z; 进口点 P1=(r1,0), 出口点 P2=r2·(cosφ,sinφ)
%       叶片角自切向量起: t1 = sind(b1)·ê_r + cosd(b1)·ê_θ (进口)
%                         t2 = sind(180-b2)·ê_r2 + cosd(180-b2)·ê_θ2 (出口, b2>90 前弯)
% 方法: 给定包角 φ, "圆心在两端法线上"是 2×2 线性方程 (直接解 C), 残差 = |C-P1|-|C-P2|,
%       对 φ 一维求根; 根须通过 半径/弧长/流向 三重核验 (滤除切向平行的退化假根)
% 输入: r1/r2 进出口半径 (m), beta1A/beta2A 安装角 (deg), tEval=参数点 (0~1, []=不取点)
% 输出: phi 包角(deg) | C 圆心(2) | Ra 圆弧半径 | Lb 弧长 | th=弧段角度范围(deg, 绕 C)
%       tEval 给定时追加输出 P (numel(tEval)×2 点坐标, t=0 进口 → t=1 出口)
if nargin < 5, tEval = []; end
P1 = [r1, 0];
t1 = [sind(beta1A), cosd(beta1A)];

resid = @(ph) arcResid(ph, P1, t1, r2, beta2A);
phis = linspace(1, 240, 320);
rr = arrayfun(resid, phis);
fin = isfinite(rr);
if nnz(fin) < 3
    error('圆弧叶片无解: r1=%.3f r2=%.3f beta1A=%.1f beta2A=%.1f', r1, r2, beta1A, beta2A);
end
rrf = interp1(phis(fin), rr(fin), phis, 'linear', 'extrap');

best = [];
for k = 1:numel(phis)-1
    if isfinite(rrf(k)) && rrf(k)*rrf(k+1) < 0
        try
            phk = fzero(resid, [phis(k), phis(k+1)]);
        catch
            continue;
        end
        if ~isfinite(phk), continue; end
        [Ck, resv] = centerFor(phk, P1, t1, r2, beta2A);
        if ~isfinite(resv) || abs(resv) > 1e-6*max(r1, r2), continue; end   % 假根核验
        Rak = norm(Ck - P1);
        P2k = r2*[cosd(phk), sind(phk)];
        chord = norm(P2k - P1);
        if Rak < 0.55*chord || Rak > 40*chord, continue; end                % 半径合理性
        th1 = atan2d(P1(2)-Ck(2), P1(1)-Ck(1));
        th2 = atan2d(P2k(2)-Ck(2), P2k(1)-Ck(1));
        dth = mod(th2 - th1, 360);  if dth > 180, dth = dth - 360; end
        Lbk = Rak*abs(dth)*pi/180;
        if Lbk < 0.999*chord, continue; end                                 % 弧长≥弦长
        % 流向核验: 沿弧方向切向与进/出口气流方向同向 (点积>0)
        sg = sign(dth);  if sg == 0, continue; end
        T1 = sg*[-(P1(2)-Ck(2)), (P1(1)-Ck(1))];  T1 = T1/norm(T1);
        T2 = sg*[-(P2k(2)-Ck(2)), (P2k(1)-Ck(1))]; T2 = T2/norm(T2);
        if dot(T1, t1) < 0.3 || dot(T2, tang2(P2k, beta2A)) < 0.3, continue; end
        Sk = struct('phiW',phk,'C',Ck,'Ra',Rak,'Lb',Lbk,'th',[th1, th1+dth]);
        if isempty(best) || Sk.Lb < best.Lb, best = Sk; end                 % 最短弧优先
    end
end
if isempty(best)
    error('圆弧叶片无解: r1=%.3f r2=%.3f beta1A=%.1f beta2A=%.1f (该组合无法单圆弧成型)', ...
        r1, r2, beta1A, beta2A);
end
phi = best.phiW;  C = best.C;  Ra = best.Ra;  Lb = best.Lb;  th = best.th;

if ~isempty(tEval)
    P = C + Ra*[cosd(th(1)+tEval(:)*diff(th)), sind(th(1)+tEval(:)*diff(th))];
end
end

%% ==================== 内部函数 ====================
function t2 = tang2(P2, b2)
% 出口全局流向 (P2 极角由 P2 自身决定)
ph = atan2d(P2(2), P2(1));
er = [cosd(ph), sind(ph)];  et = [-sind(ph), cosd(ph)];
t2 = sind(180-b2)*er + cosd(180-b2)*et;
end

function [C, res] = centerFor(ph, P1, t1, r2, beta2A)
% 给定包角: 解 2×2 线性方程 dot(C,t1)=dot(P1,t1), dot(C,t2)=dot(P2,t2)
P2 = r2*[cosd(ph), sind(ph)];
t2 = tang2(P2, beta2A);
A = [t1(:), t2(:)];
if abs(det(A)) < 1e-12, C = [NaN NaN]; res = NaN; return; end
b = [dot(P1, t1); dot(P2, t2)];
C = (A \ b)';
res = norm(C - P1) - norm(C - P2);
end

function res = arcResid(ph, P1, t1, r2, beta2A)
[C, res] = centerFor(ph, P1, t1, r2, beta2A);
if ~isfinite(res), res = NaN; end
end
