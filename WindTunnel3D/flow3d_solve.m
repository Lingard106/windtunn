function S = flow3d_solve(P, G)
% flow3d_solve — 体素网格定常流场求解器 (最小方格单元 + 邻居耦合)
% 三步定常重建 (物理正确且数值稳健, 替代高 Re 下刚性的瞬态 N-S):
%  [1] 粘性核心流: 边界层排挤迭代 — 壁面摩擦使核心加速, 含层流/湍流转捩(Re_x)
%      与 cf(Re), 直接计入气体粘性与雷诺数
%  [2] 体素速度剖面: 逐单元按壁距填充 — 层流抛物型 / 湍流粘性底层+对数律+1/7
%      幂律; 混合长度涡粘 nu_t 用 6 邻居差分, 多轮 3D 扩散松弛 (邻居通量耦合)
%  [3] 横向速度: 逐站位解 2D 泊松方程 nabla^2(phi) = -du/dx (遮蔽体素 Neumann),
%      v=dφ/dy, w=dφ/dz — 离散无散且壁面法向零; 可选 Boussinesq 浮力分层环流
% 可压缩: 逐体素 Mach 与等熵密度比; normalShock() 模块供高速工况
% 输出 S: U,V,W,spd,nut,mach,rhoRatio,vort,mask,x,y,z,d2w,delta,T,diag

nx=P.flow.nx; ny=P.flow.ny; nz=P.flow.nz;
L = G.x(end);
HboxY = 2*max(G.a)*1.06;               % 域宽 (收缩段入口最宽处 ×1.06)
HboxZ = 2*max(G.b)*1.06;               % 域高 — 矩形截面 y/z 独立, 不浪费竖向格数
dx=L/nx; dy=HboxY/ny; dz=HboxZ/nz;
xc = ((1:nx)-0.5)'*dx;
yc = -HboxY/2 + ((1:ny)-0.5)*dy;
zc = -HboxZ/2 + ((1:nz)-0.5)*dz;

aC = interp1(G.x, G.a, xc, 'pchip', 'extrap');
bC = interp1(G.x, G.b, xc, 'pchip', 'extrap');
U1inv = interp1(G.x, G.U, xc, 'pchip', 'extrap');
A3 = reshape(aC,[],1,1);  B3 = reshape(bC,[],1,1);
Y3 = reshape(yc,1,ny,1);  Z3 = reshape(zc,1,1,nz);

mask = abs(Y3) < A3 & abs(Z3) < B3;                 % 洞内体素
d2w  = min(A3-abs(Y3), B3-abs(Z3));                 % 到最近壁面距离
perim = 4*(aC+bC);  area = 4*aC.*bC;

%% ---- [1] 粘性核心流: 排挤厚度定点迭代 (平板边界层 + PLA粗糙 + 扩散段逆压增厚) ----
% 有效长度 xe: 收缩段顺压抑制边界层生长 (x0.25), 试验段/扩散段正常发展
xe = xc;
mk = xc > G.xSec(2);          xe(mk) = 0.25*(xc(mk) - G.xSec(2));
mk = xc > G.xSec(3);          xe(mk) = 0.08 + (xc(mk) - G.xSec(3));
mk = xc > G.xSec(4);          xe(mk) = 0.08 + P.ts.L + (xc(mk) - G.xSec(4));
indD = xc >= G.xSec(4);                                        % 扩散段站位

ks = 0;
if isfield(P,'wall') && P.wall.on, ks = P.wall.ks; end

% 扩散段逆压增厚系数: 扩张角越接近经验分离线 (2θ≈10°), δ 增厚越多 (上限 2.2×)
halfAng = atand((max(G.a(G.x>=G.xSec(4)))-P.ts.W/2)/max(P.df.L,eps))*2;
apgFac  = 1 + 1.2*min(1, max(halfAng,0)/10);

uCore = U1inv;
cf    = 0.664./sqrt(max(uCore.*xe/P.nu, 1));     % 迭代初值 (光滑层流)
for it = 1:40
    Re_x  = uCore .* xe / P.nu;
    laminar = Re_x <= P.transitionRe;
    % PLA 粗糙度: Re_ks = ks*uTau/nu > 5 时提前转捩并提高摩擦 (Schlichting 全粗糙式)
    uTau0  = uCore .* sqrt(cf/2);
    rough  = ks*uTau0/P.nu > 5;
    laminar = laminar & ~rough;
    dstar = 1.72*xe./sqrt(max(Re_x,1));                    % 层流排挤厚度
    dstar(~laminar) = 0.046*xe(~laminar)./max(Re_x(~laminar),1).^0.2;  % 湍流
    dstar = min(dstar, 0.05*min(aC,bC));                        % 物理上限: δ* ≤ 10% 半宽
    cf = 0.664./sqrt(max(Re_x,1));                         % 层流局部摩擦系数
    cfT = 0.0592*max(Re_x(~laminar),1).^(-0.25);           % 湍流光滑
    if ks > 0
        cfR = (1.89 + 1.62*log10(max(xe(~laminar)./ks, 30))).^(-2.5);  % 湍流全粗糙
        cfT = max(cfT, cfR);
    end
    cf(~laminar) = cfT;
    Aeff  = area - perim.*dstar;                           % 有效流通面积
    uNew  = (U1inv(1)*Aeff(1)) ./ max(Aeff, 0.2*area);
    uCore = uCore + 0.6*(uNew - uCore);
    uCore = min(max(uCore, 0.5*U1inv), 1.6*U1inv);   % 物理界限: 防强逆压迭代发散
end
uTau  = uCore.*sqrt(cf/2);
dSub  = 11.6*P.nu./max(uTau, 1e-6);                        % 粘性底层厚度
dLam  = 2.9*dstar;                                         % 层流 δ99 = 2.9δ*
dTrb  = 8*dstar;                                           % 湍流 δ99 = 8δ* (1/7 幂律)
delta = dLam;  delta(~laminar) = dTrb(~laminar);
delta = max(delta, 0.35*dy);
% 扩散段逆压增厚 (湍流边界层在逆压下 δ 比平板快 ~2×, 仅影响视觉/剖面厚度;
% 核心风速的排挤迭代仍用平板基准, 保持 R 诊断口径一致)
dDiff = xc >= G.xSec(4) & ~laminar;
delta(dDiff) = delta(dDiff).*min(1 + 1.0*max(halfAng/10,0), 2.0);
delta = min(delta, 0.35*min(aC,bC));                   % 视觉上限: δ99 ≤ 35% 半宽
xTr = xc(find(~laminar, 1, 'first'));                      % 首个湍流站位

% 摩擦对核心速度的影响 (有摩擦 vs 无摩擦对比, 直接可见)
[~, ds0] = deal(0);
Aeff0 = area;                                              % 无摩擦理想核心
uIdeal = (uCore(1)*Aeff(1)) ./ max(area, 0.2*area);
fGain = (uCore./max(uIdeal,eps) - 1)*100;
if isempty(xTr), xTr = Inf; end

uTau3  = reshape(uTau, [],1,1);
delta3 = reshape(delta,[],1,1);
Lam3   = repmat(reshape(laminar,[],1,1), [1 ny nz]);

%% ---- [2] 体素速度剖面 ----
dn = d2w ./ delta3;                                        % y/delta
up = d2w .* uTau3 / P.nu;                                  % d+ 壁面单位
u17   = uCore .* min(dn.^(1/7), 1);                        % 外区 1/7 幂律
uLog  = (2.44*log(max(up,1.01)) + 5.0) .* uTau3;           % 对数律 (有量纲)
uTurb = min(max(uLog, 0), u17);                            % 内层对数+外层幂律
dnL   = min(d2w ./ reshape(3*dstar,[],1,1), 1);            % 抛物线宽度对齐 δ* ( deficit=δ* )
uLam  = uCore .* (1 - (1-dnL).^2);                         % 层流抛物型
U = uTurb;  U(Lam3) = uLam(Lam3);
sub = up < 5 & ~Lam3;                                      % 湍流站位的粘性底层
uTauE = repmat(uTau3, [1 ny nz]);
U(sub) = up(sub).*uTauE(sub);
U(~mask) = 0;
U = min(U, reshape(uCore,[],1,1)*1.05);
DBG = ~isempty(getenv('WT_DBG'));
if DBG
    its0 = find(xc>=G.xSec(3),1);
    uP = squeeze(U(its0,:,:));  dP = squeeze(d2w(its0,:,:));
    fprintf('  [dbg] 剖面: d2w/mm -> u: '); n0=0;
    for j = 1:ny, for k = 1:nz
        if dP(j,k)>0 && n0<8, fprintf('%.0f:%.1f ', dP(j,k)*1000, uP(j,k)); n0=n0+1; end
    end, end
    fprintf(' | delta=%.1fmm uCore=%.1f\n', delta(its0)*1000, uCore(its0));
end

% 混合长度涡粘: |du/dn| 用 6 邻居差分 (单元间耦合)
DBGt = ~isempty(getenv('WT_DBG'));  if DBGt, tAll = tic; tS2 = tic; end
nuN = abs(U - nbMean(U, mask)) ./ max(d2w, 0.25*dy);
nut = min((0.41*d2w).^2 .* nuN .* ~Lam3, 500*P.nu);
nuT = P.nu + nut;

% 3D 扩散松弛 (邻居通量耦合, 定常扩散平衡; 显式步长受最粘性单元限制)
% 单精度扫描: 内存带宽减半 ~1.6x 提速, 结果差异仅浮点舍入 (~1e-6 相对)
dtE = 0.2 * min(dx,dy)^2 / (6*max(nuT(mask)) + eps);
Us  = single(U);   nuTs = single(nuT);   dtEs = single(dtE);
for sw = 1:60
    Us = Us + dtEs .* nuTs .* lap3(Us, mask);
    Us(~mask) = 0;
end
U = min(double(Us), reshape(uCore,[],1,1)*1.08);
if DBGt, fprintf('  [dbg] stage 扩散松弛60轮: %.1f s\n', toc(tS2)); end

% 质量流量守恒重标定 (逐站缩放到入口流量, 消除剖面形状近似的系统偏差)
Q0   = uCore(1)*(area(1) - perim(1)*dstar(1));
qNow = squeeze(sum(sum(U.*mask,2),3))*dy*dz;
U    = min(U .* reshape(Q0./max(qNow,eps), [],1,1), reshape(uCore,[],1,1)*1.05);

%% ---- 逆压梯度与分离判据 (一维湍流边界层判据, 扩散段) ----
% R = (dp/dx·δ*)/τw: 壁面摩擦对抗逆压的能力比; R→1 预示分离 (湍流经验阈值 ~0.5-1)
% 口径与 boundary_layer_study 一致: 局部 uCore/δ*, 有效长度 xe
uCs  = smoothdata(uCore(:),'movmean',9); dU1  = gradient(uCs, xc);
dpdx = P.rho .* uCs .* (-dU1);                 % 一维: -dp/dx = ρU dU/dx (扩散段为正=逆压)
tw1  = 0.5*P.rho*uCore.^2.*cf;
Rsep = (dpdx.*dstar)./max(tw1, 1e-6);
ind  = xc >= G.xSec(3) & xc <= xc(end) - 3*dx;    % 试验段之后才有扩散 (排除出口伪值)
RmaxD = max(Rsep(ind));
[~, iR] = max(Rsep .* double(ind));
if RmaxD < 0.3, sepV = '逆压弱, 无分离风险'; elseif RmaxD < 1, sepV = '逆压显著, 接近分离'; else, sepV = '扩散段末端进入分离风险区 (定性)'; end

% 分离区近壁反向带 (扩散段 R→1 站位, 经验分离剖面): 近壁 35%δ 内线性倒流
RdD  = Rsep(indD);
sepOn = false(nx,1);  sepOn(indD) = RdD > 0.9;             % R>0.9 站位标记分离趋势
if any(sepOn)
    sepA3 = reshape(min(0.42*max(Rsep-0.9,0), 0.42), [nx,1,1]);   % 反向幅值系数
    dr = 0.45*delta3;                                            % 反向带厚度
    revBand = double(d2w < dr & d2w > 0.02*dy);
    U = U - sepA3.*(1 - d2w./dr).^2 .* revBand;
    U(~mask) = 0;  U = min(max(U, -0.25*reshape(uCore,[],1,1)), reshape(uCore,[],1,1)*1.05);
    fprintf('  [分离模型] 扩散段出现近壁反向带 (站点 %d 个)\n', nnz(sepOn));
end

%% ---- [3] 横向速度: 跨站批量 2D 泊松投影 (邻居耦合, GPU 可用时自动加速) ----
% 全部站位 (nx,ny,nz) 同时 Jacobi: 每轮扫全网格, 一次解决逐站循环的开销
useGPU = false;
try
    if license('test','Distrib_Computing_Toolbox') && gpuDeviceCount > 0
        gd = gpuDevice;
        fprintf('  [GPU] 使用 %s (%.1f GB 显存)\n', gd.Name, gd.TotalMemory/2^30);
        useGPU = true;
    end
catch
    useGPU = false;   % 无 Parallel Computing Toolbox / 无 GPU: CPU 批量模式
end
if ~useGPU
    if exist('gpuDeviceCount','file') == 2 && license('test','Distrib_Computing_Toolbox')
        fprintf('  [CPU] PCT 已装但未检测到 NVIDIA CUDA 设备 (MATLAB gpuArray 仅支持 NVIDIA, AMD/Intel 卡不适用), 使用 CPU 批量模式; GPU 加速走 FluidX3D (OpenCL) 路线\n');
    else
        fprintf('  [CPU] 未安装 Parallel Computing Toolbox, 使用 CPU 批量模式\n');
    end
end
g = @(A) A;  if useGPU, g = @gpuArray; end

dUdx = zeros(nx,ny,nz);
dUdx(2:end-1,:,:) = (U(3:end,:,:) - U(1:end-2,:,:))/(2*dx);
dUdx(1,:,:) = (U(2,:,:)-U(1,:,:))/dx;
dUdx(end,:,:)= (U(end,:,:)-U(end-1,:,:))/dx;

src  = g(-dUdx);   src  = src .* mask;
mYp = false(nx,ny,nz); mYp(:,1:end-1,:) = mask(:,2:end,:);   % +y 邻居在洞内
mYm = false(nx,ny,nz); mYm(:,2:end,:)   = mask(:,1:end-1,:);
mZp = false(nx,ny,nz); mZp(:,:,1:end-1) = mask(:,:,2:end);   % +z
mZm = false(nx,ny,nz); mZm(:,:,2:end)   = mask(:,:,1:end-1);
cnt = sum(double(mask), [2 3]);                              % 每站洞内体素数
cnt = max(cnt, 1);
M3  = g(mask);  mYp = g(mYp);  mYm = g(mYm);  mZp = g(mZp);  mZm = g(mZm);
cnt = g(cnt);

% 热启动: 一维伯努利场 (与投影同源, 逐站常数)
phi1d = -cumtrapz(xc, uCore .* gradient(uCore, xc));
phi = g(repmat(reshape(phi1d,[],1,1), [1 ny nz]) .* mask);
phi = single(phi);  srcS = single(src);   % 单精度扫描 (带宽减半), 收敛后转回
if DBGt, tS3 = tic; end

% 跨站批量 Jacobi 迭代: 每轮扫全网格, 一次解决逐站循环的开销
for sw = 1:P.flow.sweeps
    pYp = circshift(phi,[0,-1,0]); pYp(:,end,:) = 0;   % +y 邻居值
    pYm = circshift(phi,[0, 1,0]); pYm(:,1,:)   = 0;
    pZp = circshift(phi,[0,0,-1]); pZp(:,:,end) = 0;
    pZm = circshift(phi,[0,0, 1]); pZm(:,:,1)   = 0;
    nbYp = phi + mYp.*(pYp-phi);   nbYm = phi + mYm.*(pYm-phi);   % 遮蔽体素取 Neumann
    nbZp = phi + mZp.*(pZp-phi);   nbZm = phi + mZm.*(pZm-phi);
    phiN = ((nbYp+nbYm)/dy^2 + (nbZp+nbZm)/dz^2 - srcS) / (2/dy^2+2/dz^2);
    phi  = phiN .* M3;                                          % 纯 Jacobi
    phi  = phi - reshape(sum(phi,[2 3])./cnt, [],1,1);          % Neumann 定常数
end
phi = double(phi);

if DBGt, fprintf('  [dbg] stage 泊松投影: %.1f s\n', toc(tS3)); tS4 = tic; end

% v = dφ/dy, w = dφ/dz (遮蔽邻居取 Neumann → 壁面法向速度为零)
pYp = circshift(phi,[0,-1,0]); pYp(:,end,:) = 0;
pYm = circshift(phi,[0, 1,0]); pYm(:,1,:)   = 0;
pZp = circshift(phi,[0,0,-1]); pZp(:,:,end) = 0;
pZm = circshift(phi,[0,0, 1]); pZm(:,:,1)   = 0;
nbYp = phi + mYp.*(pYp-phi);   nbYm = phi + mYm.*(pYm-phi);
nbZp = phi + mZp.*(pZp-phi);   nbZm = phi + mZm.*(pZm-phi);
V = (nbYp - nbYm)/(2*dy);
W = (nbZp - nbZm)/(2*dz);
if useGPU, V = gather(V);  W = gather(W);  phi = gather(phi); end
V(~mask) = 0;  W(~mask) = 0;

if ~isempty(getenv('WT_DBG'))
    fprintf('  [stage] 投影后 V=[%.2f %.2f] W=[%.2f %.2f]\n', min(V(:)), max(V(:)), min(W(:)), max(W(:)));
end

% 竖直界面浮力分层环流 (Boussinesq, 可选)
if P.gravityOn
    T = 0.5*P.dT_wall*(Z3./B3);                            % 稳定温度分层
    wB = -(9.81/P.T0).*T.*d2w.^2 ./ (2*nuT);               % 层流环流 (浮力-粘性平衡)
    W = W + 0.15*wB;
else
    T = [];   % gravityOn=false 不再分配 300MB 零场 (存档字段保持兼容)
end
%% ---- 派生场 ----
spd  = sqrt(U.^2+V.^2+W.^2);
mach = spd / P.aSound;
rhoRatio = (1 + 0.5*(P.gamma-1)*mach.^2).^(-1/(P.gamma-1));
dUz = (sh3(U,3)-sh3(U,-3))/(2*dz);   dVz = (sh3(V,3)-sh3(V,-3))/(2*dz);
dWy = (sh3(W,2)-sh3(W,-2))/(2*dy);   dWx = (sh3(W,1)-sh3(W,-1))/(2*dx);
vort = sqrt(max(dWy-dVz,0).^2 + max(dUz-dWx,0).^2);        % 涡量幅值(展示)

%% ---- 诊断 ----
q3d  = squeeze(sum(sum(U.*mask,2),3))*dy*dz;
qErr = max(abs(q3d - q3d(1)))/q3d(1);
its  = xc>=G.xSec(3) & xc<=G.xSec(4);
imid = find(its,1)+round(sum(its)/2)-1;
uM = squeeze(U(imid,:,:));  dM = squeeze(d2w(imid,:,:));
core = uM(dM(:) > max(delta(imid), 1.25*dy));   % 核心 = 边界层外一整圈体素
if isempty(core), strat = NaN; else, strat = 1 - min(core)/uCore(imid); end

%% ---- 湍流度 Tu 诊断链 (风扇→蜂窝器→纱网→收缩段→沿程, 一维经验式) ----
% 把 P.sc 的蜂窝器/纱网参数用起来: 估计试验段来流湍流度 (教学风洞典型 0.3~1%)
%  风扇出口 Tu0 ≈ 10% (轴流风扇+导叶后典型值)
%  蜂窝器: 湍流各向同性化 + 纵向衰减, 系数 ≈0.65 (3~6mm 孔经验值)
%  纱网: Dryden 型 Tu2/Tu1 = 1/(1+K), 30 目丝网压降系数 K≈1.0/层
%  收缩段: 拉格朗日无旋衰减 Tu ∝ (U2/U1)^-0.7 ≈ R^-0.7 (Batchelor-Proudman 理论范围 0.5~1)
Tu0    = 0.10;
TuHoney = 0.65;
Kscr   = 1.0;
TuIn   = Tu0 * TuHoney * (1/(1+Kscr))^P.sc.nScreen;        % 稳定段出口 (收缩段入口)
TuCt   = TuIn * P.ct.R^-0.7;                               % 收缩段出口 = 试验段来流
Tu1d   = TuIn .* ((G.a .* G.b)./(G.a(1)*G.b(1))).^0.7;     % 沿程 (面积扩大→Tu 回升)
TuX    = interp1(G.x, Tu1d, xc, 'pchip', 'extrap');

S = struct('U',U,'V',V,'W',W,'spd',spd,'nut',nut,'mach',mach,'rhoRatio',rhoRatio, ...
    'vort',vort,'mask',mask,'x',xc,'y',yc,'z',zc,'d2w',d2w,'delta',delta,'T',T, ...
    'diag', struct('Re_ts',P.V_test*P.ts.L/P.nu, 'x_transition',xTr, ...
    'qErr',qErr, 'stratMid',strat, 'delta_tsEnd',delta(find(its,1,'last')), ...
    'dx',dx,'dy',dy,'dz',dz,'uCore',uCore,'cf',cf,'laminar',laminar,'ks',ks, ...
    'sepX',xc,'sepR',Rsep,'sepMaxD',RmaxD,'sepVerdict',sepV, ...
    'Tu_ts',TuCt, 'Tu_chain',[Tu0 TuIn TuCt]));

% 大场字段转单精度: 存档体积减半, explore_3d 加载更快 (渲染精度足够)
for f = {'U','V','W','spd','nut','mach','rhoRatio','vort','d2w','T'}
    S.(f{1}) = single(S.(f{1}));
end

fprintf(['--- flow3d_solve 完成: 体素 %dx%dx%d (单元 %.1fmm) | Re_ts=%.1e | ' ...
    '转捩 x=%.2fm | 试验段出口 delta=%.1fmm | 中部分层度=%.1f%% | 流量偏差=%.1f%% | uCore=[%.1f %.1f] ---\n'], ...
    nx,ny,nz, dx*1000, S.diag.Re_ts, S.diag.x_transition, ...
    S.diag.delta_tsEnd*1000, strat*100, qErr*100, min(uCore), max(uCore));
fprintf('  [逆压/分离] 扩散段逆压比 R_max=%.2f → %s (R≥1 分离; 当前设计余量 %.0f%%)\n', ...
    RmaxD, sepV, (1-RmaxD)*100);
if ks > 0
    iE = find(xc>=G.xSec(3) & xc<=G.xSec(4), 1, 'last');
    cfSm = 0.0592*max(Re_x(iE),1).^(-0.25);
    fprintf(['  [PLA] 壁面粗糙度 ks=%.2fmm: 粗糙雷诺数 Re_ks=%.0f (>5, 全粗糙区), ' ...
        '试验段末端 cf=%.4f vs 光滑壁 %.4f (+%.0f%%), 边界层全程湍流\n'], ...
        ks*1000, ks*uTau(iE)/P.nu, cf(iE), cfSm, (cf(iE)/cfSm-1)*100);
end
fprintf(['  [Tu 湍流度链] 风扇后 %.1f%% → 蜂窝器 → 纱网×%d → 收缩段 (R=%.0f) ' ...
    '→ 试验段来流 Tu≈%.2f%% (教学风洞典型 0.3~1%%)\n'], ...
    Tu0*100, P.sc.nScreen, P.ct.R, TuCt*100);

% 保存结果 (供 explore_3d 交互浏览器快速加载, 免重复求解)
if DBGt, fprintf('  [dbg] stage 派生场+诊断: %.1f s | 合计(至诊断): %.1f s\n', toc(tS4), toc(tAll)); end
try
    save(fullfile(pwd, 'flow3d_result.mat'), 'S', 'P');
    fprintf('结果已存 flow3d_result.mat\n');
catch e
    fprintf('结果保存跳过: %s\n', e.message);
end
end

%% ==================== 邻居工具 ====================
function M = nbMean(U, mask)
% 6 邻居平均 (壁面邻居按 0 = 无滑移)
mE=false(size(mask)); mE(1:end-1,:,:)=mask(2:end,:,:);
mW=false(size(mask)); mW(2:end,:,:)=mask(1:end-1,:,:);
mN=false(size(mask)); mN(:,1:end-1,:)=mask(:,2:end,:);
mS=false(size(mask)); mS(:,2:end,:)=mask(:,1:end-1,:);
mF=false(size(mask)); mF(:,:,1:end-1)=mask(:,:,2:end);
mB=false(size(mask)); mB(:,:,2:end)=mask(:,:,1:end-1,:);
cnt = double(mE)+double(mW)+double(mN)+double(mS)+double(mF)+double(mB);
sm  = U.*double(mE) + U.*double(mW) + U.*double(mN) + U.*double(mS) + U.*double(mF) + U.*double(mB);
M = sm ./ max(cnt,1);
M(cnt==0) = U(cnt==0);
end

function Lp = lap3(A, mask)
mE=false(size(mask)); mE(1:end-1,:,:)=mask(2:end,:,:);
mW=false(size(mask)); mW(2:end,:,:)=mask(1:end-1,:,:);
mN=false(size(mask)); mN(:,1:end-1,:)=mask(:,2:end,:);
mS=false(size(mask)); mS(:,2:end,:)=mask(:,1:end-1,:);
mF=false(size(mask)); mF(:,:,1:end-1)=mask(:,:,2:end);
mB=false(size(mask)); mB(:,:,2:end)=mask(:,:,1:end-1,:);
Lp = mE.*(sh3(A,1)-A) + mW.*(sh3(A,-1)-A) ...
   + mN.*(sh3(A,2)-A) + mS.*(sh3(A,-2)-A) ...
   + mF.*(sh3(A,3)-A) + mB.*(sh3(A,-3)-A);
end

function B = sh3(A, dir)
switch dir
    case 1,  B = circshift(A,[-1 0 0]); B(end,:,:)=0;
    case -1, B = circshift(A,[ 1 0 0]); B(1,:,:)=0;
    case 2,  B = circshift(A,[0 -1 0]); B(:,end,:)=0;
    case -2, B = circshift(A,[0  1 0]); B(:,1,:)=0;
    case 3,  B = circshift(A,[0 0 -1]); B(:,:,end)=0;
    case -3, B = circshift(A,[0 0  1]); B(:,:,1)=0;
end
end

function B = cshift2(A, k, dim)
if dim == 1
    if k>0, B = circshift(A,[-1 0]); B(end,:)=0; else, B = circshift(A,[1 0]); B(1,:)=0; end
else
    if k>0, B = circshift(A,[0 -1]); B(:,end)=0; else, B = circshift(A,[0 1]); B(:,1)=0; end
end
end

%% ==================== 正激波关系模块 ====================
function out = normalShock(M1, gamma)
% 正激波前后关系 (高速工况/教学演示; 25 m/s 时 M1~0.07, 无激波)
if nargin < 2, gamma = 1.4; end
M1 = max(M1, 1.0001);
M2 = sqrt((1 + (gamma-1)/2*M1.^2) ./ (gamma*M1.^2 - (gamma-1)/2));
p2p1  = 1 + 2*gamma/(gamma+1).*(M1.^2-1);
rho2p1 = (gamma+1).*M1.^2 ./ ((gamma-1).*M1.^2 + 2);
p02p01 = (((gamma+1)*M1.^2/2) ./ (1+(gamma-1)/2*M1.^2)).^(gamma/(gamma-1)) ...
       .* ((gamma+1)/(2*gamma*M1.^2-(gamma-1))).^(1/(gamma-1));
out = struct('M2',M2,'p2p1',p2p1,'rho2p1',rho2p1,'p02p01',p02p01);
end
