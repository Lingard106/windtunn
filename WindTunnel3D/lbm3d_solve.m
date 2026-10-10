function S = lbm3d_solve(P, solid, objMask, grid, wallSolid)
% lbm3d_solve — 涵道 + 测试物 动态风场 LBM 求解器 (D3Q19, TRT 碰撞)
% 域: tunnel_voxelize 的体素涵道; 壁面/测试物 = 全反弹 (fullway bounce-back)
% 动态: 显式时间推进, 捕捉涡脱落/瞬态; 测试物气动力 = 动量交换法
% GPU: 有 Parallel Computing Toolbox + GPU 时自动全程 gpuArray
% 演示雷诺数 P.lbm.Re (真实 Re~1e5 需外部求解器, 见 README 选型)

nx=grid.nx; ny=grid.ny; nz=grid.nz; dx=grid.dx;
fprintf('=== LBM 动态求解: 体素 %dx%dx%d (%.2fmm) ===\n', nx,ny,nz, dx*1000);

%% ---- GPU 检测 ----
useGPU = false;
try
    if license('test','Distrib_Computing_Toolbox') && gpuDeviceCount > 0
        gd = gpuDevice;
        fprintf('  [GPU] %s (%.1f GB)\n', gd.Name, gd.TotalMemory/2^30);
        useGPU = true;
    end
catch
    useGPU = false;
end
g = @(A) A;  if useGPU, g = @gpuArray; end

%% ---- LBM 单位制 ----
U    = P.V_test;                 % 物理来流速度
uLx  = min(P.lbm.uLx, 0.08);     % 晶格来流速度 (Ma≤0.14, 压缩误差<1%; 上限保稳定)
dt   = uLx*dx/U;                 % 晶格时间步 (物理 s)
Lc   = P.ts.W/dx;                % 试验段宽度的晶格数
nu   = max(uLx*Lc/P.lbm.Re, 0.0035);   % 晶格粘度: 目标 Re + 稳定下限 τ+≥0.5105
ReEff = round(uLx*Lc/nu);              % 实际达到的演示雷诺数
tauP = 0.5 + 3*nu;
omP  = 1/tauP;
omM  = 1/(0.5 + (3/16)/(1/omP - 0.5));   % TRT 魔数 Λ=(τ+-½)(τ--½)=3/16 (Ginzburg)
velScale = U/uLx;                % 晶格速度 → 物理速度 (试验段基准)
uInBase = uLx*grid.Ats/grid.Ain; % 入口晶格速度: 面积比通量匹配 -> 试验段精确 U
fprintf('  入口通量匹配: u_in=%.4f (A比 %.2f)\n', uInBase, grid.Ats/grid.Ain);
tScale   = dt;                   % 晶格步 → 物理秒
fScale   = P.rho*dx^4/dt^2;      % 晶格力 → 物理力 (N)
fprintf('  晶格: ν=%.2e, τ+=%.4f | 演示 Re=%d (目标 %d) | dt=%.2es\n', ...
    nu, tauP, ReEff, P.lbm.Re, dt);

%% ---- D3Q19 速度集 ----
e = [0 0 0; 1 0 0;-1 0 0; 0 1 0; 0 -1 0; 0 0 1; 0 0 -1;
     1 1 0;-1 -1 0; 1 -1 0;-1 1 0; 1 0 1;-1 0 -1; 1 0 -1;-1 0 1;
     0 1 1; 0 -1 -1; 0 1 -1; 0 -1 1]';
w  = [1/3, repmat(1/18,1,6), repmat(1/36,1,12)];
opp = ones(1,19);
negE = -e';
[tf, loc] = ismember(negE, e', 'rows');     % 逐行匹配反向速度 (find 线性索引会错位)
if ~all(tf), error('D3Q19 速度集不自洽: 缺少反向速度'); end
opp(2:19) = reshape(loc(2:19), 1, []);

%% ---- 场初始化 ----
f = zeros(19, nx, ny, nz);
uin = uLx;
for k = 1:19
    f(k,:,:,:) = w(k);           % ρ=1 静止平衡 (下面加来流)
end
% 静止平衡 ρ=1, u=0 就是 w(k); 来流通过入口边界条件逐步建立
solidG = g(logical(solid));  objG = g(logical(objMask));
fluidG = ~solidG;
fluidD = double(fluidG);   % 提升出循环: 原每步×每 k 重建 19 次 (~100MB/步 churn)
f = g(f);
ipX = round(0.72*nx);  ipY = round(ny/2);  ipZ = round(nz/2);
probeIdx = [ipX ipY ipZ; ipX ipY-4 ipZ; ipX ipY+4 ipZ];
nPr = size(probeIdx,1);
probeT = zeros(0, nPr+4);      % t, u@各探针, Fx, Fy, Fz
snapEvery = P.lbm.snapEvery;
snapDn = 2;                    % 快照降采样
spSn = zeros(max(1,ceil(nx/snapDn)), max(1,ceil(ny/snapDn)), max(1,ceil(nz/snapDn)), 0);
if useGPU, spSn = g(spSn); end
tSnap = zeros(1,0);
p0Sn = zeros(ceil(nx/2), ceil(nz/2), ceil(P.lbm.steps/snapEvery)+1);  p0q = 0;  % 总压切片时序 (y=中面)
revSum = zeros(nx,1);  cntSum = zeros(nx,1);   % 逆流检测累加器 (逐站)
jm = round(ny/2);

%% ---- 主循环 ----
steps = P.lbm.steps;
tAll = tic;
rampN = max(1, round(P.lbm.rampFrac*steps));   % 来流软启动, 防脉冲失稳
fstar = zeros(size(f));
if useGPU, fstar = g(fstar); end
for n = 1:steps
    uin = uInBase*min(1, n/rampN);              % 来流爬坡 (通量匹配入口)
    % --- 宏观量 (固体上钳制, 防镜像值的除零/发散) ---
    rho = reshape(sum(f,1), [nx,ny,nz]);
    ux  = reshape(sum(f .* reshape(e(1,:),[19,1,1,1]),1), [nx,ny,nz]) ./ rho;
    uy  = reshape(sum(f .* reshape(e(2,:),[19,1,1,1]),1), [nx,ny,nz]) ./ rho;
    uz  = reshape(sum(f .* reshape(e(3,:),[19,1,1,1]),1), [nx,ny,nz]) ./ rho;
    rho(solidG) = 1;  ux(solidG) = 0;  uy(solidG) = 0;  uz(solidG) = 0;
    usq = 1.5*(ux.^2 + uy.^2 + uz.^2);

    % --- 碰撞 (TRT, 仅流体格点更新) ---
    for k = 1:19
        fk  = squeeze(f(k,:,:,:));   fok = squeeze(f(opp(k),:,:,:));
        cu  = e(1,k)*ux + e(2,k)*uy + e(3,k)*uz;
        feqk  = w(k)*rho.*(1 + 3*cu + 4.5*cu.^2 - usq);
        feqo  = w(opp(k))*rho.*(1 - 3*cu + 4.5*cu.^2 - usq);
        new = fk - omP*((fk+fok)/2 - (feqk+feqo)/2) ...
                 - omM*((fk-fok)/2 - (feqk-feqo)/2);
        fstar(k,:,:,:) = fk + fluidD.*(new - fk);
    end

    % --- 测试物气动力 (动量交换, 半程反弹) ---
    Fx = 0; Fy = 0; Fz = 0;
    for k = 2:19
        nbSolid = circshift(objG, reshape(-e(:,k),[1,3]));   % x+e_k 处是测试物
        link = fluidG & nbSolid;
        if ~any(link(:)), continue; end
        fv = squeeze(fstar(k,:,:,:)) .* link;
        Fx = Fx + 2*e(1,k)*sum(fv(:));  Fy = Fy + 2*e(2,k)*sum(fv(:));
        Fz = Fz + 2*e(3,k)*sum(fv(:));
    end

    % --- 迁移 (回绕面清零, 防止环绕污染) ---
    f(1,:,:,:) = squeeze(fstar(1,:,:,:));            % 静止分量原地
    for k = 2:19
        fk = circshift(squeeze(fstar(k,:,:,:)), reshape(e(:,k),[1,3]));
        if e(1,k) > 0, fk(1,:,:) = 0; elseif e(1,k) < 0, fk(end,:,:) = 0; end
        if e(2,k) > 0, fk(:,1,:) = 0; elseif e(2,k) < 0, fk(:,end,:) = 0; end
        if e(3,k) > 0, fk(:,:,1) = 0; elseif e(3,k) < 0, fk(:,:,end) = 0; end
        f(k,:,:,:) = fk;
    end

    % --- 全程反弹: 流后在固体上对交换 (标准 fullway BB) ---
    for k = 2:19
        k2 = opp(k);
        if k < k2
            t1 = f(k, solidG);
            f(k, solidG)  = f(k2, solidG);
            f(k2, solidG) = t1;
        end
    end

    % --- 边界条件 ---
    % 入口 x=1: 来流平衡态 (爬坡, 仅流体格)
    m1 = squeeze(fluidG(1,:,:));
    for k = 1:19
        cui = e(1,k)*uin;
        f(k,1,:,:) = w(k)*(1 + 3*cui + 4.5*cui^2 - 1.5*uin^2).*double(m1) ...
                   + squeeze(f(k,1,:,:)).*double(~m1);
    end
    % 出口 x=nx: 零梯度 + 密度归一 (防质量累积漂移)
    f(:,nx,:,:) = f(:,nx-1,:,:);
    rhoOut = reshape(max(squeeze(rho(nx-1,:,:)), 0.5), 1, ny, nz);
    f(:,nx,:,:) = squeeze(f(:,nx,:,:))./rhoOut;

    % --- 数值守卫: 局部晶格速度超限/密度塌陷则提前收场 (保住已算结果) ---
    if max(abs(ux(fluidG))) > 0.35 || min(rho(fluidG)) < 0.3
        fprintf('  [警告] step %d: 局部失稳 (晶格u=%.3f, rho_min=%.3f), 提前停止 -- 请降低 uLx 或加大粘度\n', ...
            n, max(abs(ux(fluidG))), min(rho(fluidG)));
        break;
    end

    % --- 记录 ---
    if mod(n, 10) == 0
        row = [n*tScale];
        for q = 1:nPr
            row(end+1) = ux(probeIdx(q,1), probeIdx(q,2), probeIdx(q,3))*velScale; %#ok<AGROW>
        end
        probeT(end+1,:) = [row, Fx, Fy, Fz]; %#ok<AGROW>
    end
    if mod(n, snapEvery) == 0
        spd = squeeze(sqrt(ux.^2+uy.^2+uz.^2)) * velScale;
        spSn(:,:,:,end+1) = spd(1:snapDn:end, 1:snapDn:end, 1:snapDn:end); %#ok<AGROW>
        % 总压切片 (y=中面): p0 = cs²(ρ-1) + ½ρ|u|² (晶格) → 物理帕斯卡
        p0L = (rho-1)/3 + 0.5*rho.*(ux.^2+uy.^2+uz.^2);
        pS = squeeze(p0L(:,jm,:)) * (P.rho*dx^2/dt^2);
        p0q = p0q + 1;
        p0Sn(:,:,p0q) = pS(1:2:end, 1:2:end); %#ok<AGROW>
        tSnap(end+1) = n*tScale; %#ok<AGROW>
        % 逆流检测 (分离征兆): 贴壁 2 格内流向速度为负的体素占比 (逐站)
        nbs = false(nx,ny,nz);
        nbs(2:end,:,:)  = nbs(2:end,:,:)  | solidG(1:end-1,:,:);
        nbs(1:end-1,:,:) = nbs(1:end-1,:,:) | solidG(2:end,:,:);
        nbs(:,2:end,:)  = nbs(:,2:end,:)  | solidG(:,1:end-1,:);
        nbs(:,1:end-1,:) = nbs(:,1:end-1,:) | solidG(:,2:end,:);
        nbs(:,:,2:end)  = nbs(:,:,2:end)  | solidG(:,:,1:end-1);
        nbs(:,:,1:end-1) = nbs(:,:,1:end-1) | solidG(:,:,2:end);
        nwF = fluidG & nbs;
        rev = double(ux < -0.02 & nwF);
        revSum = revSum + reshape(sum(rev,[2,3]),[nx,1]); %#ok<AGROW>
        cntSum = cntSum + reshape(sum(double(nwF),[2,3]),[nx,1]); %#ok<AGROW>
    end
    if mod(n, max(1,round(steps/10))) == 0
        fprintf('  step %4d/%d  |u|max=%.2f m/s  Fx=%.3g N\n', ...
            n, steps, max(double(ux(fluidG)))*velScale, Fx*fScale);
    end
end
fprintf('  LBM 演化耗时 %.1f s\n', toc(tAll));

%% ---- 派生场与气动力系数 ----
rho = reshape(sum(f,1), [nx,ny,nz]);
ux  = reshape(sum(f .* reshape(e(1,:),[19,1,1,1]),1), [nx,ny,nz]) ./ rho;
uy  = reshape(sum(f .* reshape(e(2,:),[19,1,1,1]),1), [nx,ny,nz]) ./ rho;
uz  = reshape(sum(f .* reshape(e(3,:),[19,1,1,1]),1), [nx,ny,nz]) ./ rho;
spd = sqrt(ux.^2+uy.^2+uz.^2);

% 涡量 (中心差分, 演示级)
wX = circshift(uz,[0,-1,0]) - circshift(uz,[0,1,0]) - circshift(uy,[0,0,-1]) + circshift(uy,[0,0,1]);
wY = circshift(ux,[0,0,-1]) - circshift(ux,[0,0,1]) - circshift(uz,[1,0,0]) + circshift(uz,[-1,0,0]);
wZ = circshift(uy,[1,0,0]) - circshift(uy,[-1,0,0]) - circshift(ux,[0,-1,0]) + circshift(ux,[0,1,0]);
sc = 1/(2*dx) * velScale;
vort = sqrt(wX.^2 + wY.^2 + wZ.^2) * sc;

% 气动力系数 (时程 + 后 1/4 均值)
Aref = refArea(P);
qS = 0.5*P.rho*U^2*Aref;
Cd_t = probeT(:,end-2)*fScale/qS;
Cl_t = probeT(:,end)*fScale/qS;
Cd = mean(Cd_t(round(end*0.75):end));
Cl = mean(Cl_t(round(end*0.75):end));

% 推力 (轴向力) 末段均值
FxT = probeT(:,end-2);
if isempty(FxT), Thrust = NaN; else, Thrust = mean(FxT(round(end*0.75):end)); end

revFrac = revSum ./ max(cntSum, 1);
[revMax, iRev] = max(revFrac);
S = struct('U',ux*velScale,'V',uy*velScale,'W',uz*velScale,'spd',spd*velScale, ...
    'vort',vort,'rho',rho,'mask',solid,'obj',objMask, ...
    'x',grid.x,'y',grid.y,'z',grid.z,'grid',grid, ...
    'probeT',gather(probeT),'Cd_t',Cd_t,'Cl_t',Cl_t,'Cd',Cd,'Cl',Cl, ...
    'snap',gather(spSn),'tSnap',tSnap,'p0Sn',p0Sn(:,:,1:max(p0q,1)), ...
    'revFrac',revFrac,'revX',grid.x, ...
    ...
    'diag',struct('steps',steps,'dt',dt,'velScale',velScale,'fScale',fScale, ...
    'nu',nu,'Re_demo',ReEff,'useGPU',useGPU,'Aref',Aref,'Thrust',Thrust, ...
    'revMax',revMax,'revX',grid.x(iRev)));

fprintf(['--- lbm3d_solve 完成: 步数 %d | 演示 Re=%d | Cd=%.2f Cl=%.2f | ' ...
    'GPU=%d | 快照 %d 帧 ---\n'], steps, ReEff, Cd, Cl, useGPU, numel(tSnap));
% 逆流/分离检测判定
if revMax > 0.08
    fprintf('  [分离征兆] 贴壁逆流占比峰值 %.0f%% @ x=%.2fm — 扩散段疑似分离, 建议减小扩张角或增设防分离网\n', ...
        revMax*100, grid.x(iRev));
else
    fprintf('  [分离检测] 贴壁逆流占比峰值 %.1f%% — 无明显分离\n', revMax*100);
end
end

%% ==================== 工具 ====================
function A = refArea(P)
switch lower(P.lbm.object)
    case 'sphere',   A = pi*P.lbm.objSize^2/4;
    case 'cylinder', A = P.lbm.objSize*P.ts.H;   % 单位展长×高
    case 'plate',    A = P.lbm.objSize*P.lbm.objSize/3;
    case 'naca',     A = P.lbm.objSize*P.ts.H;
    otherwise,       A = P.lbm.objSize^2;
end
end
