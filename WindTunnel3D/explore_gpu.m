function explore_gpu()
% explore_gpu — GPU(FluidX3D) 动态风场结果浏览器 (与 explore_3d 同款交互)
%   渲染: 洋葱嵌套半透明等值面 (温和变色过渡, 无网格纹) + 少量薄雾层补体感
%   交互: 竖直/水平切片滑块 + x 裁剪 + 场切换 + 时间快照滑块 + 显隐复选框
%   附加: forces.csv 存在时显示测试物 Cd/Cl 时程 (GPU 端动量交换解算)
%   环境变量: WT_T=<t> 指定快照; WT_SNAP=1 导出快照图
delete(findall(0,'Tag','WT3D_gpu'));
delete(findall(0,'Tag','WT3D_gpufig'));
set(groot,'defaultFigureColor','w','defaultAxesColor','w', ...
    'defaultAxesXColor',[.15 .15 .15],'defaultAxesYColor',[.15 .15 .15], ...
    'defaultAxesZColor',[.15 .15 .15],'defaultTextColor',[.15 .15 .15]);

%% ---- 数据加载 ----
R = pwd;
meta = load(fullfile(R,'gpu_meta.mat'));  meta = meta.meta;
P = tunnel_params();
G = tunnel_geometry(P);
[x0, G, isFan, objVal, rotVal] = gpu_fan_compat(G, P, meta);   % 风机耦合模式兼容 (x0/Gext/标记)
if isFan
    rpmC = abs(meta.impOmega)/(2*pi)*60;   % 物理角速度 (rad/s, 带旋向符号) → rpm
    fprintf('  [风机耦合模式] 风源=旋转叶轮 (n=%.0f rpm), 涵道含增压箱(-y进气)→叶轮(绕y轴)→蜗壳(切向+x)→前置扩散段\n', rpmC);
end
fid = fopen(fullfile(R,'windtunnel.raw'),'rb');
rawV = fread(fid, inf, 'uint8=>uint8');  fclose(fid);
nx=meta.nx; ny=meta.ny; nz=meta.nz; dx=meta.dx;
raw = reshape(rawV, [nx ny nz]);
mask = raw >= 1;  objM = raw == objVal;  rotM = rotVal > 0 & raw == rotVal;
xs = x0 + ((1:nx)-0.5)'*dx;
if isfield(meta,'y0'), y0 = meta.y0; else, y0 = -(ny*dx)/2; end   % 域原点 (旧 meta 无 y0/z0 时按对称域回退)
if isfield(meta,'z0'), z0 = meta.z0; else, z0 = -(nz*dx)/2; end
ys = y0 + ((1:ny)-0.5)'*dx;
zs = z0 + ((1:nz)-0.5)'*dx;
% 动态叶轮 (旋转解 meta.rotate=true): 叶轮按真实转角逐帧重建, 不再静态
hasRot = isfield(meta,'rotate') && meta.rotate;
if hasRot
    blR = meta.blade;  flR = meta.fanLayout;
    tbhR = max(blR.tB/2, 0.6*dx);
    wrapDgR = mod(blR.th(2)-blR.th(1), 360);  pitchRadR = 2*pi/blR.Z;  th1Dr = blR.th(1);
    nMxR = ceil((wrapDgR + 360/blR.Z + 4)/(360/blR.Z));
    X2R = reshape(xs - flR.xC, nx, 1);  Z2R = reshape(zs - flR.zF, 1, nz);
    RC2R = sqrt(X2R.^2 + Z2R.^2);  TH2R = mod(atan2(Z2R, X2R), 2*pi);
    yBwR = (flR.yBlade(1)+flR.yBlade(2))/2;  hBwR = abs(flR.yBlade(2)-flR.yBlade(1))/2;
    fprintf('[动态叶轮] 旋转解: 叶轮将按 impOmega*t 真实转角逐帧重建\n');
end
dnE = 2;   % 等值面半分辨率
[X3g, Y3g, Z3g] = ndgrid(xs(1:dnE:end), ys(1:dnE:end), zs(1:dnE:end));   % 静态网格只建一次 (省每次 ~150MB 与重建耗时)
velS = meta.velScale;
hA = max(interp1(G.x,G.a,xs,'pchip','extrap'));

% 可用快照
d = dir(fullfile(R,'fluidx3d','vtk','u_*.vtk'));
if isempty(d), error('无 GPU 结果: 先跑 export_fluidx3d + build_windtunnel.bat + windtunnel.exe'); end
ts = zeros(numel(d),1);
for q = 1:numel(d)
    c = regexp(d(q).name, 'u_.*?(\d+)\.vtk', 'tokens', 'once');
    ts(q) = str2double(c{1});
end
[ts, order] = sort(ts);  d = d(order);
tSel = numel(ts);
if ~isempty(getenv('WT_T')), [~, tSel] = min(abs(ts - str2double(getenv('WT_T')))); end

% 力时程
F = [];
fcsv = fullfile(R,'fluidx3d','forces.csv');
if isfile(fcsv)
    Fc = readmatrix(fcsv);
    if size(Fc,2) >= 4 && all(isfinite(Fc(1,:)))
        Fc(Fc(:,1)==0, :) = [];
        F = Fc;
    end
end

%% ---- 场读取与派生 (按快照缓存) ----
% rho 快照映射 (FluidX3D 命名 rho_<t>rho-<t>.vtk); WT_VTK=u 档无 rho 文件时隐藏该视图
drho = dir(fullfile(R,'fluidx3d','vtk','rho_*.vtk'));
rhoMap = containers.Map('KeyType','double','ValueType','any');
for q = 1:numel(drho)
    tok = regexp(drho(q).name, 'rho_(\d+)rho-(\d+)\.vtk', 'tokens', 'once');
    if ~isempty(tok), rhoMap(str2double(tok{2})) = fullfile(drho(q).folder, drho(q).name); end
end
fldKeys  = {'spd','vort','q','rho','mean','turb'};
fldNames = {'流速 |u| (m/s)','涡量 |ω| (1/s)','Q 准则涡结构 (1/s²)','密度 ρ (kg/m³)', ...
    '时均流速 |Ū| (逐步统计)','湍流度 Tu (%) — 尾流/边界层脉动'};
if isempty(drho)
    kf = strcmp(fldKeys,'rho');  fldKeys(kf) = [];  fldNames(kf) = [];
    fprintf('  [快照] 无 rho_*.vtk (WT_VTK=u 档), 密度视图已隐藏\n');
end
CACHE.key = {};  CACHE.t = [];  CACHE.data = {};  CACHE.clim = {};
MEAN = [];  TURB = [];   % 时均/湍流度缓存 (与快照时刻无关, 只算一次)

function [Fd, clim] = fieldData(key, ti)
    if strcmp(key,'mean') || strcmp(key,'turb')    % 时均/湍流度 (GPU 逐步统计优先)
        if isempty(MEAN)
            meanF = fullfile(R,'fluidx3d','vtk','mean_u.vtk');
            tuF   = fullfile(R,'fluidx3d','vtk','tu.vtk');
            if isfile(meanF) && isfile(tuF)      % 新版: 求解器端每 100 步累加, 无欠采样
                U3 = read_fluidx3d_vtk(meanF)*velS;
                MEAN = sqrt(sum(U3.^2, 4));  MEAN(mask) = NaN;
                TURB = read_fluidx3d_vtk(tuF);  TURB(mask) = NaN;   % Tu (%)
                fprintf('  [时均统计] 已加载 GPU 逐步时均/湍流度 (mean_u.vtk + tu.vtk)\n');
            else                                 % 旧存档回退: 后半段快照平均 (子采样, Tu 为下界)
                iUse = ceil(numel(ts)/2):numel(ts);
                fprintf('  [时均统计] 无 GPU 逐步统计文件, 回退读取后半段 %d 个快照...\n', numel(iUse));
                sU  = zeros(nx,ny,nz,3,'single');  sU2 = zeros(nx,ny,nz,3,'single');
                for q = iUse
                    [U3, ~, ~] = read_fluidx3d_vtk(fullfile(R,'fluidx3d','vtk',d(q).name));
                    U3 = single(U3)*velS;
                    sU = sU + U3;  sU2 = sU2 + U3.^2;
                end
                Um  = sU/numel(iUse);
                ur  = sqrt(max(sU2/numel(iUse) - Um.^2, 0));   % 各分量脉动 RMS
                MEAN = sqrt(sum(Um.^2, 4));
                TURB = sqrt(sum(ur.^2, 4))/max(meta.V_test,eps)*100;  % Tu (%) 三分量合成
                MEAN(mask) = NaN;  TURB(mask) = NaN;
                fprintf('  [时均统计] 完成: |U|mean=[%.1f %.1f] m/s, Tu 最大 %.1f%% (快照子采样, Tu 为下界)\n', ...
                    min(MEAN(:)), max(MEAN(:)), max(TURB(:)));
            end
        end
        if strcmp(key,'mean'), Fd = MEAN; clim = [0, meta.V_test*1.3];
        else,                   Fd = TURB;
            v = TURB(1:7:numel(TURB));  v = sort(v(isfinite(v)));
            clim = [0, v(min(numel(v),round(0.995*numel(v))))];
            if clim(2)<=clim(1), clim(2)=clim(1)+1; end
        end
        return;
    end
    for q = 1:numel(CACHE.key)
        if strcmp(CACHE.key{q}, key) && CACHE.t(q) == ts(ti)
            Fd = CACHE.data{q};  clim = CACHE.clim{q};
            if q < numel(CACHE.key)   % LRU: 命中移到末尾
                CACHE.key{end+1} = CACHE.key{q};   CACHE.t(end+1) = CACHE.t(q);
                CACHE.data{end+1} = CACHE.data{q}; CACHE.clim{end+1} = CACHE.clim{q};
                CACHE.key(q) = [];  CACHE.t(q) = [];  CACHE.data(q) = [];  CACHE.clim(q) = [];
            end
            return;
        end
    end
    [U3, ~, ~] = read_fluidx3d_vtk(fullfile(R,'fluidx3d','vtk',d(ti).name));
    U3 = U3*velS;
    switch key
        case 'spd'
            Fd = sqrt(U3(:,:,:,1).^2+U3(:,:,:,2).^2+U3(:,:,:,3).^2);
            clim = [0, meta.V_test*1.3];
        case 'vort'
            Ux=U3(:,:,:,1); Uy=U3(:,:,:,2); Uz=U3(:,:,:,3);  sc = velS/dx;
            wX = (shift3(Uz,2)-shift3(Uz,-2) - shift3(Uy,3)+shift3(Uy,-3))/(2*dx);
            wY = (shift3(Ux,3)-shift3(Ux,-3) - shift3(Uz,1)+shift3(Uz,-1))/(2*dx);
            wZ = (shift3(Uy,1)-shift3(Uy,-1) - shift3(Ux,2)+shift3(Ux,-2))/(2*dx);
            Fd = sqrt(wX.^2 + wY.^2 + wZ.^2) * sc;
            s7 = 1:7:numel(Fd);                       % 1/7 抽样 (~1M 点) 估 99.5% 分位, 省全量 sort
            v = Fd(s7);  v = v(isfinite(v) & ~mask(s7));  v = sort(v);
            clim = [0, v(min(numel(v),round(0.995*numel(v))))];
            if clim(2)<=clim(1), clim(2)=clim(1)+1; end
        case 'q'   % Q = ½(||Ω||²F − ||S||²F): 涡旋转强于剪切变形的区域 (涡结构识别)
            Ux=U3(:,:,:,1); Uy=U3(:,:,:,2); Uz=U3(:,:,:,3);
            [Ux_x,Ux_y,Ux_z] = gradient(Ux, dx, dx, dx);
            [Uy_x,Uy_y,Uy_z] = gradient(Uy, dx, dx, dx);
            [Uz_x,Uz_y,Uz_z] = gradient(Uz, dx, dx, dx);
            o12=(Ux_y-Uy_x)/2; o13=(Ux_z-Uz_x)/2; o23=(Uy_z-Uz_y)/2;
            s11=Ux_x; s22=Uy_y; s33=Uz_z;
            s12=(Ux_y+Uy_x)/2; s13=(Ux_z+Uz_x)/2; s23=(Uy_z+Uz_y)/2;
            Fd = (o12.^2+o13.^2+o23.^2) - 0.5*(s11.^2+s22.^2+s33.^2) - (s12.^2+s13.^2+s23.^2);
            s7 = 1:7:numel(Fd);
            v = Fd(s7);  v = v(isfinite(v) & v>0 & ~mask(s7));  v = sort(v);
            clim = [0, v(min(numel(v),round(0.995*numel(v))))];
            if isempty(v) || clim(2)<=clim(1), clim = [0, 1]; end
        case 'rho'
            rF = rhoMap(ts(ti));   % rho_*.vtk 按时刻映射 (初始化时建立; 无文件时该视图整体隐藏)
            Fd = read_fluidx3d_vtk(rF)*meta.rho;
            clim = [meta.rho*0.96, meta.rho*1.06];
    end
    Fd(mask) = NaN;
    CACHE.key{end+1} = key;  CACHE.t(end+1) = ts(ti);
    CACHE.data{end+1} = Fd;  CACHE.clim{end+1} = clim;
    while numel(CACHE.key) > 4   % LRU 上限: 单条 ~200MB, 防止回放/拖动吃满内存
        CACHE.key(1) = [];  CACHE.t(1) = [];  CACHE.data(1) = [];  CACHE.clim(1) = [];
    end
end

%% ---- 图形骨架 ----
fig = figure('Name','GPU 风场浏览器 (FluidX3D 结果)', 'Color','w', ...
    'Position',[50 40 1300 740], 'ToolBar','figure', 'NumberTitle','off', 'Tag','WT3D_gpu');
ax = axes('Parent',fig, 'Position',[0.0 0.02 0.74 0.94]); hold(ax,'on');
camlight(ax,'headlight');  material(ax,'dull');  lighting(ax,'gouraud'); % 等值面深度可读

[XgY, YgY] = meshgrid(xs, ys);
[XgZ, ZgZ] = meshgrid(xs, zs);
[YgX, ZgX] = meshgrid(ys, zs);
XgY0 = XgY;  XgZ0 = XgZ;

% 涵道内壁薄壳 (分段淡色) + 四角棱线
wallCols = [0.85 0.65 0.25; 0.30 0.50 0.85; 0.90 0.45 0.20; 0.55 0.55 0.60];
Ek = linspace(-1,1,25);
hWall = gobjects(1,0);
xSav = [0, G.xSec(2), G.xSec(3), G.xSec(4), G.xSec(5)];
for k = 1:4
    mkw = xs >= max(xSav(k),x0-1e-9) & xs <= min(xSav(k+1),xs(end))+1e-9;
    if ~any(mkw), continue; end
    [XXw, EEw] = ndgrid(xs(mkw), Ek);
    ak = interp1(G.x,G.a,xs(mkw),'pchip','extrap');
    bk = interp1(G.x,G.b,xs(mkw),'pchip','extrap');
    fc = wallCols(k,:);
    hWall(end+1) = surf(ax, XXw, EEw.*ak(:),  repmat(bk(:),1,size(XXw,2)), 'FaceColor',fc,'FaceAlpha',0.10,'EdgeColor','none');
    hWall(end+1) = surf(ax, XXw, EEw.*ak(:), -repmat(bk(:),1,size(XXw,2)), 'FaceColor',fc,'FaceAlpha',0.10,'EdgeColor','none');
    hWall(end+1) = surf(ax, XXw,  repmat(ak(:),1,size(XXw,2)), EEw.*bk(:), 'FaceColor',fc,'FaceAlpha',0.10,'EdgeColor','none');
    hWall(end+1) = surf(ax, XXw, -repmat(ak(:),1,size(XXw,2)), EEw.*bk(:), 'FaceColor',fc,'FaceAlpha',0.10,'EdgeColor','none');
end
for sy_ = [-1 1], for sz_ = [-1 1]
    plot3(ax, xs, sy_*interp1(G.x,G.a,xs,'pchip','extrap'), ...
              sz_*interp1(G.x,G.b,xs,'pchip','extrap'), '-', 'Color',[0.2 0.2 0.25],'LineWidth',0.8);
end, end

% 薄雾层 (少量 z 向切片, 低透明度补体感; 不再堆三向网格)
kzBg = unique(round(linspace(2, nz-1, 14)));
hBgZ = gobjects(numel(kzBg),1);
for q = 1:numel(kzBg)
    hBgZ(q) = surf(ax, XgY, YgY, zs(kzBg(q))*ones(size(XgY)), NaN(size(XgY)), ...
        'FaceColor','interp','FaceAlpha','interp','AlphaData',zeros(size(XgY)),'EdgeColor','none');
end

% 测试物实体
hObj = [];  cpV = [];  cpRGB = [];   % cpV: 表面气动载荷 |F|/q∞ (每顶点), cpRGB: 对应颜色
if any(objM(:))
    pv = isosurface(single(objM), 0.5);
    if ~isempty(pv.vertices)
        % [R2026a 修复] 无网格 isosurface 顶点列序 = (第2维,第1维,第3维): col2→x, col1→y (旧版按 col1→x 映射, x/y 互换导致物体转置)
        c1 = pv.vertices(:,1);  c2 = pv.vertices(:,2);
        pv.vertices(:,1) = x0 + (c2-0.5)*dx;
        pv.vertices(:,2) = y0 + (c1-0.5)*dx;
        pv.vertices(:,3) = z0 + (pv.vertices(:,3)-0.5)*dx;
        hObj = patch(ax, pv, 'FaceColor',[0.2 0.5 0.9], 'FaceAlpha',0.95, 'EdgeColor','none');
        % 表面气动载荷: 最新 F_*.vtk (体素级力矢量) → 最近邻采样到顶点 → turbo 着色
        try
            dF = dir(fullfile(R,'fluidx3d','vtk','F_*.vtk'));
            if ~isempty(dF)
                [~, imx] = max([dF.datenum]);
                F3 = read_fluidx3d_vtk(fullfile(dF(imx).folder, dF(imx).name));
                if isequal(size(F3),[nx,ny,nz,3])
                    Fsc = meta.rho*dx^4/meta.dt^2;              % 晶格力 → N
                    qInf = 0.5*meta.rho*meta.V_test^2;          % 动压 (Pa)
                    loadN = sqrt(sum(F3.^2,4))*Fsc/dx^2/qInf;   % 局部载荷系数 |F|/(A·q∞)
                    loadN(~objM) = NaN;
                    vv = pv.vertices;
                    ci = min(nx,max(1,round((vv(:,1)-x0)/dx)+1));
                    cj = min(ny,max(1,round((vv(:,2)-y0)/dx)+1));
                    ck = min(nz,max(1,round((vv(:,3)-z0)/dx)+1));
                    idx = sub2ind([nx,ny,nz],ci,cj,ck);
                    cpV = loadN(idx);
                    if any(isnan(cpV))                          % 非物体侧顶点取最近物体格点
                        [fi,fj,fk] = ind2sub([nx,ny,nz], find(objM));
                        for w = find(isnan(cpV))'
                            d2 = (fi-ci(w)).^2+(fj-cj(w)).^2+(fk-ck(w)).^2;
                            [~, jm] = min(d2);
                            cpV(w) = loadN(fi(jm),fj(jm),fk(jm));
                        end
                    end
                    cpV(~isfinite(cpV)) = 0;
                    srt = sort(cpV);  cpHi = srt(min(numel(srt),round(0.95*numel(srt))));
                    if cpHi<=0, cpHi = max(cpV)+eps; end
                    cmT = turbo(256);
                    cpF = mean(cpV(pv.faces),2);                 % 逐面均值 (FV patch 支持 per-face truecolor)
                    itf = min(1,max(0,cpF/cpHi));
                    cpRGB = cmT(max(1,min(256,round(itf*255)+1)),:);
                    fprintf('[表面载荷] 已加载 %s — %d 面着色 |F|/q∞ (勾选"表面载荷"显示)\n', dF(imx).name, size(cpRGB,1));
                end
            end
        catch
            cpV = [];  cpRGB = [];
        end
    end
end

% 风机耦合模式: 旋转叶轮 (raw=2) 红色实体 + 蜗壳螺旋壁/毂/封盘 (外壳不再裸奔)
% [动态叶轮] 旋转解 (meta.rotate) 时 updateRot 会按真实转角重建并覆盖本静态 patch (try-catch 兜底)
hRot = [];  hRotPatch = [];  B2cur = [];   % B2cur: 旋转解当前相位下的叶片 xz 截面 (切片窗共享)
if rotVal > 0 && any(rotM(:))
    pvR = isosurface(single(rotM), 0.5);
    if ~isempty(pvR.vertices)
        % [R2026a 修复] 同上: 顶点列序 (第2维,第1维,第3维) — 旧映射 x/y 互换导致叶轮转置成"轴向沿风洞"
        c1 = pvR.vertices(:,1);  c2 = pvR.vertices(:,2);
        pvR.vertices(:,1) = x0 + (c2-0.5)*dx;
        pvR.vertices(:,2) = y0 + (c1-0.5)*dx;
        pvR.vertices(:,3) = z0 + (pvR.vertices(:,3)-0.5)*dx;
        hRot = patch(ax, pvR, 'FaceColor',[0.85 0.25 0.1], 'FaceAlpha',0.9, 'EdgeColor','none');
    end
    % 蜗壳螺旋壁 (xoz 平面阿基米德线, 腔 y 中面) + 毂圆 + 蜗壳外廓 — 深灰外壳
    try
        bl = meta.blade;  vf = meta.vol;  fl = meta.fanLayout;  sSp = sign(meta.impSpin);
        thV = linspace(0, 359, 120);
        rS = vf.rTongue + vf.a*mod(thV - 300, 360)*pi/180;   % Θ 系螺旋 (舌部 300°, 沿旋转方向生长)
        ph = sSp*thV;                                        % 物理角 (spinS=-1 整机 z 镜像)
        yMid = mean(fl.yBlade);
        plot3(ax, fl.xC + rS.*cosd(ph), yMid*ones(size(thV)), fl.zF + rS.*sind(ph), '-', ...
            'Color',[0.25 0.25 0.3], 'LineWidth', 2.5);
        % 毂圆 (y=const 平面, r1): 叶轮两端 + 下游侧板
        thC = linspace(0, 360, 60);
        for ye = [fl.yBlade(1), fl.yBlade(2), fl.ySideDn(2)]
            plot3(ax, fl.xC + bl.r1*cosd(thC), ye*ones(size(thC)), fl.zF + bl.r1*sind(thC), '-', ...
                'Color',[0.3 0.3 0.35], 'LineWidth', 1.2);
        end
        % 蜗壳外廓圆 (rEnd)
        rO = vf.rEnd + 0.008;
        plot3(ax, fl.xC + rO*cosd(ph), yMid*ones(size(thV)), fl.zF + rO*sind(ph), ':', ...
            'Color',[0.25 0.25 0.3], 'LineWidth', 1.4);
    catch
    end
    fprintf('[风机耦合] 旋转叶轮已渲染 (红, %d 体素) — 蜗壳螺旋壁/毂已叠加\n', nnz(rotM));
end

% 洋葱嵌套等值面 (温和变色过渡的主体) — applyField 内生成
hOnion = [];

% 截面位置标尺 (青=y, 品红=z; 仅轮廓线不遮挡)
xL = xs(1);  xR = xs(end);
hGv = plot3(ax, [xL xR xR xL xL], [0 0 0 0 0], hA*[-1 -1 1 1 -1], '--', ...
    'Color',[0 0.75 0.9], 'LineWidth', 1.4);
hGh = plot3(ax, [xL xR xR xL xL], hA*[-1 -1 1 1 -1], [0 0 0 0 0], '--', ...
    'Color',[0.9 0.2 0.8], 'LineWidth', 1.4);
hTy = plot3(ax, [xL-0.015 xL], [0 0], [-hA -hA], '-', 'Color',[0 0.75 0.9], 'LineWidth', 2.5);
hTz = plot3(ax, [xL-0.015 xL], [-hA -hA], [0 0], '-', 'Color',[0.9 0.2 0.8], 'LineWidth', 2.5);

axis(ax,'equal'); xlim(ax,[x0-0.02 xs(end)+0.02]);
ylim(ax,[-hA-0.04 hA+0.04]); zlim(ax,[-hA-0.04 hA+0.16]);
view(ax,-42,16); grid(ax,'on');
colorbar(ax); colormap(ax,turbo);
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');

%% ---- 控制面板 ----
pnl = uipanel(fig,'Units','normalized','Position',[0.75 0.01 0.24 0.975],'Title','GPU 风场控制台');
mkLbl = @(s,y) uicontrol(pnl,'Style','text','Units','normalized', ...
    'Position',[0.03 y 0.94 0.03],'HorizontalAlignment','left','String',s,'FontSize',8);
mkSld = @(y,lo,hi,val,cb) uicontrol(pnl,'Style','slider','Units','normalized', ...
    'Position',[0.04 y 0.92 0.04],'Min',lo,'Max',hi,'Value',val,'Callback',cb);

uicontrol(pnl,'Style','text','Units','normalized','Position',[0.03 0.955 0.94 0.028],'HorizontalAlignment','left','String','显示场','FontSize',8);
hPop = uicontrol(pnl,'Style','popup','Units','normalized','Position',[0.04 0.917 0.9 0.04],'String',fldNames,'Callback',@onField);

mkLbl('时间快照 (拖动回放 GPU 瞬态)', 0.878);
hSt = mkSld(0.843, 1, numel(ts), tSel, @onTime);

mkLbl('竖直截面 y (m) — 场图见下方窗口', 0.795);
hSy = mkSld(0.760, -hA*0.98, hA*0.98, 0, @onSlice);
mkLbl('水平截面 z (m)', 0.712);
hSz = mkSld(0.677, -hA*0.98, hA*0.98, 0, @onSlice);
mkLbl('x 裁剪 (隐藏下游, 看进洞内)', 0.629);
hSx = mkSld(0.594, xs(2), xs(end), xs(end), @onSlice);

hCbBg   = mkChk(0.04,0.545,'薄雾层', true,  @(s) setVis(hBgZ,  s));
hCbOn   = mkChk(0.52,0.545,'洋葱体渲染', true,  @(s) setVis(hOnion, s));
hCbObj  = mkChk(0.04,0.505,'测试物', true,  @(s) setVis(hObj,  s));
hCbWall = mkChk(0.52,0.505,'涵道壁', true,  @(s) setVis(hWall, s));
hCbCp   = mkChk(0.04,0.455,'表面载荷 |F|/q∞', false, @onCp);
uicontrol(pnl,'Style','pushbutton','Units','normalized','Position',[0.04 0.40 0.9 0.045], ...
    'String','重置视角','Callback',@onView);

%% ---- 截面分析窗 (独立 2D 窗口) ----
figS = figure('Name','GPU 截面分析窗 — 拖动滑块实时刷新', 'Color','w', ...
    'Position',[100 80 1180 480], 'NumberTitle','off', 'Tag','WT3D_gpufig');
axV = axes('Parent',figS,'Position',[0.06 0.14 0.36 0.74]); hold(axV,'on');
axH = axes('Parent',figS,'Position',[0.55 0.14 0.36 0.74]); hold(axH,'on');
hImgV = [];  hImgH = [];
hWallV = [];  hWallH = [];   % 墙型线 (静态, 只建一次)
hObjV = [];  hObjH = [];  hRotV = [];  hRotH = [];   % 物体/叶轮轮廓 (每次删旧建新, 防句柄泄漏)
xlabel(axV,'x (mm)'); ylabel(axV,'z (mm)');  title(axV,'竖直截面 (y=const)','FontSize',10);
xlabel(axH,'x (mm)'); ylabel(axH,'y (mm)');  title(axH,'水平截面 (z=const)','FontSize',10);
colormap(axV,turbo);  colormap(axH,turbo);
colorbar(axV);  colorbar(axH);

% 气动力面板
mkLbl('测试物气动特性 (GPU 动量交换)', 0.35);
hAxF = axes('Parent',pnl, 'Units','normalized', 'Position',[0.06 0.16 0.88 0.18]);

% 气动力面板: forces.csv → Cd/Cl 时程 (动量交换解算, qRef=½ρV²A_ref)
if ~isempty(F)
    tF = F(:,1)*meta.dt;
    qRef = 0.5*meta.rho*meta.V_test^2*meta.Aref;
    Cd_t = F(:,2)/qRef;  Cl_t = F(:,4)/qRef;   % 升力取 Fz (竖直向); Fy 为展向 (圆柱轴向)
    plot(hAxF, tF, smoothdata(Cd_t,'movmean',9), 'b-', 'LineWidth',1.2); hold(hAxF,'on');
    plot(hAxF, tF, smoothdata(Cl_t,'movmean',9), 'r-', 'LineWidth',1.0);
    grid(hAxF,'on');
    iS = max(1, floor(numel(tF)/2));       % 后半段均值 (剔除初始瞬态)
    if isfield(meta,'ReLattice'), ReStr = sprintf('%d (LES 有效高 Re)', meta.ReLattice);
    else, ReStr = '旧存档未记录'; end
    objStr = '被测物';  if isfield(meta,'objType'), objStr = sprintf('被测物[%s]', meta.objType); end
    blk = meta.Aref/(P.ts.W*P.ts.H)*100;
    title(hAxF, sprintf('%s C_d=%.3f  C_l=%.3f (后半段均值) | A_{ref}=%.1f cm² (迎风投影), 堵塞 %.1f%% | 名义晶格 Re=%s', ...
        objStr, mean(Cd_t(iS:end)), mean(Cl_t(iS:end)), meta.Aref*1e4, blk, ReStr), ...
        'FontSize',8);
    legend(hAxF, 'C_d','C_l', 'Location','northeast', 'FontSize',7);
    xlabel(hAxF,'t (s)'); ylabel(hAxF,'系数');
    % 涡脱频率: Cl 后半段 FFT 峰 → St (d=测试物特征尺寸; 旧存档回退参考面积当量径)
    if isfield(meta,'objSize'), dRef = meta.objSize; else, dRef = sqrt(4*meta.Aref/pi); end
    try
        iW = max(2, floor(numel(tF)/2)):numel(tF);
        ClD = detrend(Cl_t(iW));  ClD = ClD - mean(ClD);
        NF = 2^nextpow2(numel(ClD));
        wn = 0.5 - 0.5*cos(2*pi*(1:numel(ClD))'/max(numel(ClD),1));   % Hann 窗
        SP = abs(fft(ClD.*wn, NF));
        fr = (0:NF-1)'/(numel(ClD)*mean(diff(tF(iW))));
        half = 2:floor(NF/2);
        [~, ipk] = max(SP(half));
        text(hAxF, 0.98, 0.82, sprintf('f_{peak}=%.0f Hz → St=f·d/U≈%.3f (d=%.0fmm)', ...
            fr(half(ipk)), fr(half(ipk))*dRef/meta.V_test, dRef*1e3), ...
            'Units','normalized', 'HorizontalAlignment','right', 'FontSize',7);
    catch
    end
end

% 初始显示场 (可被 WT_FIELD 环境变量覆盖, 用于无头快照导出)
initKey = 'spd';
v = getenv('WT_FIELD');
if ~isempty(v) && any(strcmp(fldKeys, v)), initKey = v; end
set(hPop, 'Value', find(strcmp(fldKeys, initKey)));

ST = struct('key',initKey,'tSel',tSel,'xcut',xs(end));
applyField();   % 内部已调 onSlice (去双重刷新)

% 表面载荷着色强制开启 (WT_CP=1, 无头快照/动画导出用)
if ~isempty(cpV) && ~isempty(getenv('WT_CP'))
    set(hCbCp,'Value',1);  onCp(hCbCp);
end

% 时序动画导出 (WT_MOV=1 全部快照 / WT_MOV=<N> 每 N 个取 1) → 主视图 + 切片窗双留档
if ~isempty(getenv('WT_MOV'))
    stv = str2double(getenv('WT_MOV'));
    stride = 1;  if isfinite(stv) && stv>1, stride = round(stv); end
    idxs = 1:stride:numel(ts);
    tmpD = fullfile(tempdir,'wtgpu_mov');  if ~exist(tmpD,'dir'), mkdir(tmpD); end
    fprintf('[动画] 渲染 %d 帧 (主视图 + 切片窗)...\n', numel(idxs));
    frames  = cell(1,numel(idxs));
    framesS = cell(1,numel(idxs));
    for w = 1:numel(idxs)
        set(hSt,'Value',idxs(w));  onTime();  drawnow;
        tmpf = fullfile(tmpD,sprintf('f%03d.png',idxs(w)));
        try,   exportapp(fig, tmpf);          % 图窗含 UI 组件, 必须用 exportapp
        catch, print(fig, tmpf, '-dpng', '-r80');
        end
        frames{w} = imread(tmpf);
        if isgraphics(figS)                   % 切片分析窗同步留档
            tmpfS = fullfile(tmpD,sprintf('s%03d.png',idxs(w)));
            try,   exportapp(figS, tmpfS); catch, end
            if isfile(tmpfS), framesS{w} = imread(tmpfS); end
        end
    end
    fr4 = max(2, round(numel(idxs)/6));
    writeVidFrames(frames, fullfile(R,'gpu_timelapse.mp4'), fr4);
    if ~isempty(framesS{1})
        writeVidFrames(framesS, fullfile(R,'gpu_timelapse_slices.mp4'), fr4);
    end
    set(hSt,'Value',tSel);  onTime();   % 恢复原快照
end

if ~isempty(getenv('WT_SNAP'))
    set(hSy,'Value',-0.04);  set(hSz,'Value',0.03);
    xv = str2double(getenv('WT_XCUT'));
    if isfinite(xv), set(hSx,'Value',min(xs(end),max(xs(2),xv))); end
    if strcmp(getenv('WT_ISO'),'0'), set(hCbOn,'Value',0); setVis(hOnion,hCbOn); end   % 关等值面 (表面载荷特写)
    if strcmp(getenv('WT_FOG'),'0'), set(hCbBg,'Value',0); setVis(hBgZ,hCbBg); end
    onSlice();  drawnow;
    exportapp(fig,  fullfile(R,'gpu_browser_snapshot.png'));
    exportapp(figS, fullfile(R,'gpu_slice_window.png'));
    fprintf('快照已存 gpu_browser_snapshot.png / gpu_slice_window.png\n');
end
fprintf('--- explore_gpu 就绪 (%d 个时间快照, GPU 体素 %dx%dx%d) ---\n', numel(ts), nx, ny, nz);

%% ==================== 嵌套函数 ====================
    function applyField()
        [Fd, clim] = fieldData(ST.key, ST.tSel);
        caxis(ax, clim);
        % 风机耦合模式: 实际风速 ≠ 设计风速 → 基准自适应实测场 (否则异色水平够不着)
        Veff = meta.V_test;
        if isFan && any(strcmp(ST.key, {'spd','mean'}))
            Wp = Fd(1:4:end, 1:4:end, 1:4:end);
            Veff = max(prctile(Wp(Wp > 0.05*meta.V_test), 99), 1);
        end
        % 薄雾层: 速度类场在均匀核心完全透明 (消除"发光盒子"), 只留尾流/边界层体感
        for q = 1:numel(kzBg)
            Cd = Fd(:,:,kzBg(q))';
            Al = 0.03 + 0.12*min(max((Cd-clim(1))/diff(clim),0),1);
            if any(strcmp(ST.key,{'spd','mean'}))
                Al(Cd > 0.85*Veff) = 0;
            end
            Al(isnan(Cd)) = 0;  Cd(isnan(Cd)) = 0;
            set(hBgZ(q), 'CData',Cd, 'AlphaData',Al);
        end
        % 等值面: 按场类型选策略 (速度类=0.90V 尾流/边界层壳, 洞内一眼看穿;
        % 涡量/湍流度=尾流结构嵌套壳), 半分辨率生成
        if ~isempty(hOnion), delete(hOnion); hOnion = []; end
        Wd = Fd(1:dnE:end, 1:dnE:end, 1:dnE:end);  Wd(isnan(Wd)) = 0;   % 坐标网格 X3g/Y3g/Z3g 静态, 初始化时建一次
        switch ST.key
            case {'spd','mean'}, lv = [0.90 0.55 0.30]*Veff;  alv = [0.35 0.18 0.08];   % [改进] 加低速层: 叶轮近场旋流 (360rpm 演示解叶尖仅 8.7m/s) 也被包裹, 红蓝重叠可见
            case {'vort','q'},   lv = clim(1)+[0.35 0.60 0.85]*diff(clim);  alv = [0.10 0.22 0.38];
            case 'turb',         lv = clim(1)+[0.06 0.18 0.40]*diff(clim);  alv = [0.10 0.22 0.38];
            otherwise,           lv = clim(1)+[0.50 0.90]*diff(clim);       alv = [0.15 0.30]; % rho
        end
        cm = turbo(256);
        if any(strcmp(ST.key, {'spd','mean'}))
            cm = [linspace(0.25,0.45,256)', linspace(0.55,0.70,256)', linspace(0.95,0.85,256)']; % [修复] 速度类等值面用蓝色系 — 原 turbo 高端色与红色叶轮撞色, 被误读为"第二组横放叶轮"
        end
        hOnion = gobjects(0);
        for q = 1:numel(lv)
            fvv = isosurface(X3g, Y3g, Z3g, Wd, lv(q));
            if isempty(fvv.vertices), continue; end
            ci = max(1,min(256,round((lv(q)-clim(1))/diff(clim)*255)+1));
            po = patch(ax, fvv, 'FaceColor',cm(ci,:), 'FaceAlpha',alv(q), 'EdgeColor','none');
            hOnion(end+1) = po; %#ok<AGROW>
        end
        onSlice();
        drawnow;
    end

    function updateRot(ti, jj, kk)   % [动态叶轮] 旋转解: 按真实转角 impOmega*t 重建叶片体素 (失败时静态叶轮兜底)
        if ~hasRot, return; end
        th = meta.impOmega*meta.dt*ts(ti);
        B2 = false(nx, nz);
        for mm = 0:nMxR
            pm = mod(TH2R - th, pitchRadR) + mm*pitchRadR;
            x2 = RC2R.*cos(pm);  z2 = RC2R.*sin(pm);
            dd = sqrt((x2-blR.C(1)).^2 + (z2-blR.C(2)).^2);
            a2 = mod(atan2d(z2-blR.C(2), x2-blR.C(1)) - th1Dr, 360);
            B2 = B2 | (abs(dd-blR.Ra)<=tbhR & a2<=wrapDgR);
        end
        B2 = squeeze(B2);
        B2cur = B2;   % 共享给切片窗 (旋转相位下的 xz 截面)
        jB = find(abs(ys - yBwR) <= hBwR);
        if numel(jB) < 2, return; end
        rotT = repmat(reshape(B2, nx, 1, nz), [1, numel(jB), 1]);   % 先 3D 再 repmat (R2026a repmat 尾维 1 有合并行为)
        pvR = isosurface(single(rotT), 0.5);
        if isempty(pvR.vertices), return; end
        v = pvR.vertices;
        % [R2026a] isosurface 顶点列序 = (第2维, 第1维, 第3维): col1=y层索引, col2=x索引, col3=z索引
        v(:,1) = xs(max(1,min(nx,round(v(:,2)))));
        v(:,2) = ys(jB(max(1,min(numel(jB),round(v(:,1))))));
        v(:,3) = zs(max(1,min(nz,round(v(:,3)))));
        pvR.vertices = v;
        if isempty(hRotPatch) || ~isgraphics(hRotPatch)
            hRotPatch = patch(ax, pvR, 'FaceColor',[0.95 0.45 0.15], 'FaceAlpha',0.98, 'EdgeColor','none');
            % 形状参照物: 转轴粗线(绿, 沿 y 穿叶轮心) + 毂圆柱(深灰) + 风洞轴(蓝, 沿 x) — 让"轮子"可读
            thC = linspace(0, 360, 60);
            for ye = [flR.yBlade(1), flR.yBlade(2)]
                plot3(ax, flR.xC + blR.r1*cosd(thC), ye*ones(size(thC)), flR.zF + blR.r1*sind(thC), '-', ...
                    'Color',[0.25 0.25 0.3], 'LineWidth', 1.2);
            end
            plot3(ax, [flR.xC flR.xC], [ys(1) ys(end)], [flR.zF flR.zF], '-', ...
                'Color',[0 0.55 0], 'LineWidth', 3);
            text(ax, flR.xC, ys(end), flR.zF+0.04, ' 转轴y', 'Color',[0 0.5 0], 'FontWeight','bold');
            plot3(ax, [meta.x0 meta.x0+nx*dx], [0 0], [0 0], '-', ...
                'Color',[0 0.3 0.9], 'LineWidth', 2.5);
            text(ax, meta.x0+nx*dx-0.35, 0, -0.07, '风洞轴x', 'Color',[0 0.3 0.9], 'FontWeight','bold');
        else
            set(hRotPatch, 'Vertices', pvR.vertices, 'Faces', pvR.faces);
        end
    end

    function onSlice(~,~)
        ST.xcut = get(hSx,'Value');
        [Fd, clim] = fieldData(ST.key, ST.tSel);
        [~, jj] = min(abs(ys - get(hSy,'Value')));
        [~, kk] = min(abs(zs - get(hSz,'Value')));
        yv = ys(jj);  zv = zs(kk);
        updateRot(ST.tSel, jj, kk);   % [动态叶轮] 旋转解: 同时刷新主视图 3D 与切片窗 2D 轮廓
        % --- 主视图: 标尺轮廓/游标 ---
        set(hGv, 'YData', yv*[1 1 1 1 1]);
        set(hGh, 'ZData', zv*[1 1 1 1 1]);
        set(hTy, 'YData', [yv yv]);
        set(hTz, 'ZData', [zv zv]);
        set(hGv, 'XData', [xL ST.xcut ST.xcut xL xL]);
        set(hGh, 'XData', [xL ST.xcut ST.xcut xL xL]);
        % --- x 裁剪 ---
        Xg = XgY0;  Xg(Xg > ST.xcut) = NaN;
        for q = 1:numel(kzBg), set(hBgZ(q),'XData',Xg); end
        for q = 1:numel(hOnion)
            Vd = get(hOnion(q),'Vertices');  Vd(Vd(:,1) > ST.xcut,1) = NaN;
            set(hOnion(q),'Vertices',Vd);
        end
        title(ax, sprintf('GPU 动态风场 | t = %.3f s (步 %d/%d) | y=%.0fmm z=%.0fmm x裁=%.0fmm', ...
            ts(ST.tSel)*meta.dt, ts(ST.tSel), ts(end), yv*1e3, zv*1e3, ST.xcut*1e3));
        % --- 截面窗刷新 ---
        updateSliceFig(Fd, clim, jj, kk, yv, zv);
        drawnow limitrate;
    end

    function updateSliceFig(Fd, clim, jj, kk, yv, zv)
        Cv = squeeze(Fd(:,jj,:));                % (nx,nz)
        Ch = squeeze(Fd(:,:,kk));                % (nx,ny)
        if isempty(hImgV) || ~isgraphics(hImgV)
            hImgV = imagesc(axV, xs*1000, zs*1000, Cv'); axis(axV,'xy');
        else
            set(hImgV, 'CData', Cv');
        end
        caxis(axV, clim);
        hold(axV,'on');
        if isempty(hWallV) || ~all(isgraphics(hWallV))   % 墙型线只建一次 (型线静态)
            bI = interp1(G.x, G.a, xs, 'pchip', 'extrap');
            hWallV = plot(axV, xs*1000,  bI*1000, 'k-', xs*1000, -bI*1000, 'k-', 'LineWidth', 1.2);
        end
        if ~isempty(hObjV)
            delete(hObjV(isgraphics(hObjV)));  hObjV = [];
        end
        if any(objM(:,jj,:))
            hObjV = contour(axV, xs*1000, zs*1000, double(squeeze(objM(:,jj,:)))', [0.5 0.5], 'b-', 'LineWidth', 1.4);
        end
        if ~isempty(hRotV)
            delete(hRotV(isgraphics(hRotV)));  hRotV = [];
        end
        % [动态叶轮] 旋转解: 用当前相位的叶片截面替代静态 rotM
        rotV = rotM(:,jj,:);  rotH = rotM(:,:,kk);
        if hasRot && ~isempty(B2cur)
            rotV = B2cur;                          % (nx,nz) 竖直切片 (叶轮中面 y=jj 处各 z 同形)
            rotH = repmat(B2cur(:,kk), 1, ny);     % (nx,ny) 水平切片
        end
        if any(rotV(:), 'all')
            hRotV = contour(axV, xs*1000, zs*1000, double(rotV)', [0.5 0.5], 'r-', 'LineWidth', 1.2);
        end
        xlim(axV, [x0 ST.xcut]*1000);  ylim(axV, [-hA hA]*1000);
        title(axV, sprintf('竖直截面 y=%.0fmm (x×z) — 蓝线=测试物 红线=叶轮', yv*1000), 'FontSize', 10);
        if isempty(hImgH) || ~isgraphics(hImgH)
            hImgH = imagesc(axH, xs*1000, ys*1000, Ch'); axis(axH,'xy');
        else
            set(hImgH, 'CData', Ch');
        end
        caxis(axH, clim);
        hold(axH,'on');
        if isempty(hWallH) || ~all(isgraphics(hWallH))   % 墙型线只建一次 (型线静态)
            aI = interp1(G.x, G.a, xs, 'pchip', 'extrap');
            hWallH = plot(axH, xs*1000,  aI*1000, 'k-', xs*1000, -aI*1000, 'k-', 'LineWidth', 1.2);
        end
        if ~isempty(hObjH)
            delete(hObjH(isgraphics(hObjH)));  hObjH = [];
        end
        if any(objM(:,:,kk))
            hObjH = contour(axH, xs*1000, ys*1000, double(squeeze(objM(:,:,kk))), [0.5 0.5], 'b-', 'LineWidth', 1.4);
        end
        if ~isempty(hRotH)
            delete(hRotH(isgraphics(hRotH)));  hRotH = [];
        end
        if rotVal > 0 && any(rotH(:), 'all')
            hRotH = contour(axH, xs*1000, ys*1000, double(rotH), [0.5 0.5], 'r-', 'LineWidth', 1.2);
        end
        xlim(axH, [x0 ST.xcut]*1000);  ylim(axH, [-hA hA]*1000);
        title(axH, sprintf('水平截面 z=%.0fmm (x×y) — 蓝线=测试物 红线=叶轮', zv*1000), 'FontSize', 10);
        drawnow limitrate;
    end

    function onTime(~,~)
        ST.tSel = round(get(hSt,'Value'));
        if ST.tSel<1, ST.tSel=1; end
        if ST.tSel>numel(ts), ST.tSel=numel(ts); end
        applyField();   % 内部已调 onSlice (去双重刷新)
    end
    function onField(~,~)
        ST.key = fldKeys{get(hPop,'Value')};
        applyField();   % 内部已调 onSlice (去双重刷新)
    end
    function onView(~,~), view(ax,-42,16); end
    function onCp(s)
        if isempty(cpV), set(s,'Value',0); return; end
        if get(s,'Value'), set(hObj,'FaceColor','flat','FaceVertexCData',cpRGB);
        else, set(hObj,'FaceColor',[0.2 0.5 0.9],'FaceVertexCData',[]);
        end
        drawnow limitrate;
    end
    function h = mkChk(x,y,lbl,on,cb)
        h = uicontrol(pnl,'Style','checkbox','Units','normalized','Position',[x y 0.46 0.035], ...
            'String',lbl,'FontSize',8,'Value',on,'Callback',cb);
    end
    function setVis(h,s)
        if get(s,'Value'), v='on'; else, v='off'; end
        set(h,'Visible',v);
    end
end

%% ==================== 工具 ====================
function writeVidFrames(frames, fpath, fps)
% 帧序列 → mp4 (失败回退 GIF); frames: cell of HxWx3 uint8
ok = false;  vw = [];
try
    vw = VideoWriter(fpath,'MPEG-4');  vw.FrameRate = fps;
    open(vw);  writeVideo(vw, cat(4,frames{:}));  close(vw);  ok = true;
catch e4
    try, close(vw); catch, end
    [~,base] = fileparts(fpath);
    gifF = fullfile(fileparts(fpath), [base '.gif']);
    for w = 1:numel(frames)
        if ~isempty(frames{w})
            [Xw, mw] = rgb2ind(frames{w}, 64);
            if w==1, imwrite(Xw, mw, gifF, 'gif','Loopcount',0,'DelayTime',0.25);
            else,    imwrite(Xw, mw, gifF, 'gif','WriteMode','append','DelayTime',0.25);
            end
        end
    end
    fprintf('[动画] mp4 失败 (%s), 回退 GIF: %s\n', e4.message, gifF);
end
if ok, fprintf('[动画] 已存 %s (%d 帧)\n', fpath, numel(frames)); end
end

function B = shift3(A, dim)
switch dim
    case 1,  B = circshift(A,[-1 0 0]); B(end,:,:)=[];
    case -1, B = circshift(A,[ 1 0 0]); B(1,:,:)=[]; B = cat(1,B,B(end,:,:));
    case 2,  B = circshift(A,[0 -1 0]); B(:,end,:)=[];
    case -2, B = circshift(A,[0  1 0]); B(:,1,:)=[]; B = cat(2,B,B(:,end,:));
    case 3,  B = circshift(A,[0 0 -1]); B(:,:,end)=[];
    case -3, B = circshift(A,[0 0  1]); B(:,:,1)=[]; B = cat(3,B,B(:,:,end));
end
sA = size(A);
if ~isequal(size(B), sA)
    switch abs(dim)
        case 1, B = cat(1, B, B(end,:,:));
        case 2, B = cat(2, B, B(:,end,:));
        case 3, B = cat(3, B, B(:,:,end));
    end
end
end
