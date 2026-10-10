function explore_3d()
% explore_3d — 风场交互浏览器: 主视图=涵道体填充+截面位置标尺; 截面场图单开窗口
%   主视图: 涵道内壁 + 风场体填充 + (PLA边界层壳) + 截面平面轮廓标尺 (不遮挡)
%   截面窗: 竖直截面 (x×z @当前y) 与 水平截面 (x×y @当前z) 两幅场图, 拖动实时刷新
%   环境变量: WT_RESOLVE=1 强制重解; WT_QUICK=1 低分辨率; WT_SNAP=1 导出快照
if isempty(getappdata(0,'WT3D_INIT'))
    set(groot,'defaultFigureColor','w','defaultAxesColor','w', ...
        'defaultAxesXColor',[.15 .15 .15],'defaultAxesYColor',[.15 .15 .15], ...
        'defaultAxesZColor',[.15 .15 .15],'defaultTextColor',[.15 .15 .15]);
    setappdata(0,'WT3D_INIT',true);
end
% 只关闭旧版浏览器窗口 (主视图+截面窗), 不动 visualize_3d 的分析图
delete(findall(0,'Tag','WT3D_browser'));
delete(findall(0,'Tag','WT3D_slicefig'));

%% ---- 数据: 用户改参重解 > 存档 > 默认求解 ----
Pusr = getappdata(0,'WT3D_USERP');
if ~isempty(Pusr)
    P = Pusr;  rmappdata(0,'WT3D_USERP');
    fprintf('参数已更新, 重新求解...\n');
    flow3d_solve(P, tunnel_geometry(P));
    D = load(fullfile(pwd,'flow3d_result.mat'));  S = D.S;  P = D.P;
elseif exist(fullfile(pwd,'flow3d_result.mat'),'file') && isempty(getenv('WT_RESOLVE'))
    D = load(fullfile(pwd,'flow3d_result.mat'));
    S = D.S;  P = D.P;
    fprintf('已加载 flow3d_result.mat (体素 %dx%dx%d)\n', numel(S.x), numel(S.y), numel(S.z));
else
    P = tunnel_params();
    if ~isempty(getenv('WT_QUICK'))                      % 秒级冒烟档: 单元 ×6
        P.flow.nx=max(60,round(P.flow.nx/6)); P.flow.ny=max(32,round(P.flow.ny/6)); P.flow.nz=max(32,round(P.flow.nz/6));
    elseif ~isempty(getenv('WT_MID'))                    % 中档: 单元 ×2
        P.flow.nx=max(60,round(P.flow.nx/2)); P.flow.ny=max(32,round(P.flow.ny/2)); P.flow.nz=max(32,round(P.flow.nz/2));
    end   % 不设环境变量 = 默认最高档 (tunnel_params 按目标单元尺寸派生)
    flow3d_solve(P, tunnel_geometry(P));
    D = load(fullfile(pwd,'flow3d_result.mat'));  S = D.S;  P = D.P;
end
G = tunnel_geometry(P);
hA = max(G.a);  hB = max(G.b);

%% ---- 场字典 ----
fldKeys  = {'spd','vort','nutn','mach'};
fldNames = {'流速 (m/s)','涡量 (1/s)','涡粘比 νt/ν','马赫数'};
FD = struct();
FD.spd  = {S.spd,      '流速 (m/s)',   [0, P.V_test*1.25]};
FD.vort = {S.vort,     '涡量 (1/s)',   pctRange(S.vort, S.mask, 0.995)};
FD.nutn = {S.nut/P.nu, '涡粘比 νt/ν',  pctRange(S.nut/P.nu, S.mask, 0.99)};
FD.mach = {S.mach,     '马赫数',       [0, double(max(S.mach(:)))*1.1+1e-6]};
cmap = turbo(256);

%% ---- 主视图 (3D) ----
fig = figure('Name','风场主视图 — 鼠标拖拽旋转', 'Color','w', ...
    'Position',[50 300 1150 620], 'ToolBar','figure', 'NumberTitle','off', ...
    'Tag','WT3D_browser');
ax = axes('Parent',fig, 'Position',[0.02 0.02 0.80 0.94]); hold(ax,'on');

[XgY, YgY] = meshgrid(S.x, S.y);
XgY0 = XgY;

% 背景体填充: 三向正交薄层
[XgZ2, ZgZ2] = meshgrid(S.x, S.z);
[YgX, ZgX]   = meshgrid(S.y, S.z);
kzBg = unique(round(linspace(1, numel(S.z), 40)));
kyBg = unique(round(linspace(2, numel(S.y)-1, 30)));
kxBg = unique(round(linspace(3, numel(S.x)-2, 26)));
hBgZ = gobjects(numel(kzBg),1);  hBgY = gobjects(numel(kyBg),1);  hBgX = gobjects(numel(kxBg),1);
for q = 1:numel(kzBg)
    hBgZ(q) = surf(ax, XgY, YgY, S.z(kzBg(q))*ones(size(XgY)), NaN(size(XgY)), ...
        'FaceColor','interp','FaceAlpha','interp','AlphaData',zeros(size(XgY)),'EdgeColor','none');
end
for q = 1:numel(kyBg)
    hBgY(q) = surf(ax, XgZ2, S.y(kyBg(q))*ones(size(XgZ2)), ZgZ2, NaN(size(XgZ2)), ...
        'FaceColor','interp','FaceAlpha','interp','AlphaData',zeros(size(XgZ2)),'EdgeColor','none');
end
for q = 1:numel(kxBg)
    hBgX(q) = surf(ax, S.x(kxBg(q))*ones(size(YgX)), YgX, ZgX, NaN(size(YgX)), ...
        'FaceColor','interp','FaceAlpha','interp','AlphaData',zeros(size(YgX)),'EdgeColor','none');
end
hAllBg = [hBgZ; hBgY; hBgX];

% 涵道壁 (淡) + 四角棱线
hWall = gobjects(1,0);
Ek = linspace(-1,1,13);
for k = 1:4
    mkw = S.x >= G.xSec(k)-1e-9 & S.x <= G.xSec(k+1)+1e-9;
    [XXw, EEw] = ndgrid(S.x(mkw), Ek);
    ak = interp1(G.x,G.a,S.x(mkw),'pchip','extrap');
    bk = interp1(G.x,G.b,S.x(mkw),'pchip','extrap');
    fc = [0.5 0.55 0.6];
    hWall(end+1) = surf(ax, XXw, EEw.*ak(:),  repmat(bk(:),1,size(XXw,2)), 'FaceColor',fc,'FaceAlpha',0.08,'EdgeColor','none');
    hWall(end+1) = surf(ax, XXw, EEw.*ak(:), -repmat(bk(:),1,size(XXw,2)), 'FaceColor',fc,'FaceAlpha',0.08,'EdgeColor','none');
    hWall(end+1) = surf(ax, XXw,  repmat(ak(:),1,size(XXw,2)), EEw.*bk(:), 'FaceColor',fc,'FaceAlpha',0.08,'EdgeColor','none');
    hWall(end+1) = surf(ax, XXw, -repmat(ak(:),1,size(XXw,2)), EEw.*bk(:), 'FaceColor',fc,'FaceAlpha',0.08,'EdgeColor','none');
end
for sy_ = [-1 1], for sz_ = [-1 1]
    plot3(ax, G.x, sy_*G.a, sz_*G.b, '-', 'Color',[0.2 0.2 0.25], 'LineWidth', 0.8);
end, end

% PLA 边界层壳层
hBL = gobjects(1,0);
if isfield(P,'wall') && P.wall.on
    aI = interp1(G.x,G.a,S.x,'pchip','extrap');
    bI = interp1(G.x,G.b,S.x,'pchip','extrap');
    offA = max(aI(:) - S.delta(:), 0.002);
    offB = max(bI(:) - S.delta(:), 0.002);
    sub = 1:4:numel(S.x);
    [XXs, EEs] = ndgrid(S.x(sub), Ek);
    aS = offA(sub);  bS = offB(sub);
    blc = [0.98 0.30 0.08];
    hBL(end+1) = surf(ax, XXs, EEs.*aS(:),  repmat(bS(:),1,size(XXs,2)), 'FaceColor',blc,'FaceAlpha',0.18,'EdgeColor','none');
    hBL(end+1) = surf(ax, XXs, EEs.*aS(:), -repmat(bS(:),1,size(XXs,2)), 'FaceColor',blc,'FaceAlpha',0.18,'EdgeColor','none');
    hBL(end+1) = surf(ax, XXs,  repmat(aS(:),1,size(XXs,2)), EEs.*bS(:), 'FaceColor',blc,'FaceAlpha',0.18,'EdgeColor','none');
    hBL(end+1) = surf(ax, XXs, -repmat(aS(:),1,size(XXs,2)), EEs.*bS(:), 'FaceColor',blc,'FaceAlpha',0.18,'EdgeColor','none');
    iTs = find(S.x<=G.xSec(4),1,'last');
    text(ax, S.x(iTs), 0, hB*1.26, sprintf('PLA 边界层 (末端 \\delta=%.1fmm)', S.delta(iTs)*1000), ...
        'Color',[0.75 0.15 0.05], 'FontSize',9, 'HorizontalAlignment','right');
end

% 试验段核心流等值面
its = S.x >= G.xSec(3)-1e-9 & S.x <= G.xSec(4)+1e-9;
[Xt, Yt, Zt] = ndgrid(S.x(its), S.y, S.z);
spdTs = S.spd;  spdTs(~S.mask) = 0;
hIso = [];
fv = isosurface(Xt, Yt, Zt, spdTs(its,:,:), P.V_test*0.88);
if ~isempty(fv.vertices)
    hIso = patch(ax, fv, 'FaceColor',[0.95 0.55 0.1], 'FaceAlpha',0.22, 'EdgeColor','none');
end

% 截面位置标尺: 仅平面轮廓线 (青=y 截面, 品红=z 截面), 不遮挡模型
xL = S.x(1);  xR = S.x(end);
hGv = plot3(ax, [xL xR xR xL xL], [0 0 0 0 0], hB*[-1 -1 1 1 -1], '-', ...
    'Color',[0 0.75 0.9], 'LineWidth', 1.4, 'LineStyle','--');
hGh = plot3(ax, [xL xR xR xL xL], hA*[-1 -1 1 1 -1], [0 0 0 0 0], '-', ...
    'Color',[0.9 0.2 0.8], 'LineWidth', 1.4, 'LineStyle','--');
% 尺寸线游标 (y 尺寸线/ z 尺寸线上的游标刻度)
hTy = plot3(ax, [xL-0.02 xL], [0 0], [-hB -hB], '-', 'Color',[0 0.75 0.9], 'LineWidth', 2.5);
hTz = plot3(ax, [xL-0.02 xL], [-hA -hA], [0 0], '-', 'Color',[0.9 0.2 0.8], 'LineWidth', 2.5);

axis(ax,'equal'); xlim(ax,[-0.03 G.x(end)+0.03]);
ylim(ax,[-hA-0.05 hA+0.05]); zlim(ax,[-hB-0.05 hB+0.18]);
view(ax,-42,16); grid(ax,'on');
colorbar(ax); colormap(ax,cmap);
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');

%% ---- 截面分析窗 (独立 2D 窗口) ----
figS = figure('Name','截面分析窗 — 拖动主窗滑块实时刷新', 'Color','w', ...
    'Position',[80 80 1180 480], 'NumberTitle','off', 'Tag','WT3D_slicefig');
axV = axes('Parent',figS,'Position',[0.06 0.14 0.36 0.74]); hold(axV,'on');
axH = axes('Parent',figS,'Position',[0.55 0.14 0.36 0.74]); hold(axH,'on');
hImgV = [];  hImgH = [];
xlabel(axV,'x (mm)'); ylabel(axV,'z (mm)');  title(axV,'竖直截面 (y=const)','FontSize',10);
xlabel(axH,'x (mm)'); ylabel(axH,'y (mm)');  title(axH,'水平截面 (z=const)','FontSize',10);
colormap(axV,cmap); colormap(axH,cmap);
cbV = colorbar(axV);  cbH = colorbar(axH);
cbV.Label.String = '场值';  cbH.Label.String = '场值';
legend(axV, 'off');  legend(axH, 'off');

%% ---- 控制面板 ----
pnl = uipanel(fig,'Units','normalized','Position',[0.83 0.01 0.16 0.975],'Title','切片控制');
mkLbl = @(s,y) uicontrol(pnl,'Style','text','Units','normalized', ...
    'Position',[0.03 y 0.94 0.03],'HorizontalAlignment','left','String',s,'FontSize',8);
mkSld = @(y,lo,hi,val,cb) uicontrol(pnl,'Style','slider','Units','normalized', ...
    'Position',[0.04 y 0.92 0.04],'Min',lo,'Max',hi,'Value',val,'Callback',cb);

cfgKeys  = {'balanced','lowTurb','compact','aniso'};
cfgNames = {'均衡 balanced','低湍流 lowTurb','紧凑 compact','矩形 aniso'};
uicontrol(pnl,'Style','text','Units','normalized','Position',[0.03 0.955 0.94 0.028], ...
    'HorizontalAlignment','left','String','构型预设 (切换即重解)','FontSize',8);
hCfg = uicontrol(pnl,'Style','popup','Units','normalized','Position',[0.04 0.918 0.9 0.04], ...
    'String',cfgNames,'Value',max(1,find(strcmp(cfgKeys,P.config),1)),'Callback',@onCfg);

edLbls = {'V∞ (m/s)','收缩比 R','收缩段长 (mm)','试验段宽 (mm)','试验段高 (mm)', ...
          '试验段长 (mm)','扩散段出口宽 (mm)','扩散段出口高 (mm)','PLA 粗糙度 ks (mm)'};
edVal  = {P.V_test, P.ct.R, P.ct.L*1e3, P.ts.W*1e3, P.ts.H*1e3, P.ts.L*1e3, ...
          P.df.W_out*1e3, P.df.H_out*1e3, P.wall.ks*1e6};
uicontrol(pnl,'Style','text','Units','normalized','Position',[0.03 0.878 0.94 0.026], ...
    'HorizontalAlignment','left','String','参数 (改后点重解)','FontSize',8);
hEd = gobjects(numel(edLbls),1);
for q = 1:numel(edLbls)
    yy = 0.845 - (q-1)*0.039;
    uicontrol(pnl,'Style','text','Units','normalized','Position',[0.03 yy 0.52 0.03], ...
        'HorizontalAlignment','left','String',edLbls{q},'FontSize',8);
    hEd(q) = uicontrol(pnl,'Style','edit','Units','normalized','Position',[0.57 yy 0.40 0.034], ...
        'String',num2str(edVal{q},'%.4g'));
end
uicontrol(pnl,'Style','pushbutton','Units','normalized','Position',[0.04 0.472 0.9 0.04], ...
    'String','重新求解 (应用参数)','FontWeight','bold','Callback',@onResolve);

uicontrol(pnl,'Style','text','Units','normalized','Position',[0.03 0.432 0.94 0.026], ...
    'HorizontalAlignment','left','String','显示场','FontSize',8);
hPop = uicontrol(pnl,'Style','popup','Units','normalized','Position',[0.04 0.397 0.9 0.038], ...
    'String',fldNames,'Callback',@onField);

mkLbl('竖直截面 y (m)', 0.357);
hSy = mkSld(0.322, -hA*0.98, hA*0.98, 0, @onSlice);
mkLbl('水平截面 z (m)', 0.282);
hSz = mkSld(0.247, -hB*0.98, hB*0.98, 0, @onSlice);
mkLbl('x 裁剪 (隐藏下游)', 0.207);
hSx = mkSld(0.172, S.x(2), S.x(end), S.x(end), @onSlice);

hCbBg   = mkChk(0.04,0.130,'体填充',      true,  @(s) setVis(hAllBg, s));
hCbIso  = mkChk(0.52,0.130,'等值面',      true,  @(s) setVis(hIso,   s));
hCbWall = mkChk(0.04,0.095,'涵道壁',      true,  @(s) setVis(hWall,  s));
hCbBL   = mkChk(0.52,0.095,'PLA 边界层',  true,  @(s) setVis(hBL,    s));
hCbGv   = mkChk(0.04,0.060,'y 截面标尺',  true,  @(s) setVis([hGv,hTy], s));
hCbGh   = mkChk(0.52,0.060,'z 截面标尺',  true,  @(s) setVis([hGh,hTz], s));
hCbSV   = mkChk(0.04,0.025,'截面窗:竖直', true,  @(s) setVisibleAx(axV, s));
hCbSH   = mkChk(0.52,0.025,'截面窗:水平', true,  @(s) setVisibleAx(axH, s));

uicontrol(pnl,'Style','pushbutton','Units','normalized','Position',[0.04 0.155 0.9 0.035], ...
    'String','重置视角','Callback',@onView);

ST.key = 'spd';  ST.xcut = S.x(end);
applyField();  onSlice();
title(ax, sprintf('风场体渲染 [%s 构型 | 体素 %dx%dx%d] — 截面场图见独立窗口', ...
    P.config, numel(S.x), numel(S.y), numel(S.z)));

if ~isempty(getenv('WT_SNAP'))
    set(hSy,'Value',-0.04);  set(hSz,'Value',0.03);  set(hSx,'Value',G.xSec(4));
    onSlice();  drawnow;
    exportapp(fig,  fullfile(pwd,'explore_snapshot.png'));
    exportapp(figS, fullfile(pwd,'slice_window.png'));
    fprintf('快照已存 explore_snapshot.png / slice_window.png\n');
end
fprintf('--- explore_3d 就绪 ---\n');

%% ==================== 嵌套函数 ====================
    function applyField()
        F = FD.(ST.key);
        Fd = F{1};  Fd(~S.mask) = NaN;
        clim = F{3};
        caxis(ax, clim);
        for q = 1:numel(kzBg)
            Cd = Fd(:,:,kzBg(q))';
            Al = 0.07 + 0.28*min(max((Cd-clim(1))/diff(clim), 0), 1);
            Al(isnan(Cd)) = 0;  Cd(isnan(Cd)) = 0;
            set(hBgZ(q), 'CData', Cd, 'AlphaData', Al);
        end
        for q = 1:numel(kyBg)
            Cd = permute(Fd(:,kyBg(q),:),[3 1 2]);
            Al = 0.07 + 0.28*min(max((Cd-clim(1))/diff(clim), 0), 1);
            Al(isnan(Cd)) = 0;  Cd(isnan(Cd)) = 0;
            set(hBgY(q), 'CData', Cd, 'AlphaData', Al);
        end
        for q = 1:numel(kxBg)
            Cd = squeeze(Fd(kxBg(q),:,:))';
            Al = 0.07 + 0.28*min(max((Cd-clim(1))/diff(clim), 0), 1);
            Al(isnan(Cd)) = 0;  Cd(isnan(Cd)) = 0;
            set(hBgX(q), 'CData', Cd, 'AlphaData', Al);
        end
        cbV.Label.String = F{2};  cbH.Label.String = F{2};
        onSlice();
    end

    function onSlice(~,~)
        ST.xcut = get(hSx,'Value');
        F = FD.(ST.key);  Fd = F{1};  Fd(~S.mask) = NaN;
        clim = F{3};
        [~, jj] = min(abs(S.y - get(hSy,'Value')));
        [~, kk] = min(abs(zs0() - get(hSz,'Value')));
        yv = S.y(jj);  zv = S.z(kk);
        % --- 主视图: 标尺轮廓随滑块移动 ---
        set(hGv, 'YData', yv*[1 1 1 1 1]);
        set(hGh, 'ZData', zv*[1 1 1 1 1]);
        set(hTy, 'YData', [yv yv]);
        set(hTz, 'ZData', [zv zv]);
        % x 裁剪: 体填充 XData 置 NaN + 标尺轮廓缩短
        Xg = XgY0;  Xg(Xg > ST.xcut) = NaN;
        for q = 1:numel(kzBg), set(hBgZ(q), 'XData', Xg); end
        for q = 1:numel(kyBg)
            Xd = XgZ2;  Xd(Xd > ST.xcut) = NaN;
            set(hBgY(q), 'XData', Xd);
        end
        for q = 1:numel(kxBg)
            if S.x(kxBg(q)) <= ST.xcut, v = 'on'; else, v = 'off'; end
            set(hBgX(q), 'Visible', v);
        end
        set(hGv, 'XData', [xL ST.xcut ST.xcut xL xL]);
        set(hGh, 'XData', [xL ST.xcut ST.xcut xL xL]);
        if ~isempty(hIso)
            Vd = get(hIso,'Vertices');  Vd(Vd(:,1) > ST.xcut,1) = NaN;
            set(hIso,'Vertices',Vd);
        end
        % --- 截面窗刷新 ---
        updateSliceFig(Fd, clim, jj, kk, yv, zv);
        % --- 状态文字 ---
        uVs = squeeze(S.spd(:,jj,:));  mVs = squeeze(S.mask(:,jj,:));
        dVs = squeeze(S.d2w(:,jj,:));
        its2 = S.x >= G.xSec(3) & S.x <= G.xSec(4);
        mkc = its2(:) & mVs & dVs > max(S.delta(find(its2,1)+round(sum(its2)/2)-1), 1.25*S.diag.dz);
        if any(mkc(:))
            cu = uVs(mkc);
            sStr = sprintf('试验段核心 %.1f~%.1f m/s (均值 %.1f)', min(cu), max(cu), mean(cu));
        else
            sStr = '该截面在洞外';
        end
        title(ax, sprintf('%s | y=%.0fmm, z=%.0fmm, x裁剪=%.0fmm | %s', ...
            F{2}, yv*1000, zv*1000, ST.xcut*1000, sStr));
        drawnow limitrate;
    end

    function zc0 = zs0(), zc0 = S.z; end

    function updateSliceFig(Fd, clim, jj, kk, yv, zv)
        figure(figS);
        % 竖直截面 (x × z @ y=jj)
        C = squeeze(Fd(:,jj,:));                         % (nx,nz)
        if isempty(hImgV)
            hImgV = imagesc(axV, S.x*1000, S.z*1000, C'); axis(axV,'xy');
        else
            set(hImgV, 'CData', C');
        end
        caxis(axV, clim);
        bI = interp1(G.x, G.b, S.x, 'pchip', 'extrap');
        hold(axV,'on');
        plot(axV, S.x*1000,  bI*1000, 'k-', S.x*1000, -bI*1000, 'k-', 'LineWidth', 1.2);
        xlim(axV, [S.x(1) ST.xcut]*1000);  ylim(axV, [-hB hB]*1000);
        % 水平截面 (x × y @ z=kk)
        C2 = Fd(:,:,kk);                                 % (nx,ny)
        if isempty(hImgH)
            hImgH = imagesc(axH, S.x*1000, S.y*1000, C2'); axis(axH,'xy');
        else
            set(hImgH, 'CData', C2');
        end
        caxis(axH, clim);
        aI = interp1(G.x, G.a, S.x, 'pchip', 'extrap');
        hold(axH,'on');
        plot(axH, S.x*1000,  aI*1000, 'k-', S.x*1000, -aI*1000, 'k-', 'LineWidth', 1.2);
        xlim(axH, [S.x(1) ST.xcut]*1000);  ylim(axH, [-hA hA]*1000);
        title(axV, sprintf('竖直截面 y=%.0fmm (x×z)', yv*1000), 'FontSize', 10);
        title(axH, sprintf('水平截面 z=%.0fmm (x×y)', zv*1000), 'FontSize', 10);
        drawnow limitrate;
    end

    function onView(~,~)
        view(ax,-42,16);
    end

    function onCfg(~,~)
        Pc = tunnel_params(cfgKeys{get(hCfg,'Value')});
        setappdata(0,'WT3D_USERP',Pc);
        delete(fig);  delete(figS);
        explore_3d();
    end

    function onResolve(~,~)
        Pc = P;
        Pc.V_test   = rdE(hEd(1), Pc.V_test);
        Pc.ct.R     = max(2, rdE(hEd(2), Pc.ct.R));
        Pc.ct.L     = max(0.1, rdE(hEd(3), Pc.ct.L*1e3))/1e3;
        Pc.ts.W     = rdE(hEd(4), Pc.ts.W*1e3)/1e3;
        Pc.ts.H     = rdE(hEd(5), Pc.ts.H*1e3)/1e3;
        Pc.ts.L     = max(0.1, rdE(hEd(6), Pc.ts.L*1e3))/1e3;
        Pc.df.W_out = rdE(hEd(7), Pc.df.W_out*1e3)/1e3;
        Pc.df.H_out = rdE(hEd(8), Pc.df.H_out*1e3)/1e3;
        Pc.wall.ks  = max(0, rdE(hEd(9), Pc.wall.ks*1e6))/1e6;
        Pc.wall.on  = Pc.wall.ks > 0;
        ar = sqrt(Pc.ct.R);
        Pc.ct.W_in = Pc.ts.W*ar;  Pc.ct.H_in = Pc.ts.H*ar;
        setappdata(0,'WT3D_USERP',Pc);
        delete(fig);  delete(figS);
        explore_3d();
    end

    function v = rdE(h, old)
        v = str2double(get(h,'String'));
        if isnan(v) || v <= 0, v = old; end
    end
    function h = mkChk(x, y, lbl, on, cb)
        h = uicontrol(pnl,'Style','checkbox','Units','normalized', ...
            'Position',[x y 0.46 0.035],'String',lbl,'FontSize',8,'Value',on,'Callback',cb);
    end
    function setVis(h, s)
        if get(s,'Value'), v = 'on'; else, v = 'off'; end
        set(h,'Visible',v);
    end
    function setVisibleAx(axT, s)
        if get(s,'Value'), v='on'; else, v='off'; end
        set(axT,'Visible',v);
    end
end

%% ==================== 局部工具 ====================
function r = pctRange(A, mask, q)
v = A(mask);  v = v(isfinite(v));
v = sort(v(:));
r = [v(max(1,round((1-q)/2*numel(v)))), v(round((1+q)/2*numel(v)))];
if r(2) <= r(1), r(2) = r(1) + eps; end
end
