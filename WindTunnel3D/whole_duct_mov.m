function whole_duct_mov()
% whole_duct_mov — 风机驱动全涵道整体视频 (流畅化: 成对插值 2× 帧)
% 面板: 左=纵切 y=0 |U|(试验段来流观测) 右上=z=mid 水平 |U|(蜗壳出口→短管)
%       右下=纵切涡量 |ω|(扰流/层流观测)  + Q(t) 走势 + 进度条
% 性能 (2026-09-30): 只读入快照后立即取两个切片参与计算 (旧版全场 |U|+gradient,
%   每帧多耗 ~600MB 内存与大半渲染时间); 图形对象只建一次, 帧内仅 set CData
% 用法: whole_duct_mov   (需先跑完 windtunnel_fan.exe)
RT = fileparts(mfilename('fullpath'));   % 项目根 (套件内 <suite>/WindTunnel3D, 无盘符硬编码)
addpath(RT);                             % read_fluidx3d_vtk 等
M = load(fullfile(RT,'gpu_meta.mat'));  meta = M.meta;
nx = meta.nx;  ny = meta.ny;  nz = meta.nz;  x0 = meta.x0;  Sv = meta.velScale;
dx = meta.dx;

vd = dir(fullfile(RT,'fluidx3d','vtk','u_*.vtk'));
if isempty(vd), error('无 VTK 快照'); end
ts = zeros(numel(vd),1);
for k = 1:numel(vd)
    tok = regexp(vd(k).name, 'u_(\d+)u-(\d+)\.vtk', 'tokens', 'once');
    ts(k) = str2double(tok{2});
end
[ts, ord] = sort(ts);  vd = vd(ord);
sel = 1:numel(ts);

fid = fopen(fullfile(RT,'fluidx3d','windtunnel.raw'),'rb');
raw = fread(fid, inf, 'uint8');  fclose(fid);
raw = reshape(raw, nx, ny, nz);
if isfield(meta,'y0'), y0 = meta.y0; else, y0 = -(ny*dx)/2; end   % 域原点 (风机耦合域不对称)
if isfield(meta,'z0'), z0 = meta.z0; else, z0 = -(nz*dx)/2; end
xs = x0 + ((1:nx)-0.5)'*dx;
yv = y0 + ((1:ny)-0.5)*dx;
zv = z0 + ((1:nz)-0.5)*dx;
jy = min(ny,max(1,round((0-y0)/dx+0.5)));   % 过 y=0 的纵切 (蜗壳腔/叶轮中面)
jz = min(nz,max(1,round((0-z0)/dx+0.5)));   % 过 z=0 的水平切 (切向出口/风洞轴线)
solidVt = reshape(raw(:,jy,:) >= 1, nx, nz);
solidHz = reshape(raw(:,:,jz) >= 1, nx, ny);
% 测量面 (试验段入口+2格) 开孔率 — 与求解器同口径
xm = round((1.55 + 2*dx - x0)/dx);
openF = mean(raw(xm,:,:) == 0, 'all');

% Q(t) 诊断序列
T = readmatrix(fullfile(RT,'fluidx3d','fan_diag.csv'));
QmT = T(:,1);  QmV = T(:,2);

fig = figure('Color','w','Position',[30 30 1560 760],'Visible','off');
out = fullfile(RT,'gpu_whole_fan.mp4');
v = VideoWriter(out, 'MPEG-4');
fps = 4;  if ~isempty(getenv('WT_FPS')), fps = max(1, str2double(getenv('WT_FPS'))); end
v.FrameRate = fps;  v.Quality = 92;
open(v);
cleaner = onCleanup(@() close(v));

hIV = [];  hIW = [];  hAxV = [];  hAxW = [];  hTv = [];  hTw = [];  hPb = [];  hTxt = [];
hRot = [];   % 动态叶轮 (旋转模式: 每帧按真实转角重建叶片截面)
% ---- 动态叶轮预计算 (WT_ROT 模式): 叶片弧判定同求解器 bladeTest ----
hasRot = isfield(meta,'rotate') && meta.rotate;
if hasRot
    bl = meta.blade;  fl = meta.fanLayout;
    tbhR = max(bl.tB/2, 0.6*dx);
    wrapDg = mod(bl.th(2)-bl.th(1), 360);  pitchRad = 2*pi/bl.Z;  th1D = bl.th(1);
    nMx = ceil((wrapDg + 360/bl.Z + 4)/(360/bl.Z));
    X2 = reshape(xs - fl.xC, nx, 1);
    Z2 = reshape(zv - fl.zF, 1, nz);
    RC2 = sqrt(X2.^2 + Z2.^2);  TH2 = mod(atan2(Z2, X2), 2*pi);
    fprintf('[动态叶轮] 旋转模式: 叶片将按 impOmega*t 真实转角逐帧重建\n');
end
prev = struct('Sl',[],'WY',[]);  prevT = NaN;  nF = 0;
for kk = 1:numel(sel)
    Pk = slicePack(sel(kk));
    tk = ts(sel(kk));
    if ~isempty(prev.Sl)
        for a = [1/3, 2/3]
            Pi = struct('Sl', prev.Sl*(1-a)+Pk.Sl*a, 'WY', prev.WY*(1-a)+Pk.WY*a);
            ti = prevT*(1-a) + tk*a;
            nF = drawFrame(Pi, ti, nF);
        end
    end
    nF = drawFrame(Pk, tk, nF);
    prev = Pk;  prevT = tk;
end
close(v);
fprintf('--- whole_duct_mov 完成: %s (%d 帧, %.1f s) ---\n', out, nF, nF/fps);

    function Pk = slicePack(seli)
        % 读一个快照, 立即取 y=0 纵切与 z=mid 水平切两个面 (不再全场算 |U|)
        A3 = single(read_fluidx3d_vtk(fullfile(RT,'fluidx3d','vtk', vd(seli).name)))*Sv;
        Ay = A3(:,jy,:,:);  Az = A3(:,:,jz,:);
        Sl = squeeze(sqrt(sum(Ay.^2, 4)));  Sl(solidVt) = NaN;          % (nx,nz)
        Sh = squeeze(sqrt(sum(Az.^2, 4)));  Sh(solidHz) = NaN;          % (nx,ny)
        Ux = squeeze(Ay(:,:,:,1));  Uzw = squeeze(Ay(:,:,:,3));
        WY = abs(gradient(Uzw, dx) - gradient(Ux, dx));                 % ω_y = ∂uz/∂x − ∂ux/∂z (切片内)
        WY(solidVt) = NaN;
        Pk = struct('Sl',Sl,'Sh',Sh,'WY',WY);
    end

    function n = drawFrame(Pk, t, nf)
        Wp = Pk.WY(isfinite(Pk.WY) & Pk.WY > 1);
        wMax = max(prctile(Wp(:), 99), 100);
        qNow = interp1(QmT, QmV, t, 'nearest');
        if isempty(hIV)   % 图形骨架只建一次
            clf(fig);
            hAxV = subplot(1,2,1);  hold(hAxV,'on');  axis(hAxV,'equal');
            hIV = imagesc(hAxV, xs*1000, zv*1000, Pk.Sl'); set(hAxV,'YDir','normal');   % R2026a: Image 无 YDir, 用 axes 的
            colormap(hAxV, turbo);  caxis(hAxV, [0 25]);  colorbar(hAxV);
            set(hIV, 'AlphaData', ~isnan(Pk.Sl'));
            xlabel(hAxV, 'x (m)');  ylabel(hAxV, 'z (m)');
            hTv = title(hAxV, '', 'FontWeight','bold');
            hAxW = subplot(1,2,2);  hold(hAxW,'on');  axis(hAxW,'equal');
            hIW = imagesc(hAxW, xs*1000, zv*1000, Pk.WY'); set(hAxW,'YDir','normal');
            colormap(hAxW, turbo);  caxis(hAxW, [0 wMax]);  colorbar(hAxW);
            set(hIW, 'AlphaData', ~isnan(Pk.WY'));
            xlabel(hAxW, 'x (m)');  ylabel(hAxW, 'z (m)');
            hTw = title(hAxW, '');
            axes(fig, 'Position', [0.08 0.94 0.8 0.02]);
            hPb = patch([0 0 0 0], [0 0 1 1], [0.85 0.3 0.1], 'EdgeColor','none');
            xlim([0 1]);  ylim([0 1]);  axis off;
            hTxt = text(0, 1.6, '', 'FontSize', 10, 'Units', 'normalized', 'FontWeight', 'bold');
        else
            set(hIV, 'CData', Pk.Sl', 'AlphaData', ~isnan(Pk.Sl'));
            set(hIW, 'CData', Pk.WY', 'AlphaData', ~isnan(Pk.WY'));
            caxis(hAxW, [0 wMax]);
        end
        if hasRot   % 动态叶轮: 每帧按真实转角重建叶片截面 (红色覆盖, 含插值帧)
            th = meta.impOmega*meta.dt*t;
            B2 = false(nx, nz);
            for mm = 0:nMx
                pm = mod(TH2 - th, pitchRad) + mm*pitchRad;
                x2 = RC2.*cos(pm);  z2 = RC2.*sin(pm);
                dd = sqrt((x2-bl.C(1)).^2 + (z2-bl.C(2)).^2);
                a2 = mod(atan2d(z2-bl.C(2), x2-bl.C(1)) - th1D, 360);
                B2 = B2 | (abs(dd-bl.Ra)<=tbhR & a2<=wrapDg);
            end
            B2 = squeeze(B2);   % (nx,1,nz) 广播残留压回 2D
            if isempty(hRot)
                C3 = repmat(reshape([0.85 0.2 0.1], 1, 1, 3), nz, nx);
                hRot = imagesc(hAxV, xs*1000, zv*1000, C3, 'AlphaData', double(B2')*0.9);
            else
                set(hRot, 'AlphaData', double(B2')*0.9);
            end
        end
        set(hTv, 'String', sprintf('纵切 y=0 全涵道 | t=%.3f s (步 %d) | Q=%.2f m³/s', t*meta.dt, round(t), qNow));
        set(hTw, 'String', sprintf('纵切涡量 |ω_y| (均色上限 %.0f 1/s, 斑点=扰流)', wMax));
        set(hPb, 'XData', [0 t/meta.nSteps 1*t/meta.nSteps 0]);
        set(hTxt, 'String', sprintf('FanSim3D 风机驱动 | D_2=460mm Z=32 \\beta_{2A}=130° n=960rpm | 帧 %d', nf+1));
        drawnow;
        writeVideo(v, frame2im(getframe(fig)));
        n = nf + 1;
        fprintf('帧 %d (t=%.3f s)\n', n, t*meta.dt);
    end
end
