function fan_mov(stride)
% fan_mov — 风机 GPU 结果延时动画 (流畅化: 成对快照线性插值 → 2× 帧率)
%   fan_mov     → 双切片 |U| + 纵切涡量面板 (层流/扰流观测)
%   fan_mov(2)  → 每 2 个快照取 1
% 每帧自含: z=mid 水平 |U| + y=0 纵切 |U| + y=0 涡量 |ω_y|(扰流=斑点/层流=均匀)
%   固体=灰色(电机毂/盘/壁), 叶轮=深红, Q(t) 实测小图, 进度条
% 性能 (2026-09-30): 快照读入后立即取三个面参与计算 (旧版全场 |U|, 每帧多耗 ~370MB
%   与大半时间); 图形骨架只建一次, 帧内仅 set CData; 插值在切片包上进行
if nargin < 1 || isempty(stride), stride = 1; end
% 套件相对定位: 无盘符硬编码 (环境变量 WT3D_DIR 可覆盖)
FAN = fileparts(mfilename('fullpath'));
WT = getenv('WT3D_DIR');  if isempty(WT), WT = fullfile(FAN,'..','WindTunnel3D'); end
addpath(WT);
M = load(fullfile(FAN,'fan_meta.mat'));  meta = M.meta;
nx = meta.nx;  ny = meta.ny;  nz = meta.nz;  x0 = meta.x0;  Sv = meta.velScale;
dx = meta.dx;

vd = dir(fullfile(FAN,'fluidx3d','vtk','u_*.vtk'));
if isempty(vd), error('无 VTK 快照 — 先跑 export_fan_fluidx3d + windfan.exe'); end
ts = zeros(numel(vd),1);
for k = 1:numel(vd)
    tok = regexp(vd(k).name, 'u_(\d+)u-(\d+)\.vtk', 'tokens', 'once');
    ts(k) = str2double(tok{2});
end
[ts, ord] = sort(ts);  vd = vd(ord);
sel = 1:stride:numel(ts);

fid = fopen(fullfile(FAN,'fluidx3d','fan_sim.raw'),'rb');
raw = fread(fid, inf, 'uint8');  fclose(fid);
raw = reshape(raw, nx, ny, nz);
xs = x0 + ((1:nx)-0.5)*dx;
yv = ((1:ny)-0.5-ny/2)*dx;
zv = yv;
jz = round(nz/2);  jy = round(ny/2);
solidHz = reshape(raw(:,:,jz) >= 1, nx, ny);
solidVt = reshape(raw(:,jy,:) >= 1, nx, nz);
rotHz = reshape(raw(:,:,jz) == 2, nx, ny);
rotVt = reshape(raw(:,jy,:) == 2, nx, nz);
openF = mean(raw(nx-4,:,:) == 0, 'all');
uMax = 28;

fig = figure('Color','w','Position',[40 40 1280 900],'Visible','off');
out = fullfile(FAN,'fan_timelapse.mp4');
v = VideoWriter(out, 'MPEG-4');
v.FrameRate = 8;  v.Quality = 92;
open(v);
cleaner = onCleanup(@() close(v));

hAxH = [];  hAxV = [];  hAxW = [];  hAxQ = [];   % 图形骨架句柄 (只建一次)
hI = [];  hI2 = [];  hIW = [];  hPQ = [];  hPb = [];  hTxt = [];  hT1 = [];
prev = [];  prevT = NaN;  nF = 0;
for kk = 1:numel(sel)
    Pk = slicePack(sel(kk));
    tk = ts(sel(kk));
    if ~isempty(prev)                                   % 两段插值帧 (流畅化, 切片包插值)
        for a = [1/3, 2/3]
            Pi = struct('Sl', prev.Sl*(1-a)+Pk.Sl*a, 'UH', prev.UH*(1-a)+Pk.UH*a, ...
                        'Ws', prev.Ws*(1-a)+Pk.Ws*a, 'Qnow', prev.Qnow*(1-a)+Pk.Qnow*a);
            ti = prevT*(1-a) + tk*a;
            nF = drawFrame(Pi, ti, nF);
        end
    end
    nF = drawFrame(Pk, tk, nF);
    prev = Pk;  prevT = tk;
end
close(v);
fprintf('--- fan_mov 完成: %s (%d 帧, %.1f s) ---\n', out, nF, nF/8);

    function Pk = slicePack(seli)
        % 读一个快照, 立即取 y=0 纵切 / z=mid 水平切 / Q 测量面三个面
        A3 = single(read_fluidx3d_vtk(fullfile(FAN,'fluidx3d','vtk', vd(seli).name)))*Sv;
        Ay = A3(:,jy,:,:);  Az = A3(:,:,jz,:);
        Sl = squeeze(sqrt(sum(Ay.^2, 4)));                              % (nx,nz)
        UH = squeeze(sqrt(sum(Az.^2, 4)));                              % (nx,ny)
        Ux = squeeze(Ay(:,:,:,1));  Uzw = squeeze(Ay(:,:,:,3));
        Ws = abs(gradient(Uzw, dx) - gradient(Ux, dx));                 % ω_y (切片内)
        Ws(solidVt) = NaN;
        Qm = squeeze(sqrt(sum(A3(nx-4,:,:).^2, 4)));                    % 出口测量面 (ny,nz)
        Qnow = mean(Qm(:),'omitnan')*openF*dx*dx;                       % m³/s 近似
        Pk = struct('Sl',Sl,'UH',UH,'Ws',Ws,'Qnow',Qnow);
    end

    function n = drawFrame(Pk, t, nf)
        if isempty(hAxH)   % 图形骨架只建一次
            clf(fig);
            hAxH = subplot(2,2,1);  hold(hAxH,'on');  axis(hAxH,'equal');
            UH0 = Pk.UH;  UH0(isnan(UH0)) = 0;
            hI = imagesc(hAxH, xs, yv, UH0'); set(hAxH,'YDir','normal');
            set(hI, 'AlphaData', ~solidHz');
            imagesc(hAxH, xs, yv, 0.45*ones(ny,nx,3), 'AlphaData', solidHz');
            imagesc(hAxH, xs, yv, 0.55*ones(ny,nx,3), 'AlphaData', rotHz');
            colormap(hAxH, turbo);  caxis(hAxH, [0 uMax]);
            hc = colorbar(hAxH);  ylabel(hc, '|U| (m/s)');
            axis(hAxH, [x0, x0+nx*dx, -max(yv)*1.02, max(yv)*1.02]);
            xlabel(hAxH, 'x (m)');  ylabel(hAxH, 'y (m)');
            hT1 = title(hAxH, '', 'FontWeight','bold');
            hAxV = subplot(2,2,2);  hold(hAxV,'on');  axis(hAxV,'equal');
            Sl0 = Pk.Sl;  Sl0(isnan(Sl0)) = 0;
            hI2 = imagesc(hAxV, xs, zv, Sl0'); set(hAxV,'YDir','normal');
            set(hI2, 'AlphaData', ~solidVt);
            imagesc(hAxV, xs, zv, 0.45*ones(nz,nx,3), 'AlphaData', solidVt);
            imagesc(hAxV, xs, zv, 0.55*ones(nz,nx,3), 'AlphaData', rotVt');
            colormap(hAxV, turbo);  caxis(hAxV, [0 uMax]);
            hc2 = colorbar(hAxV);  ylabel(hc2, '|U| (m/s)');
            axis(hAxV, [x0, x0+nx*dx, -max(zv)*1.02, max(zv)*1.02]);
            xlabel(hAxV, 'x (m)');  ylabel(hAxV, 'z (m)');
            title(hAxV, '纵切 y=0 (钟形口→叶轮→出口→风洞)');
            hAxW = subplot(2,2,4);  hold(hAxW,'on');  axis(hAxW,'equal');
            hIW = imagesc(hAxW, xs, zv, Pk.Ws'); set(hAxW,'YDir','normal');
            colormap(hAxW, turbo);  caxis(hAxW, [0 600]);
            hc3 = colorbar(hAxW);  ylabel(hc3, '|ω_y| (1/s)');
            axis(hAxW, [x0, x0+nx*dx, -max(zv)*1.02, max(zv)*1.02]);
            xlabel(hAxW, 'x (m)');  ylabel(hAxW, 'z (m)');
            title(hAxW, '纵切涡量 |ω_y| (均匀低值=层流, 斑点=扰流)');
            hAxQ = axes(fig, 'Position', [0.055 0.50 0.15 0.14], 'Color', 'w');
            hold(hAxQ, 'on');
            hPQ = plot(hAxQ, t, Pk.Qnow, 'ko', 'MarkerFaceColor', 'y', 'MarkerSize', 5);
            xlim(hAxQ, [0 meta.nSteps]);  ylim(hAxQ, [0, 1.8]);
            title(hAxQ, 'Q(t) 实测 (m³/s)', 'FontSize', 8);
            set(hAxQ, 'FontSize', 7);
            axes(fig, 'Position', [0.135 0.955 0.78 0.02]);
            hPb = patch([0 0 0 0], [0 0 1 1], [0.85 0.3 0.1], 'EdgeColor','none');
            xlim([0 1]);  ylim([0 1]);  axis off;
            hTxt = text(0, 1.4, '', 'FontSize', 9, 'Units', 'normalized');
        else               % 帧内仅更新数据
            UH0 = Pk.UH;  UH0(isnan(UH0)) = 0;
            Sl0 = Pk.Sl;  Sl0(isnan(Sl0)) = 0;
            set(hI,  'CData', UH0');
            set(hI2, 'CData', Sl0');
            set(hIW, 'CData', Pk.Ws');
            set(hPQ, 'XData', t, 'YData', Pk.Qnow);
            set(hT1, 'String', sprintf('水平 z=mid | t=%.3f s | Q=%.2f m³/s', t*meta.dt, Pk.Qnow));
        end
        set(hPb, 'XData', [0 t/meta.nSteps 1*t/meta.nSteps 0]);
        set(hTxt, 'String', sprintf('FanSim3D 风机驱动 | D_2=460mm Z=32 \\beta_{2A}=130° n=960rpm | 帧 %d', nf+1));
        drawnow;
        writeVideo(v, frame2im(getframe(fig)));
        n = nf + 1;
        fprintf('帧 %d (t=%.3f s)\n', n, t*meta.dt);
    end
end
