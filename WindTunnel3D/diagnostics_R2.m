% diagnostics_R2 — 3mm 全域锁死态综合诊断 (R2)
% 输出: ① 各 z 层净流量时空图 ② 关键时刻层流量剖面 ③ 静压剖面 (rho 16 帧时均, 三层采样)
% 数据: 360rpm 旋转解干净 28 快照 (t=150 倍数) + rho 16 帧 (t=9600~12000)
RT = fileparts(mfilename('fullpath'));  addpath(RT);
M = load(fullfile(RT,'gpu_meta.mat'));  meta = M.meta;
dx = meta.dx;  nx = meta.nx;  ny = meta.ny;  nz = meta.nz;
xs = meta.x0 + ((1:nx)-0.5)'*dx;
ys = meta.y0 + ((1:ny)-0.5)*dx;
zs = meta.z0 + ((1:nz)-0.5)*dx;
fid = fopen(fullfile(RT,'fluidx3d','windtunnel.raw'),'rb'); raw = fread(fid,inf,'uint8=>uint8'); fclose(fid);
raw = reshape(raw, nx, ny, nz);
jy = min(ny, max(1, round((0-meta.y0)/dx+0.5)));       % y=0 切片
fl2 = squeeze(raw(:,jy,:))==0;                          % (nx,nz) 流体掩码
nf = find(fl2);

%% ① 各 z 层净流量时空矩阵 Qmat (nSnap, nz): Qmat(q,k) = Σ_i ux(i,jy,k)·dy·dz
d = dir(fullfile(RT,'fluidx3d','vtk','u_*.vtk'));
ts = zeros(numel(d),1);
for q = 1:numel(d)
    tok = regexp(d(q).name,'u_(\d+)u-(\d+)\.vtk','tokens','once');
    ts(q) = str2double(tok{2});
end
[ts,ord] = sort(ts);  d = d(ord);
nKeep = 0;  Qmat = [];  tKeep = [];
for q = 1:numel(d)
    A = read_fluidx3d_vtk(fullfile(d(q).folder, d(q).name));
    if ~isequal(size(A), [nx, ny, nz, 3]), continue; end   % 跳过混代残留
    Ux = squeeze(A(:,jy,:,1));
    Ux(fl2==0) = 0;
    nKeep = nKeep + 1;
    Qmat(nKeep,:) = squeeze(sum(Ux,1))*ys(2)*dx;          % Σ_i ux·dy·dz → 每 z 层净流量
    tKeep(nKeep,1) = ts(q);
end
Qmat = Qmat(1:nKeep,:);  tKeep = tKeep(1:nKeep);
fprintf('有效快照 %d/%d\n', nKeep, numel(d));

fig = figure('Color','w','Position',[30 30 1450 560],'Visible','off');
subplot(1,2,1);
h = imagesc(zs*1e3, tKeep, Qmat);  set(gca,'YDir','normal');
colormap(gca, turbo); caxis(gca, [-0.05 0.05]); colorbar;
xlabel('z (mm)'); ylabel('t (步)');
title('各 z 层净流量时空图 (y=0 切片, 风机段 z=140~600)');
subplot(1,2,2); hold on; grid on;
tMark = [150 1500 3000 6000 12000];
cols = lines(numel(tMark));
for q = 1:numel(tMark)
    [~,qi2] = min(abs(tKeep-tMark(q)));
    plot(zs*1e3, Qmat(qi2,:), 'LineWidth', 1.3, 'Color', cols(q,:));
end
xlabel('z (mm)'); ylabel('Q_{层} (m³/s)');
legend(arrayfun(@(t) sprintf('t=%d', t), tMark, 'UniformOutput', false), 'Location','southeast');
title('关键时刻 z 层流量剖面');
exportgraphics(fig, fullfile(RT,'R2_qflow_spacetime.png'), 'Resolution', 125);
fprintf('① 时空图已写\n');

%% ② 静压剖面: rho 16 帧时均, 三条采样线 (叶轮心 z=zF / 射流层 z=0 / 出口下 z=zF-0.3)
dr = dir(fullfile(RT,'fluidx3d','vtk','rho_*.vtk'));
zLines = [meta.impCenterZ, 0.0, meta.impCenterZ-0.30];
rhoP = zeros(numel(dr), numel(zLines), nx);   % (帧, 线, x)
nR = 0;
for q = 1:numel(dr)
    R = read_fluidx3d_vtk(fullfile(dr(q).folder, dr(q).name));
    for L = 1:3
        kz = min(nz, max(1, round((zLines(L)-meta.z0)/dx+0.5)));
        rhoP(q,L,:) = squeeze(R(:,jy,kz));
    end
    nR = nR + 1;
end
fig2 = figure('Color','w','Position',[30 30 1400 500],'Visible','off');
hold on; grid on;
lg = {};
for L = 1:3
    pLine = squeeze(mean(rhoP(:,L,:),1));
    pLineP = (pLine - 1)*3*1.204;   % p ≈ (ρ−1)/3 × ρ0 晶格 → Pa 近似 (ρ0=1.2)
    hL = plot(xs*1e3, pLineP, 'LineWidth', 1.5);
    lg{end+1} = sprintf('采样线 z=%.2fm', zLines(L));
    if L==1, set(hL,'Color',[0 0.2 0.9]); elseif L==2, set(hL,'Color',[0.85 0.3 0.1]); else, set(hL,'Color',[0 0.5 0.2]); end
end
yline(0,'k:'); xlabel('x (mm)'); ylabel('静压 (Pa, (ρ−1)/3×ρ₀)');
legend(lg,'Location','northwest');
title(sprintf('锁死态静压剖面 (%d 帧时均): 风机段负压 vs 涵道段压力差 = 泵送有效性判据', nR));
exportgraphics(fig2, fullfile(RT,'R2_pressure_profile.png'), 'Resolution', 130);
fprintf('② 静压剖面已写\n');
% 数值表
for L = 1:3
    pL = squeeze(mean(rhoP(:,L,:),1));
    fprintf('线%d (z=%.2f): 风机段(x=-0.7~0) 均压 %.2f Pa | 涵道段(x=0.6~1.5) 均压 %.2f Pa | 差 %.2f Pa\n', ...
        L, zLines(L), ...
        mean(pLineP(xs>-0.7 & xs<0)), mean(pLineP(xs>0.6 & xs<1.5)), ...
        mean(pLineP(xs>0.6 & xs<1.5)) - mean(pLineP(xs>-0.7 & xs<0)));
end
% 静压剖面数据的物理口径: p_Pa ≈ (rho_lattice − 1)/3 × rho0_kg (理想气体 cs 基准)
