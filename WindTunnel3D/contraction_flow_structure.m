% contraction_flow_structure — show turning flow in the contraction slice (y=0)
% Renders: speed magnitude (imagesc) + in-plane velocity vectors (U,W) + duct walls
% Temp diagnostic script; result saved to contraction_flow_structure.png
load('flow3d_result.mat', 'S');
P = tunnel_params('balanced');
G = tunnel_geometry(P);
jc = round((numel(S.y)+1)/2);

sl = squeeze(S.spd(:, jc, :))';          % [nz x nx] speed
Usl = squeeze(S.U(:, jc, :))';           % [nz x nx] axial
Wsl = squeeze(S.W(:, jc, :))';           % [nz x nx] vertical
Zg = S.z * 1000;  Xg = S.x * 1000;

fig = figure('Position', [40 40 1350 620], 'Color', 'w');
subplot(1, 2, 1); hold on;
imagesc(Xg, Zg, sl); axis xy;
caxis([0 P.V_test*1.25]); colormap(turbo(256)); cb = colorbar; cb.Label.String = 'speed (m/s)';
% in-plane vectors, subsampled (true direction, arrow length = local speed*25mm per (m/s))
sx = 1:8:numel(S.x);  sz = 1:4:numel(S.z);
[XX, ZZ] = meshgrid(Xg(sx), Zg(sz));
quiver(XX, ZZ, Usl(sz, sx), Wsl(sz, sx), 0.9, 'k', 'LineWidth', 0.7, 'MaxHeadSize', 0.4);
% duct wall silhouette (upper wall a(x), lower -a)
plot(G.x*1000, interp1(G.x, G.a, G.x)*1000, 'k-', 'LineWidth', 1.5);
plot(G.x*1000, -interp1(G.x, G.a, G.x)*1000, 'k-', 'LineWidth', 1.5);
xlim([150 750]); ylim([-480 480]);
xlabel('x (mm)'); ylabel('z (mm)');
title('Contraction slice (y=0): speed + in-plane velocity vectors');

subplot(1, 2, 2); hold on; grid on;
% vertical profiles through the contraction: flow turning W(z) at 4 stations
cols = lines(4);
sts = [0.34 0.42 0.50 0.58];
for si = 1:4
    [~, i] = min(abs(S.x - sts(si)));
    wProf = squeeze(S.W(i, jc, :));          % m/s
    plot(wProf, Zg, '-', 'Color', cols(si,:), 'LineWidth', 1.6);
end
xlabel('W vertical-turn speed (m/s)'); ylabel('z (mm)');
legend(arrayfun(@(s) sprintf('x=%.0fmm', s*1000), sts, 'UniformOutput', false), 'Location', 'northwest');
title('Turning flow W(z) through the contraction');
ylim([-480 480]);

exportgraphics(fig, 'contraction_flow_structure.png', 'Resolution', 110);
fprintf('contraction_flow_structure.png written\n');
