% extract_sweep — 转速扫掠结果提取 + 出图 (收敛点 Q/Ux + 发散标记)
% 数据源: logs/log.rpm<number> (各转速) 与 logs/log.run (960 基线)
function extract_sweep(base)
if nargin < 1, base = 'D:\WindTunnelSuite\openfoam\windtunnel-mrf'; end
rpms = [790 960 1080 1180];
logs = {fullfile(base,'logs','log.rpm790'), fullfile(base,'logs','log.run'), ...
        fullfile(base,'logs','log.rpm1080'), fullfile(base,'logs','log.rpm1180')};
Q = nan(size(rpms));  Uts = Q;  Qsd = Q;  Usd = Uts;  diverged = false(size(rpms));
for k = 1:numel(rpms)
    f = logs{k};
    if ~exist(f, 'file'), fprintf('[skip] %s\n', f); continue; end
    s = readlines(f);  s = strrep(s, char(0), '');
    qL = s(contains(s, 'sum(skin_outletCap)'));
    qv = NaN(1, numel(qL));
    for i = 1:numel(qL)
        tok = regexp(qL(i), '=\s*(-?\d+\.?\d*e?[+-]?\d*)', 'tokens', 'once');
        if ~isempty(tok), qv(i) = str2double(tok{1}); end
    end
    qv = qv(~isnan(qv));
    uL = s(contains(s, 'volAverage(tsZone) of U'));
    uv = NaN(1, numel(uL));
    for i = 1:numel(uL)
        tok = regexp(uL(i), '\((-?\d+\.?\d*e?[+-]?\d*)', 'tokens', 'once');
        if ~isempty(tok), uv(i) = str2double(tok{1}); end
    end
    uv = uv(~isnan(uv));
    m = max(1, ceil(0.7*numel(qv)));
    med = median(qv(m:end));
    if abs(median(abs(qv(m:end)), 'omitnan')) > 100 || max(abs(qv(m:end))) > 500
        diverged(k) = true;   % 发散判定: 末段 |Q| 离谱
    else
        Q(k) = med;  Qsd(k) = std(qv(m:end));
        m2 = max(1, ceil(0.7*numel(uv)));
        Uts(k) = mean(uv(m2:end));  Usd(k) = std(uv(m2:end));
    end
    fprintf('rpm=%4d: Q=%.3f±%.3f | Ux=%.2f±%.2f | diverged=%d\n', rpms(k), Q(k), Qsd(k), Uts(k), Usd(k), diverged(k));
end

fig = figure('Position', [20 20 1400 560], 'Color', 'w');
subplot(1,2,1); hold on; grid on;
ok = ~diverged & ~isnan(Q);
if nnz(ok) >= 2
    kf = polyfit(rpms(ok), Q(ok), 1);
    nn = linspace(min(rpms)-40, max(rpms)+40, 30);
    plot(nn, polyval(kf, nn), 'b-', 'LineWidth', 1.5);
end
plot(rpms(ok), Q(ok), 'ro', 'MarkerSize', 9, 'LineWidth', 1.5);
lg = arrayfun(@(n) sprintf('%d', n), rpms(ok), 'UniformOutput', false);
if any(diverged(:))
    plot(rpms(diverged), zeros(nnz(diverged),1), 'rx', 'MarkerSize', 12, 'LineWidth', 2);
    lg{end+1} = 'diverged'; %#ok<AGROW>
end
xlabel('rpm'); ylabel('Q (m³/s)'); title('MRF operating points (fan law Q\propon cross-check)');
legend(lg, 'Location', 'northwest');

subplot(1,2,2); hold on; grid on;
errorbar(rpms(ok), Uts(ok), Usd(ok), 'ro-', 'LineWidth', 1.5, 'MarkerSize', 8, 'CapSize', 10);
yline(20, 'g--'); yline(30, 'g--'); yline(25, 'g-');
text(min(rpms)+20, 25.6, 'target 25', 'Color', 'g');
xlabel('rpm'); ylabel('test section Ux (m/s, vol-avg)');
title('rpm sweep: test section speed (design band 20-30)'); ylim([0 32]);
exportgraphics(fig, fullfile(base, 'sweep_result.png'), 'Resolution', 130);
fprintf('sweep_result.png done\n');
end
