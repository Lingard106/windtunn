% probe_flow — 从 legacy VTK 提取关键区域流速 (泵送方向/发散诊断)
% 用法: probe_flow('VTK/windtunnel-mrf_2750.vtk')   (无参时默认 VTK 下最新时间步)
% 2026-10-06 T8 重写: 改走 render_of 的格心解析 — 旧版自己读 POINTS 配 CELL_DATA 的 U,
% 坐标与场错位可达一格(8mm), 环带均值统计被污染; 且 vtk 路径硬编码 2750
function probe_flow(vtkFile)
if nargin < 1
    d = 'VTK';
    c = dir(fullfile(d, 'windtunnel-mrf_*.vtk'));
    assert(~isempty(c), 'no windtunnel-mrf_*.vtk under VTK/');
    [~, ix] = max([c.datenum]);
    vtkFile = fullfile(d, c(ix).name);
end
[P, U, magU, ~] = render_of(vtkFile);
fprintf('[probe_flow] %s | %d cells | |U|max %.1f\n', vtkFile, size(P,1), max(magU));

xC = -0.5556;  zF = 0.3726;
sl = abs(P(:,2)) < 0.05;
r = hypot(P(sl,1)-xC, P(sl,3)-zF);
band = false(size(P,1),1);
bandIdx = find(sl);
band(bandIdx) = r>0.15 & r<0.22;
rr = zeros(size(P,1),1);  rr(bandIdx) = r;
ur = ((P(band,1)-xC).*U(band,1) + (P(band,3)-zF).*U(band,3)) ./ max(rr(band),1e-6);
ut = ((P(band,1)-xC).*U(band,3) - (P(band,3)-zF).*U(band,1)) ./ max(rr(band),1e-6);
fprintf('rotor band %d pts | radial %+.3f (pos=outward) | tangential %+.3f (pos=+phi)\n', ...
    nnz(band), mean(ur,'omitnan'), mean(ut,'omitnan'));

ts = P(:,1)>1.56 & P(:,1)<1.74 & abs(P(:,2))<0.1 & abs(P(:,3))<0.08;
fprintf('test section Ux %+.3f m/s (%d pts)\n', mean(U(ts,1),'omitnan'), nnz(ts));

eye = hypot(P(:,1)-xC, P(:,3)-zF)<0.15 & P(:,2)>0.13 & P(:,2)<0.16;
fprintf('eye Uy %+.3f m/s (neg=intake ok, %d pts)\n', mean(U(eye,2),'omitnan'), nnz(eye));
end
