% render_of — OpenFOAM 场渲染核心 (读 legacy VTK 的 CELL_DATA: 格心字段 C + 格心 U)
% 语义: 黑底/蓝青白流动/红固体/双轴锚点 (对齐 explore_gpu)
% 供 slices_of / gif_of 共用; 输出全 ASCII (脚本链编码教训)
function [P, U, magU, solid] = render_of(vtkFile)
s = readlines(vtkFile);
nC = cell_count(s, 'CELL_DATA');
C = cellfield(s, 'C ', 3, nC);       % foamToVTK 原生格心字段
U = cellfield(s, 'U ', 3, nC);
P = C;
magU = vecnorm(U, 2, 2);
fprintf('[render_of] cell centres %d, x range [%.2f, %.2f], NaN cells %d\n', ...
    nC, min(P(:,1)), max(P(:,1)), nnz(isnan(P(:))));

% ---- 固体表面点 (patch legacy VTK: VTK/<patch>/<patch>_<time>.vtk 的顶点) ----
solid = [];
vroot = fileparts(vtkFile);
for pn = {'impeller', 'skin_casing'}
    cand = dir(fullfile(vroot, pn{1}, [pn{1} '_*.vtk']));
    if ~isempty(cand)
        solid = [solid; read_patch_pts(fullfile(cand(1).folder, cand(1).name))];
    end
end
end

function nC = cell_count(s, tag)
iu = find(contains(s, tag), 1);
tok = regexp(s{iu}, '(\d+)', 'tokens', 'once');
nC = str2double(tok{1});
end

function v = cellfield(s, name, ncomp, n)
iu = find(startsWith(s, name), 1);
buf = [];  i = iu + 1;
while numel(buf) < n*ncomp && i <= numel(s)
    isHeader = ~isempty(regexp(s{i}, '^[A-Za-z]\w* \d+ \d+', 'once'));
    if ~isempty(strtrim(s{i})) && ~isHeader
        buf = [buf, sscanf(s{i}, '%f')']; %#ok<AGROW>
    end
    i = i + 1;
end
v = reshape(buf(1:n*ncomp), ncomp, [])';
end

function pts = read_patch_pts(f)
s = readlines(f);
ip = find(contains(s, 'POINTS '), 1);
np = sscanf(s{ip}, 'POINTS %d');
nPL = ceil(np*3/10);
d = sscanf(strjoin(string(s(ip+1:ip+nPL)), ' '), '%f');
pts = reshape(d, 3, [])';
pts = pts(1:np, :);
end
