% gen_blockmesh — 从 tunnel_params 生成 OpenFOAM blockMeshDict (五段变截面风洞)
% 前置扩散段(fd) → 稳定段(sc) → 收缩段(ct) → 试验段(ts) → 扩散段(df)
% 每段一个 block, 轴向 x, 截面 y(宽)×z(高), 矩形截面
RT = 'D:\WindTunnelSuite\WindTunnel3D';
addpath(RT);
P = tunnel_params();
G = tunnel_geometry(P);

% 网格分辨率: 每段轴向/横向格数 (总格数约 2-5M, CPU 可跑)
nx_fd = 40;   % 前置扩散段 轴向 40
nx_sc = 30;   % 稳定段 30
nx_ct = 40;   % 收缩段 40
nx_ts = 20;   % 试验段 20
nx_df = 40;   % 扩散段 40
ny_w = 24;    % 宽度方向 24
nz_h = 16;    % 高度方向 16

% 各段起止 x 坐标 (从 G.xSec 取)
xSec = G.xSec;   % [0, 0.5, 0.95, 1.55, 1.85, 2.5]
% [fd_start, fd_end/sc_start, sc_end/ct_start, ct_end/ts_start, ts_end/df_start, df_end]
sections = {
    xSec(1), xSec(2), nx_fd, 'fd'     % 前置扩散段
    xSec(2), xSec(3), nx_sc, 'sc'     % 稳定段
    xSec(3), xSec(4), nx_ct, 'ct'     % 收缩段
    xSec(4), xSec(5), nx_ts, 'ts'     % 试验段
    xSec(5), xSec(6), nx_df, 'df'     % 扩散段
};
nSec = size(sections, 1);

% 简化: 用均匀矩形截面 (每段用入口/出口的宽高线性内插)
% 从 G.a(半宽) G.b(半高) 获取每段的截面
fprintf('=== 风洞五段几何 ===\n');
for s = 1:nSec
    x0 = sections{s,1};  x1 = sections{s,2};
    i0 = find(abs(G.x-x0)<1e-6, 1);  i1 = find(abs(G.x-x1)<1e-6, 1);
    if isempty(i0), i0 = 1; end
    if isempty(i1), i1 = numel(G.x); end
    a0 = G.a(i0)*2;  b0 = G.b(i0)*2;   % 宽/高
    a1 = G.a(i1)*2;  b1 = G.b(i1)*2;
    fprintf('%s: x=%.2f~%.2f | %.0f×%.0f → %.0f×%.0f mm\n', sections{s,4}, x0, x1, a0*1e3, b0*1e3, a1*1e3, b1*1e3);
end

%% 生成 blockMeshDict
fid = fopen(fullfile(RT,'openfoam','windtunnel-base','system','blockMeshDict'), 'w');
fprintf(fid, '/*--------------------------------*- C++ -*----------------------------------*\\\n');
fprintf(fid, '| =========                 |                                                 |\n');
fprintf(fid, '| \\\\      /  F ield         | OpenFOAM: The Open Source CFD Toolbox           |\n');
fprintf(fid, '|  \\\\    /   O peration     | Version: 1912                                   |\n');
fprintf(fid, '|   \\\\  /    A nd           | Web:      www.OpenFOAM.com                      |\n');
fprintf(fid, '|    \\\\/     M anipulation  |                                                 |\n');
fprintf(fid, '\\*---------------------------------------------------------------------------*/\n');
fprintf(fid, 'FoamFile\n{\n    format      ascii;\n    class       dictionary;\n    object      blockMeshDict;\n}\n// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //\n\n');
fprintf(fid, 'convertToMeters 1;\n\n');

% 顶点: 每段截面 4 角点, 相邻段共享截面 (不共享格点以保证壁面阶跃)
% 简化: 每段独立 8 顶点, 段间面重合
nv = 0;
vertices = '';
blocks = '';
edges = '';
% 每段截面: y=±W/2, z=±H/2 (以 z 轴为高度, y 轴为宽度, x 为流向)
% 顶点编号: 每段 8 个 (x0/x1 × y± × z±)
for s = 1:nSec
    x0 = sections{s,1};  x1 = sections{s,2};
    % 用 tunnel_geometry 的型线取入口/出口截面尺寸
    i0 = find(abs(G.x-x0)<1e-6, 1);  i1 = find(abs(G.x-x1)<1e-6, 1);
    if isempty(i0), i0 = 1; end
    if isempty(i1), i1 = numel(G.x); end
    W0 = G.a(i0)*2;  H0 = G.b(i0)*2;
    W1 = G.a(i1)*2;  H1 = G.b(i1)*2;
    nx = sections{s,3};
    % 顶点 (8 个): v0-v3 在 x0 面 (y-H/2 z-H/2; y+W/2 z-H/2; y+W/2 z+H/2; y-H/2 z+H/2)
    %              v4-v7 在 x1 面 (同序)
    % 对渐变截面: 用入口截面尺寸做 x0 的 4 顶点, 出口截面尺寸做 x1 的 4 顶点 → blockMesh 自动线性内插
    v0 = nv + 0;  v1 = nv + 1;  v2 = nv + 2;  v3 = nv + 3;
    v4 = nv + 4;  v5 = nv + 5;  v6 = nv + 6;  v7 = nv + 7;
    vertices = [vertices sprintf('    (%.6f %.6f %.6f)   // s%d v0\n', x0, -W0/2, -H0/2), ...
                sprintf('    (%.6f %.6f %.6f)   // s%d v1\n', x0,  W0/2, -H0/2), ...
                sprintf('    (%.6f %.6f %.6f)   // s%d v2\n', x0,  W0/2,  H0/2), ...
                sprintf('    (%.6f %.6f %.6f)   // s%d v3\n', x0, -W0/2,  H0/2), ...
                sprintf('    (%.6f %.6f %.6f)   // s%d v4\n', x1, -W1/2, -H1/2), ...
                sprintf('    (%.6f %.6f %.6f)   // s%d v5\n', x1,  W1/2, -H1/2), ...
                sprintf('    (%.6f %.6f %.6f)   // s%d v6\n', x1,  W1/2,  H1/2), ...
                sprintf('    (%.6f %.6f %.6f)   // s%d v7\n', x1, -W1/2,  H1/2)];
    nv = nv + 8;
    blocks = [blocks sprintf('    hex (%d %d %d %d %d %d %d %d) (%d %d %d) simpleGrading (1 1 1)  // %s\n', ...
        v0, v1, v2, v3, v4, v5, v6, v7, nx, ny_w, nz_h, sections{s,4})];
end

% 默认面
defaultFaces = '';
fprintf(fid, 'vertices\n(\n%s);\n\n', vertices);
fprintf(fid, 'blocks\n(\n%s);\n\n', blocks);
fprintf(fid, 'edges\n(\n);\n\n');
fprintf(fid, 'boundary\n(\n');
% 入口
x0_first = sections{1,1};
i0 = 1;
W0 = G.a(i0)*2;  H0 = G.b(i0)*2;
fprintf(fid, '    inlet\n    {\n        type patch;\n        faces\n        (\n            (%d 3 2 1)\n        );\n    }\n', 0);
% 出口
xl = sections{nSec,2};
i1 = find(abs(G.x-xl)<1e-6, 1);
if isempty(i1), i1 = numel(G.x); end
W1 = G.a(i1)*2;  H1 = G.b(i1)*2;
nv_last = nv - 8;
fprintf(fid, '    outlet\n    {\n        type patch;\n        faces\n        (\n            (%d %d %d %d)\n        );\n    }\n', nv_last+4, nv_last+5, nv_last+6, nv_last+7);
% 壁面
fprintf(fid, '    walls\n    {\n        type wall;\n        faces\n        (\n');
for s = 1:nSec
    b0 = (s-1)*8;
    % 底面
    fprintf(fid, '            (%d %d %d %d)\n', b0+0, b0+4, b0+5, b0+1);
    % 顶面
    fprintf(fid, '            (%d %d %d %d)\n', b0+3, b0+2, b0+6, b0+7);
    % 侧面
    fprintf(fid, '            (%d %d %d %d)\n', b0+1, b0+5, b0+6, b0+2);
    fprintf(fid, '            (%d %d %d %d)\n', b0+0, b0+3, b0+7, b0+4);
end
fprintf(fid, '        );\n    }\n');
fprintf(fid, ');\n\nmergePatchPairs\n(\n);\n\n');
fprintf(fid, '// ************************************************************************* //\n');
fclose(fid);
fprintf('blockMeshDict 已写 (%d 顶点, %d 块)\n', nv, nSec);
