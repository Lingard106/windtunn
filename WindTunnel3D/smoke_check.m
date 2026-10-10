%% _smoke_check - 参数修改后的几何/网格/派生量快速体检 (不求解)
P = tunnel_params();
G = tunnel_geometry(P);
fprintf('=== 构型 %s ===\n', P.config);
fprintf('试验段: %.0fx%.0fx%.0f mm @ %.0f m/s\n', P.ts.W*1e3, P.ts.H*1e3, P.ts.L*1e3, P.V_test);
fprintf('收缩段: R=%.1f, L=%.0fmm, 入口 %.0fx%.0f mm\n', P.ct.R, P.ct.L*1e3, P.ct.W_in*1e3, P.ct.H_in*1e3);
fprintf('稳定段: L=%.0fmm | 扩散段: 出口 %.0fx%.0f, L=%.0fmm\n', ...
    P.sc.L*1e3, P.df.W_out*1e3, P.df.H_out*1e3, P.df.L*1e3);
fprintf('全长: %.2f m | 收缩段峰值壁面角: %.1fdeg (要求<35) | 扩散段等效锥角: %.2fdeg (全张角 %.2fdeg, 要求2theta<7)\n', ...
    G.x(end), G.theta_ct_max, G.theta_df_eq, 2*G.theta_df_eq);
fprintf('稳态网格 (默认档): %dx%dx%d = %.1fM 格, 单元 %.2fx%.2fx%.2f mm\n', ...
    P.flow.nx, P.flow.ny, P.flow.nz, P.flow.nx*P.flow.ny*P.flow.nz/1e6, ...
    G.x(end)/P.flow.nx*1e3, P.ct.W_in*1.06/P.flow.ny*1e3, P.ct.H_in*1.06/P.flow.nz*1e3);
fprintf('核心速度范围: [%.1f, %.1f] m/s (设计 %.0f)\n', min(G.U), max(G.U), P.V_test);

% GPU 网格预估 (2mm 档, 只做体素化)
P2 = P;  P2.lbm.dx = 0.002; if strcmpi(P2.lbm.object,'stl') && ~exist(P2.lbm.objFile,'file'), P2.lbm.object = 'sphere'; end
G2 = tunnel_geometry(P2);
[~, ~, gr] = tunnel_voxelize(P2, G2);
fprintf('GPU 2mm 档网格: %dx%dx%d = %.1fM 格\n', gr.nx, gr.ny, gr.nz, gr.nx*gr.ny*gr.nz/1e6);
% 冒烟档 (6mm)
P3 = P;  P3.lbm.dx = 0.006;  P3.lbm.margins = [0.06 0.12]; if strcmpi(P3.lbm.object,'stl') && ~exist(P3.lbm.objFile,'file'), P3.lbm.object = 'sphere'; end
G3 = tunnel_geometry(P3);
[~, ~, gr3] = tunnel_voxelize(P3, G3);
fprintf('GPU 6mm 冒烟档网格: %dx%dx%d = %.2fM 格\n', gr3.nx, gr3.ny, gr3.nz, gr3.nx*gr3.ny*gr3.nz/1e6);
% 涵道完整性: 域宽必须覆盖最宽处
aI = interp1(G2.x, G2.a, gr.x(1)+((1:gr.nx)-0.5)*0.002, 'pchip', 'extrap');
fprintf('域覆盖检查: max(2a)=%.3f m vs 域宽 ny*dx=%.3f m -> %s\n', ...
    2*max(aI), gr.ny*0.002, string(2*max(aI) < gr.ny*0.002));
% aniso 预设回归 (矩形截面不再截断)
Pa = tunnel_params('aniso'); if strcmpi(Pa.lbm.object,'stl') && ~exist(Pa.lbm.objFile,'file'), Pa.lbm.object = 'sphere'; end
Ga = tunnel_geometry(Pa);
[~, ~, gra] = tunnel_voxelize(Pa, Ga);
aIa = interp1(Ga.x, Ga.a, gra.x(1)+((1:gra.nx)-0.5)*Pa.lbm.dx, 'pchip', 'extrap');
fprintf('aniso 回归: max(2a)=%.3f vs 域宽 %.3f -> %s\n', ...
    2*max(aIa), gra.ny*Pa.lbm.dx, string(2*max(aIa) < gra.ny*Pa.lbm.dx));
fprintf('=== 体检完成 ===\n');
