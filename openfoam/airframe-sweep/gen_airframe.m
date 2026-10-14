function gen_airframe(cfg)
%GEN_AIRFRAME  参数化弹体+十字尾舵几何生成器 v1 (制导闭环 CFD 侧交付)
%
%   gen_airframe()       默认参数 = 占位 glider_phase0.urdf 逐项咬合
%   gen_airframe(cfg)    cfg 覆盖字段, 见 DEFAULTS
%
%   产物 (geometry/):
%     body.stl / fin_h_pos.stl / fin_h_neg.stl / fin_v_pos.stl / fin_v_neg.stl
%         独立网格 (弹体系绝对坐标, 米制) — snappyHexMesh / CFD 装配用
%     airframe_params.json  几何/质量惯量(分部件+系统合成)/无量纲化基准/坐标系约定
%     glider.mjcf           MuJoCo 模型骨架 (freejoint + 4 舵 revolute, 舵面用 box 图元)
%     README.md             产物清单与用法
%
%   坐标系: B 系 — x 前弹轴, y 右, z 下, 右手系; 原点 = 弹体几何中心。
%   与占位 URDF 的 link 系 (x前 y左 z上) 的 y/z 翻转由仿真器侧转换;
%   本生成器与 aero_table.csv 一律 B 系 (docs/mujoco_alignment_questions.md §4.2)。
%
%   自检: ①flat 模式薄片积分惯量 vs 解析式 (rel<1e-3)
%         ②STL 包围盒 == 预期  ③网格闭合体积 vs 解析体积 (rel<0.5%)

if nargin < 1, cfg = struct(); end

% ---------------- 参数 ----------------
D.x_L       = 1.2;      % 弹长 [m]
D.x_R       = 0.075;    % 弹体半径 [m]
D.nose      = 'flat';   % 'flat' | 'cone'
D.nose_len  = 0.25;     % 头锥长度 (nose='cone' 时生效) [m]
D.fin_chord = 0.10;     % 舵面弦长 (x 向) [m]
D.fin_span  = 0.12;     % 舵面展长 [m]
D.fin_thick = 0.003;    % 舵面厚度 [m]
D.x_fin     = -0.55;    % 舵铰链轴 x (盒体迎流前缘, 向 -x 延伸 chord) [m]
D.fin_root  = 0.03;     % 舵根距弹轴 (嵌入弹体防缝隙) [m]
D.m_body    = 10.0;     % 弹体质量 [kg]
D.m_fin     = 0.05;     % 单舵质量 [kg]
D.rho       = 1.225;    % 标称大气密度 (仅写入元数据) [kg/m^3]
D.n_th      = 64;       % 圆周分段
D.n_sl      = 400;      % 轴向惯量积分薄片数
D = merge_struct(D, cfg);

x_t = -D.x_L/2;  x_n = D.x_L/2;
assert(D.fin_root < D.x_R, 'fin_root 须小于弹体半径 (嵌入弹体)');
assert(any(strcmpi(D.nose, {'flat','cone'})), 'nose 仅支持 flat/cone');

outdir = fullfile(fileparts(mfilename('fullpath')), 'geometry');
if ~exist(outdir, 'dir'), mkdir(outdir); end

fprintf('=== gen_airframe v1 — B 系 (x前 y右 z下), 原点=弹体几何中心 ===\n');

% ---------------- 弹体 profile 与惯量 ----------------
if strcmpi(D.nose, 'flat')
    xs = [x_t, x_n];                  rs = [D.x_R, D.x_R];
else
    xs = [x_t, x_n-D.nose_len, x_n];  rs = [D.x_R, D.x_R, 0];
end

[Ib, Vbody, Fbody] = rev_body(xs, rs, D);

if strcmpi(D.nose, 'flat')
    Ixx_a = 0.5*D.m_body*D.x_R^2;
    Iyy_a = D.m_body*(D.x_L^2/12 + D.x_R^2/4);
    rel = max(abs([Ib.I(1,1)-Ixx_a, Ib.I(2,2)-Iyy_a]) / max([Ixx_a, Iyy_a]));
    assert(rel < 1e-3, 'flat 弹体惯量薄片积分 vs 解析式偏差 %.2e 超限', rel);
    fprintf('  自检① 弹体惯量 薄片积分 vs 解析: rel err = %.2e  PASS\n', rel);
end

% ---------------- 舵面 (4 片) ----------------
% 水平对 h_pos(+z)/h_neg(-z): 展沿 z, 厚沿 y, 铰链轴 y (同向=俯仰 de, 差动=滚转 da)
% 垂直对 v_pos(+y 右)/v_neg(-y): 展沿 y, 厚沿 z, 铰链轴 z (同向=偏航 dr)
fin_names = {'h_pos','h_neg','v_pos','v_neg'};
fins = struct();
for i = 1:4
    n = fin_names{i};
    is_h = any(strcmp(n, {'h_pos','h_neg'}));
    sgn  = 2*(i==1 || i==3) - 1;   % h_pos/v_pos 取 +
    if is_h, dims = [D.fin_chord, D.fin_thick, D.fin_span]; s_axis = 3;
    else,    dims = [D.fin_chord, D.fin_span, D.fin_thick]; s_axis = 2; end
    com = [D.x_fin - D.fin_chord/2, 0, 0];
    com(s_axis) = sgn*(D.fin_root + D.fin_span/2);
    fins.(n).dims = dims;
    fins.(n).com  = com;
    fins.(n).lo   = com - dims/2;            % 盒角点 (弹体系)
    fins.(n).I    = box_inertia(D.m_fin, dims);
    % MJCF 局部量: 舵 body 原点在铰链轴, 质心/盒心相对 body 的偏移
    fins.(n).rel  = com - [D.x_fin, 0, 0];   % = [-chord/2, 0, sgn*span/2]
end

% ---------------- 系统合成 (绕全系统质心) ----------------
I_cell = {Ib.I, fins.h_pos.I, fins.h_neg.I, fins.v_pos.I, fins.v_neg.I};
parts_m   = [D.m_body, D.m_fin*ones(1,4)];
parts_com = [Ib.com; fins.h_pos.com; fins.h_neg.com; fins.v_pos.com; fins.v_neg.com];
m_sys   = sum(parts_m);
com_sys = (parts_com.' * parts_m.').' / m_sys;
I_sys = zeros(3);
for i = 1:5
    d = parts_com(i,:).' - com_sys(:);
    I_sys = I_sys + I_cell{i} + parts_m(i)*(dot(d,d)*eye(3) - d*d.');
end
fprintf('  系统合成: m=%.3f kg, com=[%.4f %.4f %.4f], Ixx=%.4f Iyy=%.4f Izz=%.4f kg·m²\n', ...
        m_sys, com_sys, I_sys(1,1), I_sys(2,2), I_sys(3,3));

% ---------------- 写 STL ----------------
write_stl(fullfile(outdir,'body.stl'),      Vbody, Fbody);
for i = 1:4
    n = fin_names{i};
    [VB, FB] = box_mesh(fins.(n).lo, fins.(n).dims);
    write_stl(fullfile(outdir, sprintf('fin_%s.stl', n)), VB, FB);
end

% 自检②③: body.stl 包围盒与闭合体积
lo_exp = [x_t, -D.x_R, -D.x_R];  hi_exp = [x_n, D.x_R, D.x_R];
assert(all(abs(min(Vbody)-lo_exp) < 1e-9) && all(abs(max(Vbody)-hi_exp) < 1e-9), ...
       'body.stl 包围盒与预期不符');
vol   = mesh_volume(Vbody, Fbody);
vol_a = pi*D.x_R^2*D.x_L - vol_nose_trim(D);
relv  = abs(vol - vol_a)/vol_a;
assert(relv < 0.005, 'body.stl 闭合体积 %.4f vs 解析 %.4f 偏差 %.2e 超限', vol, vol_a, relv);
fprintf('  自检②③ body.stl 包围盒 OK, 闭合体积 %.5f m³ (解析 %.5f, rel %.2e)  PASS\n', ...
        vol, vol_a, relv);

% ---------------- airframe_params.json ----------------
S_ref = pi*D.x_R^2;   % v1 默认: 弹身迎面积 (待 Phase 0 冻结)
meta = struct();
meta.version = 'v1 (2026-10-14)';
meta.frame = 'B: x前弹轴, y右, z下, 右手系; 原点=弹体几何中心; 与 aero_table.csv 同系';
meta.baseline = '默认参数 = 占位 glider_phase0.urdf 逐项咬合 (质量/几何); 舵铰链=盒迎流前缘';
meta.air_density = D.rho;
meta.geometry = orderfields(D);
pnames = {'body','h_pos','h_neg','v_pos','v_neg'};
plist = repmat(struct('name','','mass',0,'com',zeros(1,3),'I_diag',zeros(1,3)), 1, 5);
for i = 1:5
    plist(i).name   = pnames{i};
    plist(i).mass   = parts_m(i);
    plist(i).com    = parts_com(i,:);
    plist(i).I_diag = diag(I_cell{i}).';
end
meta.parts = plist;
meta.system = struct('mass', m_sys, 'com', com_sys, ...
    'I_diag', diag(I_sys).', 'I_full', I_sys);
meta.aero_ref = struct( ...
    'S_ref', S_ref, 'L_ref', 2*D.x_R, ...
    'moment_ref_point', '全系统质心 (见 system.com)', ...
    'coeff_axes', '稳定轴(风轴): CD沿来流, CL垂直来流(向上/−z_B为正), CY侧向; Cm/Cl/Cn绕体轴, Cm正=抬头', ...
    'status', 'S_ref/L_ref/符号约定待 Phase 0 会议冻结 (docs/mujoco_alignment_questions.md §4)');
meta.fin_naming = struct( ...
    'h_pos', '+z_B 侧水平舵 (对应占位 URDF fin_down)', ...
    'h_neg', '-z_B 侧水平舵 (对应占位 URDF fin_up)', ...
    'v_pos', '+y_B 右侧垂直舵 (对应占位 URDF fin_left)', ...
    'v_neg', '-y_B 左侧垂直舵 (对应占位 URDF fin_right)', ...
    'mix', 'de=(h_pos+h_neg)/2 同向俯仰, da=(h_pos-h_neg)/2 差动滚转, dr=(v_pos+v_neg)/2 偏航 (spec §4.1 草案, 会议定稿)');
fid = fopen(fullfile(outdir,'airframe_params.json'), 'w');
fprintf(fid, '%s', jsonencode(meta, 'PrettyPrint', true));
fclose(fid);

% ---------------- glider.mjcf ----------------
write_mjcf(fullfile(outdir,'glider.mjcf'), D, Ib, fins);

% ---------------- geometry/README.md ----------------
write_geom_readme(fullfile(outdir,'README.md'), D, S_ref);

fprintf('  产物已写入 %s\n=== gen_airframe 完成 ===\n', outdir);
end

% ================= 局部函数 =================

function D = merge_struct(D, cfg)
fn = fieldnames(cfg);
for i = 1:numel(fn), D.(fn{i}) = cfg.(fn{i}); end
end

% --- 旋转体: 薄片积分惯量 + 三角网格 (端盖/收心扇自动) ---
function [Ib, V, F] = rev_body(xs, rs, D)
prof_x = linspace(xs(1), xs(end), D.n_sl+1);
prof_r = interp1(xs, rs, prof_x);
prof_r(prof_r < 0) = 0;
dx = prof_x(2) - prof_x(1);
w = dx*ones(size(prof_x));  w(1) = dx/2;  w(end) = dx/2;   % 复合梯形权重
rho_eff = D.m_body / (pi*sum(prof_r.^2.*w));   % 由总质量回推等效密度
dm = rho_eff*pi*prof_r.^2.*w;
com_x = sum(dm.*prof_x) / D.m_body;
Ixx = sum(0.5*dm.*prof_r.^2);
Iyy = sum(dm.*((prof_x-com_x).^2 + prof_r.^2/4));
Ib.com = [com_x, 0, 0];
Ib.I = diag([Ixx, Iyy, Iyy]);

% 网格: 实环序列 (收心零环不生成, 以心点扇封口)
comb = rs(end) < 1e-6;
grid_x = xs;  grid_r = rs;
if comb, grid_x(end) = [];  grid_r(end) = []; end
n_th = D.n_th;
th = (0:n_th-1)' * 2*pi/n_th;
V = zeros(numel(grid_x)*n_th, 3);
for k = 1:numel(grid_x)
    V((k-1)*n_th+(1:n_th), :) = [repmat(grid_x(k), n_th,1), grid_r(k)*cos(th), grid_r(k)*sin(th)];
end
F = zeros(0, 3);
for k = 1:numel(grid_x)-1
    i0 = (k-1)*n_th;  i1 = k*n_th;
    for j = 1:n_th
        j2 = j + 1;  if j2 > n_th, j2 = 1; end
        F = [F; i0+j, i0+j2, i1+j2; i0+j, i1+j2, i1+j];   % 法向朝外 (θ̂×x̂=r̂)
    end
end
% 尾盖 (x=x_t, 法向 -x̂)
V = [V; grid_x(1), 0, 0];  c = size(V,1);
for j = 1:n_th
    j2 = j + 1;  if j2 > n_th, j2 = 1; end
    F = [F; c, j2, j];
end
% 头部: flat 端盖 / cone 收心扇 (均法向 +x̂)
V = [V; xs(end), 0, 0];  c = size(V,1);  i1 = (numel(grid_x)-1)*n_th;
for j = 1:n_th
    j2 = j + 1;  if j2 > n_th, j2 = 1; end
    F = [F; c, i1+j, i1+j2];
end
end

function v = vol_nose_trim(D)
% 从"全长圆柱"中扣除的头部体积差: flat 0; cone 圆柱段被锥替代, 差 = πR²·L_nose·(2/3)
if strcmpi(D.nose, 'flat'), v = 0;
else, v = pi*D.x_R^2*D.nose_len*2/3; end
end

function I = box_inertia(m, dims)
% 盒绕自身质心主惯量 (dims = [x y z] 尺寸)
a = dims(1);  b = dims(2);  c = dims(3);
I = diag(m/12*[b^2+c^2, a^2+c^2, a^2+b^2]);
end

function [V, F] = box_mesh(lo, dims)
% 盒 8 顶点 (lo = 角点), 12 三角, 法向朝外
a = dims(1);  b = dims(2);  c = dims(3);
V = lo + [0 0 0; a 0 0; a b 0; 0 b 0; 0 0 c; a 0 c; a b c; 0 b c];
F = [1 3 2; 1 4 3;    % -z
     5 6 7; 5 7 8;    % +z
     1 2 6; 1 6 5;    % -y
     4 8 7; 4 7 3;    % +y
     2 3 7; 2 7 6;    % +x (迎流端)
     1 5 8; 1 8 4];   % -x
end

function write_stl(fname, V, F)
stlwrite(triangulation(F, V), fname);
end

function vol = mesh_volume(V, F)
% 散度定理: V = Σ det[v1 v2 v3]/6 (闭合且法向朝外时为正)
vol = sum(dot(V(F(:,1),:), cross(V(F(:,2),:), V(F(:,3),:)), 2))/6;
end

function write_mjcf(fname, D, Ib, fins)
% MuJoCo 模型骨架: body 树与占位 URDF 同构 (弹体 + 4 舵各自质量),
% 气动力矩参考点用全系统合成 (airframe_params.json 的 system 字段)。
r20 = 0.349066;   % ±20° (spec §4.2)
f = fins;   % 缩写
txt = sprintf([ ...
'<mujoco model="glider_v1">\n' ...
'  <!-- CFD 侧参数化几何产物 (gen_airframe.m v1) — B 系 x前弹轴 y右 z下\n' ...
'       默认参数 = 占位 glider_phase0.urdf 咬合; 零舵偏 STL 见同目录 fin_*.stl -->\n' ...
'  <compiler angle="radian" meshdir="."/>\n' ...
'  <option timestep="0.002"/>   <!-- spec §3: 主循环 dt=2ms -->\n\n' ...
'  <asset>\n    <mesh name="body" file="body.stl"/>\n  </asset>\n\n' ...
'  <worldbody>\n' ...
'    <body name="airframe" pos="0 0 0">\n' ...
'      <freejoint name="root"/>\n' ...
'      <inertial pos="%.4g 0 0" mass="%.4g" fullinertia="%.6g %.6g %.6g 0 0 0"/>\n' ...
'      <geom name="body_geom" type="mesh" mesh="body" rgba="0.45 0.45 0.5 1"/>\n\n' ...
'      <!-- 水平对: 铰链轴 y (同向=俯仰 de, 差动=滚转 da) -->\n' ...
'      <body name="fin_h_pos" pos="%.4g 0 %.4g">\n' ...
'        <joint name="j_h_pos" type="hinge" axis="0 1 0" range="-%.4g %.4g"/>\n' ...
'        <inertial pos="%.4g 0 %.4g" mass="%.4g" fullinertia="%.6g %.6g %.6g 0 0 0"/>\n' ...
'        <geom name="g_h_pos" type="box" size="%.4g %.4g %.4g" pos="%.4g 0 %.4g" rgba="0.9 0.5 0.1 1"/>\n' ...
'      </body>\n' ...
'      <body name="fin_h_neg" pos="%.4g 0 %.4g">\n' ...
'        <joint name="j_h_neg" type="hinge" axis="0 1 0" range="-%.4g %.4g"/>\n' ...
'        <inertial pos="%.4g 0 %.4g" mass="%.4g" fullinertia="%.6g %.6g %.6g 0 0 0"/>\n' ...
'        <geom name="g_h_neg" type="box" size="%.4g %.4g %.4g" pos="%.4g 0 %.4g" rgba="0.9 0.5 0.1 1"/>\n' ...
'      </body>\n\n' ...
'      <!-- 垂直对: 铰链轴 z (同向=偏航 dr) -->\n' ...
'      <body name="fin_v_pos" pos="%.4g %.4g 0">\n' ...
'        <joint name="j_v_pos" type="hinge" axis="0 0 1" range="-%.4g %.4g"/>\n' ...
'        <inertial pos="%.4g %.4g 0" mass="%.4g" fullinertia="%.6g %.6g %.6g 0 0 0"/>\n' ...
'        <geom name="g_v_pos" type="box" size="%.4g %.4g %.4g" pos="%.4g %.4g 0" rgba="0.9 0.5 0.1 1"/>\n' ...
'      </body>\n' ...
'      <body name="fin_v_neg" pos="%.4g %.4g 0">\n' ...
'        <joint name="j_v_neg" type="hinge" axis="0 0 1" range="-%.4g %.4g"/>\n' ...
'        <inertial pos="%.4g %.4g 0" mass="%.4g" fullinertia="%.6g %.6g %.6g 0 0 0"/>\n' ...
'        <geom name="g_v_neg" type="box" size="%.4g %.4g %.4g" pos="%.4g %.4g 0" rgba="0.9 0.5 0.1 1"/>\n' ...
'      </body>\n' ...
'    </body>\n' ...
'  </worldbody>\n' ...
'</mujoco>\n'], ...
Ib.com(1), D.m_body, Ib.I(1,1), Ib.I(2,2), Ib.I(3,3), ...
D.x_fin, D.fin_root, r20, r20, ...                                  % h_pos
f.h_pos.rel(1), f.h_pos.rel(3), D.m_fin, diag(f.h_pos.I), ...
abs(f.h_pos.rel(1)), D.fin_thick/2, D.fin_span/2, f.h_pos.rel(1), f.h_pos.rel(3), ...
D.x_fin, D.fin_root, r20, r20, ...                                  % h_neg
f.h_neg.rel(1), f.h_neg.rel(3), D.m_fin, diag(f.h_neg.I), ...
abs(f.h_neg.rel(1)), D.fin_thick/2, D.fin_span/2, f.h_neg.rel(1), f.h_neg.rel(3), ...
D.x_fin, D.fin_root, r20, r20, ...                                  % v_pos
f.v_pos.rel(1), f.v_pos.rel(2), D.m_fin, diag(f.v_pos.I), ...
abs(f.v_pos.rel(1)), D.fin_span/2, D.fin_thick/2, f.v_pos.rel(1), f.v_pos.rel(2), ...
D.x_fin, D.fin_root, r20, r20, ...                                  % v_neg
f.v_neg.rel(1), f.v_neg.rel(2), D.m_fin, diag(f.v_neg.I), ...
abs(f.v_neg.rel(1)), D.fin_span/2, D.fin_thick/2, f.v_neg.rel(1), f.v_neg.rel(2));
fid = fopen(fname, 'w');  fprintf(fid, '%s', txt);  fclose(fid);
end

function write_geom_readme(fname, D, S_ref)
txt = sprintf([ ...
'# geometry/ — 参数化弹体几何产物 (gen_airframe.m v1)\n\n' ...
'坐标系 B: x 前弹轴, y 右, z 下, 右手系, 原点=弹体几何中心。单位米。\n\n' ...
'| 文件 | 内容 | 消费方 |\n|---|---|---|\n' ...
'| body.stl | 弹体旋转体网格 (nose=%s) | snappyHexMesh / MJCF asset |\n' ...
'| fin_{h_pos,h_neg,v_pos,v_neg}.stl | 4 舵面盒网格 (弹体系绝对坐标, 零舵偏) | snappyHexMesh |\n' ...
'| airframe_params.json | 几何/质量惯量/无量纲化基准/舵面命名与混控 | 查表换算 / MJCF inertial |\n' ...
'| glider.mjcf | MuJoCo 骨架 (freejoint+4 舵 hinge, 舵面 box 图元) | 视觉侧直接加载 |\n\n' ...
'- 无量纲化基准 (待 Phase 0 冻结): S_ref=%.5f m² (弹身迎面积), L_ref=%.3f m (弹径), 力矩参考点=全系统质心。\n' ...
'- 舵面 STL 是零舵偏位形; MuJoCo 中舵面用 box 图元绕铰链转动, STL 仅供 CFD 装配与扫气标定。\n' ...
'- 改参数: gen_airframe(cfg) 覆盖字段 (见生成器头注释); MJCF 弹体 inertial 已按 profile 惯量写入 (cone 亦正确)。\n'], ...
D.nose, S_ref, 2*D.x_R);
fid = fopen(fname, 'w');  fprintf(fid, '%s', txt);  fclose(fid);
end
