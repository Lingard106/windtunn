%% export_fan_fluidx3d — MATLAB 风机 → FluidX3D GPU 求解衔接器 (参照 WindTunnel3D 动态渲染管线)
% 流程: fan_voxelize 体素化 → fan_sim.raw + fan_rotor.bin + setup_fan.cpp + build_fan.bat
% 求解器特性: 动壁叶轮 ω×r / 逐步重体素化旋转 / 自适应背压出口 / 流量-转矩诊断 / VTK 快照 / 时均
% 环境开关: WT_QUICK=1 冒烟 | WT_DX=晶格覆盖 | FAN_FROZEN=1 冻结转子 | FAN_FULL=1 全域
clear; clc;
P = fan_params();
if ~isempty(getenv('FAN_FROZEN')), P.gpu.rotate = false; end
G = fan_geometry(P);
R = G.R;
if ~isfinite(R.op.Q)
    error('1D 模型无工况点 — 先跑 fan_sweep 选型或调 fan_params');
end
L = R.load;

%% ---- GPU 档位 ----
GPU = 'full';
if ~isempty(getenv('WT_QUICK')), GPU = 'smoke'; end
switch GPU
    case 'smoke', dxg = 0.006; nSteps = 6000;  nSnap = 1000;
    otherwise,    dxg = 0.003; nSteps = 30000; nSnap = 1500;
end
dxEnv = getenv('WT_DX');
if ~isempty(dxEnv) && isfinite(str2double(dxEnv))
    dxg = min(0.01, max(0.0005, str2double(dxEnv)));
    fprintf('[WT_DX] 晶格覆盖: %.2f mm\n', dxg*1e3);
end
nDiag  = P.gpu.diag;
nRot   = P.gpu.rotEvery;
nAvgStart = floor(nSteps/2);
P.gpu.dx = dxg;
% 快照 VTK 档位: 'u' (默认, 只写 u 场) | 'all' (u+rho+F) — 详见下方快照块注释
vtkAll = ~isempty(getenv('WT_VTK')) && strcmpi(getenv('WT_VTK'), 'all');

%% ---- 体素化 + 旧结果归档 ----
[solidW, rot0, rotList, grid] = fan_voxelize(P, G);
nx = grid.nx; ny = grid.ny; nz = grid.nz;
fprintf('GPU 网格: %dx%dx%d = %.1fM 格 (dx=%.1fmm)\n', nx,ny,nz, nx*ny*nz/1e6, dxg*1e3);
vtkDir = fullfile(pwd,'fluidx3d','vtk');
if exist(vtkDir,'dir')
    dOld = [dir(fullfile(vtkDir,'u_*.vtk')); dir(fullfile(vtkDir,'rho_*.vtk')); dir(fullfile(vtkDir,'F_*.vtk'))];
    if ~isempty(dOld)
        prevDir = fullfile(pwd,'fluidx3d','vtk_prev');
        if ~exist(prevDir,'dir'), mkdir(prevDir); end
        for pat = {'u_*.vtk','rho_*.vtk','F_*.vtk'}
            dd = dir(fullfile(vtkDir,pat{1}));
            if ~isempty(dd), movefile(fullfile(vtkDir,pat{1}), prevDir); end
        end
        for fN = {'mean_u.vtk','tu.vtk'}
            if isfile(fullfile(vtkDir,fN{1})), movefile(fullfile(vtkDir,fN{1}), prevDir); end
        end
        fprintf('旧一代 VTK 已归档\n');
    end
end

%% ---- 单位换算 ----
uLx  = P.gpu.uLx;
Vref = P.load.V_test;
Sv   = Vref/uLx;
nuL  = max(uLx*(0.30/dxg)/P.gpu.Re, 0.01);   % τ=3ν+0.5≥0.53 稳定下限
dtP  = uLx*dxg/Vref;
dxP  = dxg;
rhoF = 1.204;
omega = 2*pi*P.imp.rpm/60;
omegaLx = omega*dtP;
fprintf('晶格: u=%.3f, nu=%.4f (Re=%d) | dt=%.2es | omega=%.4f rad/步 | 每转 %.0f 步\n', ...
    uLx, nuL, round(uLx*(0.30/dxg)/nuL), dtP, omegaLx, 2*pi/omegaLx);

%% ---- 写 RAW 与旋转区清单 ----
raw = zeros(nx,ny,nz,'uint8');
raw(solidW) = 1;
raw(rot0)   = 2;
fid = fopen(fullfile(pwd,'fan_sim.raw'),'wb');
fwrite(fid, raw, 'uint8');  fclose(fid);
fprintf('fan_sim.raw 已写 (%.1f MB) | 静止壁 %d 格 | 叶片 %d 格\n', ...
    numel(raw)/2^20, nnz(solidW), nnz(rot0));
fid = fopen(fullfile(pwd,'fan_rotor.bin'),'wb');
% 按列转 single 再写盘 (避免 3×N double 拼接临时; n<2^24 时 float32 精确, dx≥2.5mm 恒安全)
RLs = zeros(3, size(rotList,1), 'single');
RLs(1,:) = rotList(:,1);  RLs(2,:) = rotList(:,2)/dxg;  RLs(3,:) = rotList(:,3);
fwrite(fid, RLs, 'float32');  clear RLs
fclose(fid);
% 叶片子集 (θ=0 实体格): 冻结转子下 applyRotor/初始化只需它 (~10 万格, 非全包络 ~1000 万)
inBlade = rot0(rotList(:,1) + 1);
bladeList = rotList(inBlade, :);
fid = fopen(fullfile(pwd,'fan_blade.bin'),'wb');
RLs = zeros(3, size(bladeList,1), 'single');
RLs(1,:) = bladeList(:,1);  RLs(2,:) = bladeList(:,2)/dxg;  RLs(3,:) = bladeList(:,3);
fwrite(fid, RLs, 'float32');  clear RLs inBlade
fclose(fid);
fprintf('fan_rotor.bin 已写 (%d 扫掠区格点) | fan_blade.bin 已写 (%d 叶片格)\n', ...
    size(rotList,1), size(bladeList,1));

%% ---- 叶片几何 (晶格单位, 弧度; 半厚与 fan_voxelize 加厚一致) ----
bl = G.blade;
RaLx  = bl.Ra/dxg;
tBhLx = max(bl.tB/2, 0.6*dxg)/dxg;
C0yLx = bl.C(1)/dxg;
C0zLx = bl.C(2)/dxg;
pitchDeg = 360/bl.Z;
wrapDeg  = mod(bl.th(2)-bl.th(1), 360);
th1R   = deg2rad(bl.th(1));
wrapR  = deg2rad(wrapDeg);
pitchR = deg2rad(pitchDeg);
mMax   = ceil((wrapDeg + pitchDeg + 4)/pitchDeg);
xmLx   = nx - 4;
xcL    = (G.Lx.xBlade(1)+G.Lx.xBlade(2))/2/dxg;
if strcmp(grid.dom, 'full'), Kuse = L.Krear; else, Kuse = L.KafterFlange; end
dp2drho = 3/(rhoF*Sv^2);
Fscale  = rhoF*dxP^4/dtP^2;

%% ---- 生成 setup_fan.cpp (元胞逐行, 写入器自动加换行) ----
NL = sprintf('\n');
rotStr = 'true';
if ~P.gpu.rotate, rotStr = 'false'; end
C = {};
C{end+1} = '#include "setup.hpp"';
C{end+1} = '#include <vector>';
C{end+1} = 'void writeVTKLike(const char* fname, const float* data, const uint ncomp, const uint Nx, const uint Ny, const uint Nz);';
C{end+1} = ['void main_setup() { // auto-generated (dx=' num2str(dxP*1e3,'%.1f') 'mm, dt=' num2str(dtP,'%.2e') 's)'];
C{end+1} = ['	LBM lbm(' num2str(nx) 'u, ' num2str(ny) 'u, ' num2str(nz) 'u, ' num2str(nuL,'%.6f') 'f);'];
C{end+1} = '	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz();';
C{end+1} = '	uchar* raw = new uchar[lbm.get_N()];';
C{end+1} = '	FILE* f = fopen("fan_sim.raw","rb");';
C{end+1} = '	if(!f) { print_error("fan_sim.raw not found"); exit(1); }';
C{end+1} = '	if(fread(raw, 1, lbm.get_N(), f)!=lbm.get_N()) { print_error("raw size mismatch"); exit(1); }';
C{end+1} = '	fclose(f);';
C{end+1} = '	parallel_for(lbm.get_N(), [&](ulong n) {';
C{end+1} = '		lbm.flags[n] = raw[n]==0u ? 0u : (raw[n]==2u ? (TYPE_S|TYPE_X) : TYPE_S);';
C{end+1} = '	});';
C{end+1} = '	// inlet x=0: still atmosphere; outlet x=Nx-1: adaptive back-pressure (仅流体格)';
C{end+1} = '	parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);';
C{end+1} = '		if(x==0u && raw[n]!=1u && raw[n]!=2u)    { lbm.flags[n]=TYPE_E; lbm.rho[n]=1.0f; lbm.u.x[n]=0.0f; lbm.u.y[n]=0.0f; lbm.u.z[n]=0.0f; }';
C{end+1} = '		if(x==Nx-1u && raw[n]!=1u && raw[n]!=2u) { lbm.flags[n]=TYPE_E; lbm.rho[n]=1.0f; lbm.u.x[n]=0.0f; lbm.u.y[n]=0.0f; lbm.u.z[n]=0.0f; }';
C{end+1} = '	});';
C{end+1} = ['	const float omegaFull = ' num2str(omegaLx,'%.7f') 'f; // 晶格角速度 rad/step'];
C{end+1} = ['	const float rotCy = ' num2str(ny/2,'%.4f') 'f, rotCz = ' num2str(nz/2,'%.4f') 'f; // 旋转中心 (晶格索引, 供转矩)'];
C{end+1} = ['	const float C0y=' num2str(C0yLx,'%.4f') 'f, C0z=' num2str(C0zLx,'%.4f') 'f, RaLx=' num2str(RaLx,'%.4f') 'f, tBh=' num2str(tBhLx,'%.4f') 'f; // 叶片0 (轴心偏移, 晶格)'];
C{end+1} = ['	const float th1=' num2str(th1R,'%.7f') 'f, wrap=' num2str(wrapR,'%.7f') 'f, pitch=' num2str(pitchR,'%.7f') 'f; const uint mMax=' num2str(mMax) 'u;'];
C{end+1} = ['	const bool ROTATE = ' rotStr '; // 冻结转子=false / 瞬态旋转=true'];
C{end+1} = ['	const ulong nRot = ' num2str(size(rotList,1)) 'u; // 扫掠包络 (旋转模式重体素化用)'];
C{end+1} = ['	const ulong nBlade = ' num2str(size(bladeList,1)) 'u; // θ=0 叶片实体格 (动壁速度用)'];
C{end+1} = '	float* rotData = nullptr;  float* bladeData = new float[3u*nBlade];';
C{end+1} = '	{ FILE* fb=fopen("fan_blade.bin","rb");';
C{end+1} = '	  if(!fb) { print_error("fan_blade.bin not found"); exit(1); }';
C{end+1} = '	  if(fread(bladeData, 4u, 3u*nBlade, fb)!=3u*nBlade) { print_error("blade bin size mismatch"); exit(1); } fclose(fb); }';
C{end+1} = '	if(ROTATE) {';
C{end+1} = '		rotData = new float[3u*nRot];';
C{end+1} = '		f=fopen("fan_rotor.bin","rb");';
C{end+1} = '		if(!f) { print_error("fan_rotor.bin not found"); exit(1); }';
C{end+1} = '		if(fread(rotData, 4u, 3u*nRot, f)!=3u*nRot) { print_error("rotor bin size mismatch"); exit(1); }';
C{end+1} = '		fclose(f);';
C{end+1} = '	}';
C{end+1} = '	uchar* rotState = ROTATE ? new uchar[nRot] : nullptr;';
C{end+1} = ['	const uint xm = ' num2str(xmLx) 'u;'];
C{end+1} = '	std::vector<ulong> meas;';
C{end+1} = '	for(uint z=0u; z<Nz; z++) for(uint y=0u; y<Ny; y++) { const ulong n=((ulong)z*Ny+(ulong)y)*Nx+(ulong)xm;';
C{end+1} = '		if(raw[n]!=1u) meas.push_back(n); }';
C{end+1} = '	// 出口平面流体格 (动量匹配外流 — u=0 固定会憋死出流)';
C{end+1} = '	std::vector<ulong> outp;';
C{end+1} = '	for(uint z=0u; z<Nz; z++) for(uint y=0u; y<Ny; y++) { const ulong n=((ulong)z*Ny+(ulong)y)*Nx+(ulong)(Nx-1u);';
C{end+1} = '		if(raw[n]!=1u) outp.push_back(n); }';
C{end+1} = '	print_info("measurement plane x="+to_string(xm)+" cells="+to_string((unsigned long long)meas.size()));';
C{end+1} = ['	const float velPhys = ' num2str(Sv,'%.4f') 'f;'];
C{end+1} = ['	const float dxPhys = ' num2str(dxP,'%.6f') 'f;'];
C{end+1} = '	const float Aout = (float)outp.size()*dxPhys*dxPhys;';
C{end+1} = ['	const float Kload = ' num2str(Kuse,'%.4f') 'f; // 自适应出口负载 Pa/(m^3/s)^2'];
C{end+1} = ['	const float dp2drho = ' num2str(dp2drho,'%.7f') 'f;'];
C{end+1} = ['	const float Tscale = ' num2str(Fscale*dxP,'%.7f') 'f; // lattice torque -> N*m'];
C{end+1} = ['	const float INVDT = ' num2str(1/dtP,'%.4f') 'f; // lattice rad/step -> 物理 rad/s'];
C{end+1} = ['	const ulong nSteps = ' num2str(nSteps) 'u, nAvgStart = ' num2str(nAvgStart) 'u;'];
C{end+1} = ['	const uint nDiag = ' num2str(nDiag) 'u, nRotEvery = ' num2str(nRot) 'u;'];
C{end+1} = '	float theta = 0.0f;  ulong tLastRot = 0ull;  float rhoOut = 1.0f;  float uxS = 0.0f;';
C{end+1} = '	auto bladeTest = [&](const float r, const float psi, const float th) { // 是否在任一叶片内';
C{end+1} = '		for(uint m=0u; m<mMax; m++) {';
C{end+1} = '			const float pm = fmodf(psi - th, pitch) + (float)m*pitch;';
C{end+1} = '			const float y = r*cosf(pm), z = r*sinf(pm);';
C{end+1} = '			const float d = sqrtf((y-C0y)*(y-C0y)+(z-C0z)*(z-C0z));';
C{end+1} = '			if(fabsf(d-RaLx) <= tBh) {';
C{end+1} = '				const float a = fmodf(atan2f(z-C0z, y-C0y) - th1 + 6.2831853f, 6.2831853f);';
C{end+1} = '				if(a <= wrap) return true;';
C{end+1} = '			}';
C{end+1} = '		}';
C{end+1} = '		return false;';
C{end+1} = '	};';
C{end+1} = '	auto applyRotor = [&](const float omega) { // 动壁速度: 只遍历叶片实体格 (每诊断步都写, 对齐官方动边界模式)';
C{end+1} = '		lbm.u.read_from_device();';
C{end+1} = '		parallel_for(nBlade, [&](ulong i) {';
C{end+1} = '			const ulong n = (ulong)bladeData[3u*i];';
C{end+1} = '			if(lbm.flags[n]==(TYPE_S|TYPE_X)) {';
C{end+1} = '				const float y = bladeData[3u*i+1u]*cosf(bladeData[3u*i+2u]), z = bladeData[3u*i+1u]*sinf(bladeData[3u*i+2u]);';
C{end+1} = '				lbm.u.x[n]=0.0f; lbm.u.y[n]=-omega*z; lbm.u.z[n]=omega*y;';
C{end+1} = '			}';
C{end+1} = '		});';
C{end+1} = '		lbm.u.write_to_device();';
C{end+1} = '	};';
C{end+1} = '	auto rotateUpdate = [&](const float omega) { // 重体素化: 叶片扫过区域翻转 (先读回再改)';
C{end+1} = '		lbm.flags.read_from_device();  lbm.u.read_from_device();  lbm.rho.read_from_device();';
C{end+1} = '		parallel_for(nRot, [&](ulong i) {';
C{end+1} = '			const float r = rotData[3u*i+1u], psi = rotData[3u*i+2u];';
C{end+1} = '			const ulong n = (ulong)rotData[3u*i];';
C{end+1} = '			const bool inB = bladeTest(r, psi, theta);';
C{end+1} = '			const uchar want = inB ? 1u : 0u;';
C{end+1} = '			if(want != rotState[i]) {';
C{end+1} = '				const float y = r*cosf(psi), z = r*sinf(psi);';
C{end+1} = '				lbm.flags[n] = want ? (TYPE_S|TYPE_X) : 0u;';
C{end+1} = '				lbm.rho[n]=1.0f; lbm.u.x[n]=0.0f; lbm.u.y[n]=-omega*z; lbm.u.z[n]=omega*y;';
C{end+1} = '				rotState[i] = want;';
C{end+1} = '			}';
C{end+1} = '		});';
C{end+1} = '		lbm.flags.write_to_device();  lbm.u.write_to_device();  lbm.rho.write_to_device();';
C{end+1} = '		lbm.update_moving_boundaries();';
C{end+1} = '	};';
C{end+1} = '	if(ROTATE) { // 初始叶轮状态 (θ=0, 仅旋转模式需要)';
C{end+1} = '		parallel_for(nRot, [&](ulong i) {';
C{end+1} = '			rotState[i] = bladeTest(rotData[3u*i+1u], rotData[3u*i+2u], 0.0f) ? 1u : 0u;';
C{end+1} = '		});';
C{end+1} = '	}';
C{end+1} = ['	FILE* fc = fopen("fan_diag.csv","w"); fprintf(fc, "t,Q_m3s,torque_Nm,power_W,rhoOut\n"); fclose(fc);'];
C{end+1} = '	float* uSum = new float[3u*lbm.get_N()]();  float* u2Sum = new float[3u*lbm.get_N()]();';
C{end+1} = '	ulong nAvg = 0ull;  float Qphys = 0.0f;';
C{end+1} = '	lbm.run(0u); // init first (official moving-boundary pattern)';
C{end+1} = '	applyRotor(omegaFull);';
C{end+1} = '	lbm.update_moving_boundaries();';
C{end+1} = ['	for(uint s=0u; s<nSteps; s+=' num2str(nSnap) 'u) {'];
C{end+1} = ['		for(uint ss=0u; ss<' num2str(nSnap) 'u; ss+=nDiag) {'];
C{end+1} = '			const float ramp = fminf(1.0f, (float)lbm.get_t()/(0.2f*(float)nSteps));';
C{end+1} = '			const float omega = omegaFull*ramp;';
C{end+1} = '			applyRotor(omega); // 每诊断步刷新 (ramp 后 omegaFull, 防 ramp 冻结在 91.7%)';
C{end+1} = '			if(ROTATE && lbm.get_t()>0ull && lbm.get_t()%nRotEvery==0ull) { theta += omegaFull*(float)(lbm.get_t()-tLastRot)*ramp; tLastRot = lbm.get_t(); rotateUpdate(omega); }';
C{end+1} = '			lbm.run(nDiag, nSteps);';
C{end+1} = '			// ---- 流量测量 + 自适应背压 (风机-风洞工况点自寻优) ----';
C{end+1} = '			lbm.u.read_from_device();';
C{end+1} = '			double qLx = 0.0;';
C{end+1} = '			for(ulong k=0ull; k<meas.size(); k++) qLx += (double)lbm.u.x[meas[k]];';
C{end+1} = '			Qphys = (float)qLx*velPhys*dxPhys*dxPhys;';
C{end+1} = '			const float dpLoad = Kload*Qphys*Qphys;';
C{end+1} = '			rhoOut = fmaxf(0.97f, 1.0f - 0.5f*dp2drho*dpLoad);';
C{end+1} = '			uxS += 0.1f*(Qphys/fmaxf(Aout,1e-9f) - uxS); // 有符号匹配 (消除动量吸收墙), 松弛 0.1';
C{end+1} = '			lbm.rho.read_from_device();';
C{end+1} = '			parallel_for((ulong)outp.size(), [&](ulong i) { const ulong n = outp[i];';
C{end+1} = '				lbm.rho[n] = rhoOut; lbm.u.x[n] = uxS; lbm.u.y[n] = 0.0f; lbm.u.z[n] = 0.0f; });';
C{end+1} = '			lbm.rho.write_to_device();  lbm.u.write_to_device();';
C{end+1} = '			// ---- 转矩/功率 ----';
C{end+1} = '			lbm.update_force_field();';
C{end+1} = ['			const float3 TQ = lbm.object_torque(float3(' num2str(xcL,'%.4f') 'f, rotCy, rotCz), TYPE_S|TYPE_X);'];
C{end+1} = '			const float tq = TQ.x*Tscale;';
C{end+1} = ['			fc = fopen("fan_diag.csv","a"); fprintf(fc, "%u,%.5f,%.4f,%.3f,%.5f\n", lbm.get_t(), Qphys, tq, tq*omega*INVDT, rhoOut); fclose(fc);'];
C{end+1} = '			if(lbm.get_t()>=nAvgStart) {';
C{end+1} = '				parallel_for(lbm.get_N(), [&](ulong n) { const float ux=lbm.u.x[n], uy=lbm.u.y[n], uz=lbm.u.z[n];';
C{end+1} = '					uSum[0u*lbm.get_N()+n]+=ux;  uSum[1u*lbm.get_N()+n]+=uy;  uSum[2u*lbm.get_N()+n]+=uz;';
C{end+1} = '					u2Sum[0u*lbm.get_N()+n]+=ux*ux;  u2Sum[1u*lbm.get_N()+n]+=uy*uy;  u2Sum[2u*lbm.get_N()+n]+=uz*uz; });';
C{end+1} = '				nAvg++;';
C{end+1} = '			}';
C{end+1} = '		}';
C{end+1} = '		lbm.update_force_field(); // [2026-09-30 修复] 快照前不再调 update_fields(): 其镜像重构损坏 (rho/u), 主内核每步已维护真实镜像';
C{end+1} = '		lbm.u.write_device_to_vtk("vtk/u_"+to_string(lbm.get_t()));';
if vtkAll
    C{end+1} = '		lbm.rho.write_device_to_vtk("vtk/rho_"+to_string(lbm.get_t()));';
    C{end+1} = '		lbm.F.write_device_to_vtk("vtk/F_"+to_string(lbm.get_t()));';
end
C{end+1} = '		print_info("snapshot t="+to_string(lbm.get_t())+" Q="+to_string(Qphys)+" m3/s rhoOut="+to_string(rhoOut));';
C{end+1} = '	}';
C{end+1} = '	if(nAvg>0ull) {';
C{end+1} = '		float* meanBuf = new float[3u*lbm.get_N()];  float* tuBuf = new float[lbm.get_N()];';
C{end+1} = '		for(ulong n=0ull; n<lbm.get_N(); n++) {';
C{end+1} = '			const float mx=uSum[0u*lbm.get_N()+n]/(float)nAvg, my=uSum[1u*lbm.get_N()+n]/(float)nAvg, mz=uSum[2u*lbm.get_N()+n]/(float)nAvg;';
C{end+1} = '			meanBuf[3u*n+0u]=mx;  meanBuf[3u*n+1u]=my;  meanBuf[3u*n+2u]=mz;';
C{end+1} = '			const float v2 = fmaxf(u2Sum[0u*lbm.get_N()+n]/(float)nAvg-mx*mx,0.0f)+fmaxf(u2Sum[1u*lbm.get_N()+n]/(float)nAvg-my*my,0.0f)+fmaxf(u2Sum[2u*lbm.get_N()+n]/(float)nAvg-mz*mz,0.0f);';
C{end+1} = ['			tuBuf[n] = 100.0f*sqrtf(v2)/' num2str(uLx,'%.5f') 'f;'];
C{end+1} = '		}';
C{end+1} = '		writeVTKLike("vtk/mean_u.vtk", meanBuf, 3u, Nx, Ny, Nz);';
C{end+1} = '		writeVTKLike("vtk/tu.vtk", tuBuf, 1u, Nx, Ny, Nz);';
C{end+1} = '		delete[] meanBuf;  delete[] tuBuf;';
C{end+1} = '	}';
C{end+1} = '	delete[] uSum;  delete[] u2Sum;  delete[] rotData;  delete[] rotState;  delete[] bladeData;  delete[] raw;';
C{end+1} = '	print_info("fan sim done - see vtk/ and fan_diag.csv");';
C{end+1} = '}';
C{end+1} = 'void writeVTKLike(const char* fname, const float* data, const uint ncomp, const uint Nx, const uint Ny, const uint Nz) {';
C{end+1} = '	FILE* f = fopen(fname, "wb");  if(!f) { print_error(std::string("cannot write ")+fname); return; }';
C{end+1} = '	char hdr[512];';
C{end+1} = ['	sprintf(hdr, "# vtk DataFile Version 3.0\nFanSim3D\nBINARY\nDATASET STRUCTURED_POINTS\nDIMENSIONS %u %u %u\nORIGIN %.8E %.8E %.8E\nSPACING 1.00000000 1.00000000 1.00000000\nPOINT_DATA %llu\nSCALARS data float %u\nLOOKUP_TABLE default\n", Nx, Ny, Nz, -0.5*(double)(Nx-1u), -0.5*(double)(Ny-1u), -0.5*(double)(Nz-1u), (unsigned long long)Nx*Ny*Nz, ncomp);'];
C{end+1} = '	fputs(hdr, f);';
C{end+1} = '	const ulong Ncell = (ulong)Nx*Ny*Nz;';
C{end+1} = '	float* buf = new float[Ncell*ncomp];';
C{end+1} = '	parallel_for(Ncell*ncomp, [&](ulong n) { union { float f; unsigned int u; } v; v.f = data[n]; v.u = (v.u>>24u)|((v.u>>8u)&0xFF00u)|((v.u<<8u)&0xFF0000u)|(v.u<<24u); buf[n] = v.f; });';
C{end+1} = '	fwrite(buf, 4u*Ncell*ncomp, 1u, f);  fclose(f);  delete[] buf;';
C{end+1} = '}';

fid = fopen(fullfile(pwd,'setup_fan.cpp'),'w');
for k = 1:numel(C)
    fprintf(fid, '%s\n', C{k});
end
fclose(fid);
fprintf('setup_fan.cpp 已生成 (%d 行, %d 步, 快照/诊断/重体素化 = %d/%d/%d 步)\n', numel(C), nSteps, nSnap, nDiag, nRot);

%% ---- 元数据 ----
meta = struct('dx',dxP, 'dt',dtP, 'velScale',Sv, 'nx',nx, 'ny',ny, 'nz',nz, ...
    'x0',grid.x0, 'V_ref',Vref, 'rho',rhoF, 'rpm',P.imp.rpm, 'omega',omega, ...
    'nSteps',nSteps, 'nSnap',nSnap, 'nDiag',nDiag, 'rotate',P.gpu.rotate, ...
    'dom',grid.dom, 'Kuse',Kuse, 'opQ',R.op.Q, 'opVts',R.op.Vts, 'opP',R.op.P, ...
    'blade',bl, 'Lx',G.Lx, 'nAvgStart',nAvgStart);
save(fullfile(pwd,'fan_meta.mat'), 'meta');
fprintf('fan_meta.mat 已写\n');

%% ---- 编译脚本 ----
batLines = strsplit('@echo off\ncall "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1\ncd /d %~dp0fluidx3d\ncopy /y %~dp0setup_fan.cpp src\setup_fan.cpp >nul\ncopy /y %~dp0fan_sim.raw fan_sim.raw >nul\ncopy /y %~dp0fan_rotor.bin fan_rotor.bin >nul\ncopy /y %~dp0fan_blade.bin fan_blade.bin >nul\ncl.exe /nologo /O2 /EHsc /std:c++17 /W0 /utf-8 /DGRAPHICS=0 /DFORCE_FIELD /DMOVING_BOUNDARIES /I src\OpenCL\include src\graphics.cpp src\info.cpp src\kernel.cpp src\lbm.cpp src\lodepng.cpp src\main.cpp src\setup_fan.cpp src\shapes.cpp /link /LIBPATH:src\OpenCL\lib OpenCL.lib /OUT:windfan.exe\necho BUILD_EXITCODE=%ERRORLEVEL%', '\\n');
fid = fopen(fullfile(pwd,'build_fan.bat'),'w');
fprintf(fid, '%s\r\n', batLines{:});
fclose(fid);
if ~exist(fullfile(pwd,'fluidx3d','src'),'dir')
    mkdir(fullfile(pwd,'fluidx3d'));
    copyfile('D:\WindTunnel3D\fluidx3d\src', fullfile(pwd,'fluidx3d','src'));
    fprintf('fluidx3d 源码已从 WindTunnel3D 复制\n');
end
if ~exist(fullfile(pwd,'fluidx3d','vtk'),'dir'), mkdir(fullfile(pwd,'fluidx3d','vtk')); end
fprintf('build_fan.bat 已生成\n--- 衔接完成 ---\n');

