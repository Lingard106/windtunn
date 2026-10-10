#include "setup.hpp"
#include <vector>
void writeVTKLike(const char* fname, const float* data, const uint ncomp, const uint Nx, const uint Ny, const uint Nz);
void main_setup() { // auto-generated (dx=3.0mm, dt=1.20e-05s)
	LBM lbm(231u, 362u, 362u, 0.010000f);
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz();
	uchar* raw = new uchar[lbm.get_N()];
	FILE* f = fopen("fan_sim.raw","rb");
	if(!f) { print_error("fan_sim.raw not found"); exit(1); }
	if(fread(raw, 1, lbm.get_N(), f)!=lbm.get_N()) { print_error("raw size mismatch"); exit(1); }
	fclose(f);
	parallel_for(lbm.get_N(), [&](ulong n) {
		lbm.flags[n] = raw[n]==0u ? 0u : (raw[n]==2u ? (TYPE_S|TYPE_X) : TYPE_S);
	});
	// inlet x=0: still atmosphere; outlet x=Nx-1: adaptive back-pressure (仅流体格)
	parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(x==0u && raw[n]!=1u && raw[n]!=2u)    { lbm.flags[n]=TYPE_E; lbm.rho[n]=1.0f; lbm.u.x[n]=0.0f; lbm.u.y[n]=0.0f; lbm.u.z[n]=0.0f; }
		if(x==Nx-1u && raw[n]!=1u && raw[n]!=2u) { lbm.flags[n]=TYPE_E; lbm.rho[n]=1.0f; lbm.u.x[n]=0.0f; lbm.u.y[n]=0.0f; lbm.u.z[n]=0.0f; }
	});
	const float omegaFull = 0.0012064f; // 晶格角速度 rad/step
	const float rotCy = 181.0000f, rotCz = 181.0000f; // 旋转中心 (晶格索引, 供转矩)
	const float C0y=-29.0123f, C0z=93.4102f, RaLx=119.8011f, tBh=0.6000f; // 叶片0 (轴心偏移, 晶格)
	const float th1=-0.8942029f, wrap=0.3167727f, pitch=0.1963495f; const uint mMax=3u;
	const bool ROTATE = false; // 冻结转子=false / 瞬态旋转=true
	const ulong nRot = 9955840u; // 扫掠包络 (旋转模式重体素化用)
	const ulong nBlade = 114560u; // θ=0 叶片实体格 (动壁速度用)
	float* rotData = nullptr;  float* bladeData = new float[3u*nBlade];
	{ FILE* fb=fopen("fan_blade.bin","rb");
	  if(!fb) { print_error("fan_blade.bin not found"); exit(1); }
	  if(fread(bladeData, 4u, 3u*nBlade, fb)!=3u*nBlade) { print_error("blade bin size mismatch"); exit(1); } fclose(fb); }
	if(ROTATE) {
		rotData = new float[3u*nRot];
		f=fopen("fan_rotor.bin","rb");
		if(!f) { print_error("fan_rotor.bin not found"); exit(1); }
		if(fread(rotData, 4u, 3u*nRot, f)!=3u*nRot) { print_error("rotor bin size mismatch"); exit(1); }
		fclose(f);
	}
	uchar* rotState = ROTATE ? new uchar[nRot] : nullptr;
	const uint xm = 227u;
	std::vector<ulong> meas;
	for(uint z=0u; z<Nz; z++) for(uint y=0u; y<Ny; y++) { const ulong n=((ulong)z*Ny+(ulong)y)*Nx+(ulong)xm;
		if(raw[n]!=1u) meas.push_back(n); }
	// 出口平面流体格 (动量匹配外流 — u=0 固定会憋死出流)
	std::vector<ulong> outp;
	for(uint z=0u; z<Nz; z++) for(uint y=0u; y<Ny; y++) { const ulong n=((ulong)z*Ny+(ulong)y)*Nx+(ulong)(Nx-1u);
		if(raw[n]!=1u) outp.push_back(n); }
	print_info("measurement plane x="+to_string(xm)+" cells="+to_string((unsigned long long)meas.size()));
	const float velPhys = 250.0000f;
	const float dxPhys = 0.003000f;
	const float Aout = (float)outp.size()*dxPhys*dxPhys;
	const float Kload = 129.4165f; // 自适应出口负载 Pa/(m^3/s)^2
	const float dp2drho = 0.0000399f;
	const float Tscale = 0.0020317f; // lattice torque -> N*m
	const float INVDT = 83333.3333f; // lattice rad/step -> 物理 rad/s
	const ulong nSteps = 30000u, nAvgStart = 15000u;
	const uint nDiag = 100u, nRotEvery = 120u;
	float theta = 0.0f;  ulong tLastRot = 0ull;  float rhoOut = 1.0f;  float uxS = 0.0f;
	auto bladeTest = [&](const float r, const float psi, const float th) { // 是否在任一叶片内
		for(uint m=0u; m<mMax; m++) {
			const float pm = fmodf(psi - th, pitch) + (float)m*pitch;
			const float y = r*cosf(pm), z = r*sinf(pm);
			const float d = sqrtf((y-C0y)*(y-C0y)+(z-C0z)*(z-C0z));
			if(fabsf(d-RaLx) <= tBh) {
				const float a = fmodf(atan2f(z-C0z, y-C0y) - th1 + 6.2831853f, 6.2831853f);
				if(a <= wrap) return true;
			}
		}
		return false;
	};
	auto applyRotor = [&](const float omega) { // 动壁速度: 只遍历叶片实体格 (每诊断步都写, 对齐官方动边界模式)
		lbm.u.read_from_device();
		parallel_for(nBlade, [&](ulong i) {
			const ulong n = (ulong)bladeData[3u*i];
			if(lbm.flags[n]==(TYPE_S|TYPE_X)) {
				const float y = bladeData[3u*i+1u]*cosf(bladeData[3u*i+2u]), z = bladeData[3u*i+1u]*sinf(bladeData[3u*i+2u]);
				lbm.u.x[n]=0.0f; lbm.u.y[n]=-omega*z; lbm.u.z[n]=omega*y;
			}
		});
		lbm.u.write_to_device();
	};
	auto rotateUpdate = [&](const float omega) { // 重体素化: 叶片扫过区域翻转 (先读回再改)
		lbm.flags.read_from_device();  lbm.u.read_from_device();  lbm.rho.read_from_device();
		parallel_for(nRot, [&](ulong i) {
			const float r = rotData[3u*i+1u], psi = rotData[3u*i+2u];
			const ulong n = (ulong)rotData[3u*i];
			const bool inB = bladeTest(r, psi, theta);
			const uchar want = inB ? 1u : 0u;
			if(want != rotState[i]) {
				const float y = r*cosf(psi), z = r*sinf(psi);
				lbm.flags[n] = want ? (TYPE_S|TYPE_X) : 0u;
				lbm.rho[n]=1.0f; lbm.u.x[n]=0.0f; lbm.u.y[n]=-omega*z; lbm.u.z[n]=omega*y;
				rotState[i] = want;
			}
		});
		lbm.flags.write_to_device();  lbm.u.write_to_device();  lbm.rho.write_to_device();
		lbm.update_moving_boundaries();
	};
	if(ROTATE) { // 初始叶轮状态 (θ=0, 仅旋转模式需要)
		parallel_for(nRot, [&](ulong i) {
			rotState[i] = bladeTest(rotData[3u*i+1u], rotData[3u*i+2u], 0.0f) ? 1u : 0u;
		});
	}
	FILE* fc = fopen("fan_diag.csv","w"); fprintf(fc, "t,Q_m3s,torque_Nm,power_W,rhoOut\n"); fclose(fc);
	float* uSum = new float[3u*lbm.get_N()]();  float* u2Sum = new float[3u*lbm.get_N()]();
	ulong nAvg = 0ull;  float Qphys = 0.0f;
	lbm.run(0u); // init first (official moving-boundary pattern)
	applyRotor(omegaFull);
	lbm.update_moving_boundaries();
	for(uint s=0u; s<nSteps; s+=1500u) {
		for(uint ss=0u; ss<1500u; ss+=nDiag) {
			const float ramp = fminf(1.0f, (float)lbm.get_t()/(0.2f*(float)nSteps));
			const float omega = omegaFull*ramp;
			applyRotor(omega); // 每诊断步刷新 (ramp 后 omegaFull, 防 ramp 冻结在 91.7%)
			if(ROTATE && lbm.get_t()>0ull && lbm.get_t()%nRotEvery==0ull) { theta += omegaFull*(float)(lbm.get_t()-tLastRot)*ramp; tLastRot = lbm.get_t(); rotateUpdate(omega); }
			lbm.run(nDiag, nSteps);
			// ---- 流量测量 + 自适应背压 (风机-风洞工况点自寻优) ----
			lbm.u.read_from_device();
			double qLx = 0.0;
			for(ulong k=0ull; k<meas.size(); k++) qLx += (double)lbm.u.x[meas[k]];
			Qphys = (float)qLx*velPhys*dxPhys*dxPhys;
			const float dpLoad = Kload*Qphys*Qphys;
			rhoOut = fmaxf(0.97f, 1.0f - 0.5f*dp2drho*dpLoad);
			uxS += 0.1f*(Qphys/fmaxf(Aout,1e-9f) - uxS); // 有符号匹配 (消除动量吸收墙), 松弛 0.1
			lbm.rho.read_from_device();
			parallel_for((ulong)outp.size(), [&](ulong i) { const ulong n = outp[i];
				lbm.rho[n] = rhoOut; lbm.u.x[n] = uxS; lbm.u.y[n] = 0.0f; lbm.u.z[n] = 0.0f; });
			lbm.rho.write_to_device();  lbm.u.write_to_device();
			// ---- 转矩/功率 ----
			lbm.update_force_field();
			const float3 TQ = lbm.object_torque(float3(71.3333f, rotCy, rotCz), TYPE_S|TYPE_X);
			const float tq = TQ.x*Tscale;
			fc = fopen("fan_diag.csv","a"); fprintf(fc, "%u,%.5f,%.4f,%.3f,%.5f\n", lbm.get_t(), Qphys, tq, tq*omega*INVDT, rhoOut); fclose(fc);
			if(lbm.get_t()>=nAvgStart) {
				parallel_for(lbm.get_N(), [&](ulong n) { const float ux=lbm.u.x[n], uy=lbm.u.y[n], uz=lbm.u.z[n];
					uSum[0u*lbm.get_N()+n]+=ux;  uSum[1u*lbm.get_N()+n]+=uy;  uSum[2u*lbm.get_N()+n]+=uz;
					u2Sum[0u*lbm.get_N()+n]+=ux*ux;  u2Sum[1u*lbm.get_N()+n]+=uy*uy;  u2Sum[2u*lbm.get_N()+n]+=uz*uz; });
				nAvg++;
			}
		}
		lbm.update_force_field(); // [2026-09-30 修复] 快照前不再调 update_fields(): 其镜像重构损坏 (rho/u), 主内核每步已维护真实镜像
		lbm.u.write_device_to_vtk("vtk/u_"+to_string(lbm.get_t()));
		print_info("snapshot t="+to_string(lbm.get_t())+" Q="+to_string(Qphys)+" m3/s rhoOut="+to_string(rhoOut));
	}
	if(nAvg>0ull) {
		float* meanBuf = new float[3u*lbm.get_N()];  float* tuBuf = new float[lbm.get_N()];
		for(ulong n=0ull; n<lbm.get_N(); n++) {
			const float mx=uSum[0u*lbm.get_N()+n]/(float)nAvg, my=uSum[1u*lbm.get_N()+n]/(float)nAvg, mz=uSum[2u*lbm.get_N()+n]/(float)nAvg;
			meanBuf[3u*n+0u]=mx;  meanBuf[3u*n+1u]=my;  meanBuf[3u*n+2u]=mz;
			const float v2 = fmaxf(u2Sum[0u*lbm.get_N()+n]/(float)nAvg-mx*mx,0.0f)+fmaxf(u2Sum[1u*lbm.get_N()+n]/(float)nAvg-my*my,0.0f)+fmaxf(u2Sum[2u*lbm.get_N()+n]/(float)nAvg-mz*mz,0.0f);
			tuBuf[n] = 100.0f*sqrtf(v2)/0.10000f;
		}
		writeVTKLike("vtk/mean_u.vtk", meanBuf, 3u, Nx, Ny, Nz);
		writeVTKLike("vtk/tu.vtk", tuBuf, 1u, Nx, Ny, Nz);
		delete[] meanBuf;  delete[] tuBuf;
	}
	delete[] uSum;  delete[] u2Sum;  delete[] rotData;  delete[] rotState;  delete[] bladeData;  delete[] raw;
	print_info("fan sim done - see vtk/ and fan_diag.csv");
}
void writeVTKLike(const char* fname, const float* data, const uint ncomp, const uint Nx, const uint Ny, const uint Nz) {
	FILE* f = fopen(fname, "wb");  if(!f) { print_error(std::string("cannot write ")+fname); return; }
	char hdr[512];
	sprintf(hdr, "# vtk DataFile Version 3.0\nFanSim3D\nBINARY\nDATASET STRUCTURED_POINTS\nDIMENSIONS %u %u %u\nORIGIN %.8E %.8E %.8E\nSPACING 1.00000000 1.00000000 1.00000000\nPOINT_DATA %llu\nSCALARS data float %u\nLOOKUP_TABLE default\n", Nx, Ny, Nz, -0.5*(double)(Nx-1u), -0.5*(double)(Ny-1u), -0.5*(double)(Nz-1u), (unsigned long long)Nx*Ny*Nz, ncomp);
	fputs(hdr, f);
	const ulong Ncell = (ulong)Nx*Ny*Nz;
	float* buf = new float[Ncell*ncomp];
	parallel_for(Ncell*ncomp, [&](ulong n) { union { float f; unsigned int u; } v; v.f = data[n]; v.u = (v.u>>24u)|((v.u>>8u)&0xFF00u)|((v.u<<8u)&0xFF0000u)|(v.u<<24u); buf[n] = v.f; });
	fwrite(buf, 4u*Ncell*ncomp, 1u, f);  fclose(f);  delete[] buf;
}
