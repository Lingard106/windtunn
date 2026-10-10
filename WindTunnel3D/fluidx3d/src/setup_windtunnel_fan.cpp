#include "setup.hpp"
#include <vector>
void writeVTKLike(const char* fname, const float* data, const uint ncomp, const uint Nx, const uint Ny, const uint Nz);
void main_setup() { // auto-generated (dx=3.0mm) 风机侧置: 进气+y/叶轮绕y/蜗壳切向+x
	LBM lbm(984u, 303u, 395u, 0.016330f);
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz();
	uchar* raw = new uchar[lbm.get_N()];
	FILE* f = fopen("windtunnel.raw","rb");
	if(!f) { print_error("windtunnel.raw not found"); exit(1); }
	if(fread(raw, 1, lbm.get_N(), f)!=lbm.get_N()) { print_error("raw size mismatch"); exit(1); }
	fclose(f);
	parallel_for(lbm.get_N(), [&](ulong n) { // 0流体 1壁 2叶轮(S|X) 3测试物(S|Y)
		lbm.flags[n] = raw[n]==0u ? 0u : (raw[n]==2u ? (TYPE_S|TYPE_X) : (raw[n]==3u ? (TYPE_S|TYPE_Y) : TYPE_S));
	});
	// 增压箱大气盖(+y 域面, 气体沿 -y 抽入) / 出口自适应背压+外流速度 (仅流体格)
	parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(y==Ny-1u && raw[n]==0u) { lbm.flags[n]=TYPE_E; lbm.rho[n]=1.0f; lbm.u.x[n]=0.0f; lbm.u.y[n]=0.0f; lbm.u.z[n]=0.0f; }
		if(x==Nx-1u && raw[n]==0u) { lbm.flags[n]=TYPE_E; lbm.rho[n]=1.0f; lbm.u.x[n]=0.0f; lbm.u.y[n]=0.0f; lbm.u.z[n]=0.0f; }
	});
	const float omegaFull = -0.0009651f; // 绕 +y, 符号与蜗壳螺旋配对
	const float rotPx = 181.3514f, rotPz = 213.6927f; // 转矩枢轴 (晶格系, xoz 平面)
	const ulong nBlade = 117520u;
	float* bladeData = new float[3u*nBlade];
	f=fopen("fan_blade.bin","rb");
	if(!f) { print_error("fan_blade.bin not found"); exit(1); }
	if(fread(bladeData, 4u, 3u*nBlade, f)!=3u*nBlade) { print_error("blade bin mismatch"); exit(1); }
	fclose(f);
	const bool ROTATE = 0u==1u; // WT_ROT=1: 瞬态旋转叶轮 (重体素化); false=冻结转子
	const float C0x=-29.0123f, C0z=93.4102f, RaLx=119.8011f, tBh=0.6000f; // 叶片0 圆心/半径/半厚 (晶格)
	const float th1=-0.8942029f, wrap=0.3167727f, pitch=0.1963495f; const uint mMax=3u;
	const float spinS = 1.0f; const uint nRotEvery = 10u;
	const ulong nRotList = 0u;
	float* rotData = nullptr;  uchar* rotState = nullptr;
	if(ROTATE) {
		rotData = new float[3u*nRotList];
		f=fopen("fan_rotor.bin","rb");
		if(!f) { print_error("fan_rotor.bin not found"); exit(1); }
		if(fread(rotData, 4u, 3u*nRotList, f)!=3u*nRotList) { print_error("rotor bin mismatch"); exit(1); }
		fclose(f);
		rotState = new uchar[nRotList];
	}
	const uint xm = 886u; // 流量测量面
	std::vector<ulong> meas;
	for(uint z=0u; z<Nz; z++) for(uint y=0u; y<Ny; y++) { const ulong n=((ulong)z*Ny+(ulong)y)*Nx+(ulong)xm;
		if(raw[n]==0u) meas.push_back(n); }
	// 出口平面流体格 (动量匹配外流边界: E 的 u 固定 0 会形成动量吸收墙憋死出流)
	std::vector<ulong> outp;
	for(uint z=0u; z<Nz; z++) for(uint y=0u; y<Ny; y++) { const ulong n=((ulong)z*Ny+(ulong)y)*Nx+(ulong)(Nx-1u);
		if(raw[n]==0u) outp.push_back(n); }
	const float velPhys = 312.5000f;
	const float uxInit = 0.006400f; // [初始正向流] +2 m/s 推入正向分支 (devlog 2026-10-02)
	parallel_for(lbm.get_N(), [&](ulong n) { if(raw[n]==0u) lbm.u.x[n] = uxInit; });
	const float dxPhys = 0.003000f;
	const float Aout = (float)outp.size()*dxPhys*dxPhys;
	const float Kload = 95.8504f;
	const float dp2drho = 0.0000255f;
	const float Tscale = 0.0031746f;
	const float Fscale = 1.058203f;
	const ulong nSteps = 30000u, nAvgStart = 15000u;
	const uint nDiag = 100u;
	float rhoOut = 1.0f;  float uxS = 0.0f;  float theta = 0.0f;  ulong tLastRot = 0ull; // theta: 叶轮累积转角 (Θ系, rad)
	auto applyRotor = [&](const float omega) { // 冻结转子动壁 (绕 +y: u=(w*z, 0, -w*x), 每次都写, 对齐官方动边界模式)
		lbm.u.read_from_device();
		parallel_for(nBlade, [&](ulong i) {
			const ulong n = (ulong)bladeData[3u*i];
			if(lbm.flags[n]==(TYPE_S|TYPE_X)) {
				const float xr = bladeData[3u*i+1u]*cosf(bladeData[3u*i+2u]), zr = bladeData[3u*i+1u]*sinf(bladeData[3u*i+2u]); // 轴心系 (xoz 平面)
				lbm.u.x[n]=omega*zr; lbm.u.y[n]=0.0f; lbm.u.z[n]=-omega*xr;
			}
		});
		lbm.u.write_to_device();
	};
	auto bladeTest = [&](const float r, const float psiT, const float th) { // Θ系极角下是否在任一叶片内 (xoz 平面)
		for(uint m=0u; m<mMax; m++) {
			const float pm = fmodf(psiT - th, pitch) + (float)m*pitch;
			const float x = r*cosf(pm), z = r*sinf(pm);
			const float dd = sqrtf((x-C0x)*(x-C0x)+(z-C0z)*(z-C0z));
			if(fabsf(dd-RaLx) <= tBh) {
				const float a = fmodf(atan2f(z-C0z, x-C0x) - th1 + 6.2831853f, 6.2831853f);
				if(a <= wrap) return true;
			}
		}
		return false;
	};
	auto rotateUpdate = [&](const float omega) { // 重体素化: 扫掠包络内逐格翻转 叶片<->流体 (先读回再改)
		lbm.flags.read_from_device();  lbm.u.read_from_device();  lbm.rho.read_from_device();
		parallel_for(nRotList, [&](ulong i) {
			const float r = rotData[3u*i+1u], phiP = rotData[3u*i+2u];
			const ulong n = (ulong)rotData[3u*i];
			const float psiT = fmodf(spinS*phiP + 25.1327412f, 6.2831853f); // 物理角 -> Θ系 (保正)
			const bool inB = bladeTest(r, psiT, theta);
			const uchar want = inB ? 1u : 0u;
			if(want != rotState[i]) {
				const float x = r*cosf(phiP), z = r*sinf(phiP);
				lbm.flags[n] = want ? (TYPE_S|TYPE_X) : 0u;
				if(want) { lbm.u.x[n]=omega*z; lbm.u.y[n]=0.0f; lbm.u.z[n]=-omega*x; } // 新叶片格: 动壁速度
				else      { lbm.u.x[n]=0.0f;      lbm.u.y[n]=0.0f; lbm.u.z[n]=0.0f;      } // 新暴露流体格: 静止
				// [修复] rho 保留原值不硬设 1.0 — 密度跳变是二次冲击源
				rotState[i] = want;
			}
		});
		lbm.flags.write_to_device();  lbm.u.write_to_device();  lbm.rho.write_to_device();
		lbm.update_moving_boundaries();
	};
	FILE* fc = fopen("fan_diag.csv","w"); fprintf(fc, "t,Q_m3s,torque_Nm,power_W,rhoOut\n"); fclose(fc);
	FILE* fo = fopen("forces.csv","w"); fprintf(fo, "t_lattice,Fx,Fy,Fz\n"); fclose(fo);
	float* uSum = new float[3u*lbm.get_N()]();  float* u2Sum = new float[3u*lbm.get_N()]();
	ulong nAvg = 0ull;  float Qphys = 0.0f;
	lbm.run(0u); // 初始化
	if(ROTATE) { parallel_for(nRotList, [&](ulong i) { rotState[i] = bladeTest(rotData[3u*i+1u], fmodf(spinS*rotData[3u*i+2u]+25.1327412f, 6.2831853f), 0.0f) ? 1u : 0u; }); }
	applyRotor(omegaFull);
	lbm.update_moving_boundaries();
	for(uint s=0u; s<nSteps; s+=750u) {
		for(uint ss=0u; ss<750u; ss+=nDiag) {
			const float ramp = fminf(1.0f, (float)lbm.get_t()/(0.1f*(float)nSteps));
			applyRotor(omegaFull*ramp);
			if(ROTATE && lbm.get_t()>0ull && lbm.get_t()%nRotEvery==0ull) { theta += omegaFull*(float)(lbm.get_t()-tLastRot)*ramp; tLastRot = lbm.get_t(); rotateUpdate(omegaFull*ramp); } // 叶轮真实转动 (重体素化)
			lbm.run(nDiag, nSteps);
			lbm.u.read_from_device();
			double qLx = 0.0;
			for(ulong k=0ull; k<meas.size(); k++) qLx += (double)lbm.u.x[meas[k]];
			Qphys = (float)qLx*velPhys*dxPhys*dxPhys;
			const float dpLoad = Kload*Qphys*Qphys;
			const float Keff = Kload*fminf(1.0f, (float)lbm.get_t()/(0.5f*(float)nSteps)); // [启动阀] 负载前 50% 步数线性爬升: 防低流量启动滑入喘振左支 (3mm 全域锁死根因)
			rhoOut = fmaxf(0.97f, 1.0f - 0.5f*dp2drho*Keff*Qphys*Qphys);
			uxS += 0.1f*(Qphys/fmaxf(Aout,1e-9f) - uxS);
			// 出口 E: rho=背压, u=外流速度 (动量匹配, 去除 u=0 动量吸收墙)
			lbm.rho.read_from_device();
			parallel_for((ulong)outp.size(), [&](ulong i) { const ulong n = outp[i];
				lbm.rho[n] = rhoOut; lbm.u.x[n] = uxS; lbm.u.y[n] = 0.0f; lbm.u.z[n] = 0.0f; });
			lbm.rho.write_to_device();  lbm.u.write_to_device();
			lbm.update_force_field();
			const float3 TQ = lbm.object_torque(float3(rotPx, 0.0f, rotPz), TYPE_S|TYPE_X); // 绕 y 轴转矩
			const float tq = TQ.y*Tscale;
			fc = fopen("fan_diag.csv","a"); fprintf(fc, "%u,%.5f,%.4f,%.3f,%.5f\n", lbm.get_t(), Qphys, tq, tq*omegaFull, rhoOut); fclose(fc);
			const float3 FV = lbm.object_force(TYPE_S|TYPE_Y);
			fo = fopen("forces.csv","a"); fprintf(fo, "%u,%.6g,%.6g,%.6g\n", lbm.get_t(), FV.x*Fscale, FV.y*Fscale, FV.z*Fscale); fclose(fo);
			if(lbm.get_t()>=nAvgStart) {
				parallel_for(lbm.get_N(), [&](ulong n) { const float ux=lbm.u.x[n], uy=lbm.u.y[n], uz=lbm.u.z[n];
					uSum[0u*lbm.get_N()+n]+=ux;  uSum[1u*lbm.get_N()+n]+=uy;  uSum[2u*lbm.get_N()+n]+=uz;
					u2Sum[0u*lbm.get_N()+n]+=ux*ux;  u2Sum[1u*lbm.get_N()+n]+=uy*uy;  u2Sum[2u*lbm.get_N()+n]+=uz*uz; });
				nAvg++;
			}
		}
		lbm.update_force_field(); // [2026-09-30 修复] 快照前不再调 update_fields(): 其镜像重构损坏, 主内核每步已维护真实镜像
		lbm.u.write_device_to_vtk("vtk/u_"+to_string(lbm.get_t()));
		print_info("snapshot t="+to_string(lbm.get_t())+" Q="+to_string(Qphys)+" m3/s rhoOut="+to_string(rhoOut));
	}
	if(nAvg>0ull) {
		float* meanBuf = new float[3u*lbm.get_N()];  float* tuBuf = new float[lbm.get_N()];
		for(ulong n=0ull; n<lbm.get_N(); n++) {
			const float mx=uSum[0u*lbm.get_N()+n]/(float)nAvg, my=uSum[1u*lbm.get_N()+n]/(float)nAvg, mz=uSum[2u*lbm.get_N()+n]/(float)nAvg;
			meanBuf[3u*n+0u]=mx;  meanBuf[3u*n+1u]=my;  meanBuf[3u*n+2u]=mz;
			const float v2 = fmaxf(u2Sum[0u*lbm.get_N()+n]/(float)nAvg-mx*mx,0.0f)+fmaxf(u2Sum[1u*lbm.get_N()+n]/(float)nAvg-my*my,0.0f)+fmaxf(u2Sum[2u*lbm.get_N()+n]/(float)nAvg-mz*mz,0.0f);
			tuBuf[n] = 100.0f*sqrtf(v2)/0.08000f;
		}
		writeVTKLike("vtk/mean_u.vtk", meanBuf, 3u, Nx, Ny, Nz);
		writeVTKLike("vtk/tu.vtk", tuBuf, 1u, Nx, Ny, Nz);
		delete[] meanBuf;  delete[] tuBuf;
	}
	delete[] uSum;  delete[] u2Sum;  delete[] bladeData;  delete[] raw;
	print_info("fan-coupled wind tunnel done");
}
void writeVTKLike(const char* fname, const float* data, const uint ncomp, const uint Nx, const uint Ny, const uint Nz) {
	FILE* f = fopen(fname, "wb");  if(!f) { print_error(std::string("cannot write ")+fname); return; }
	char hdr[512];
	sprintf(hdr, "# vtk DataFile Version 3.0\nWindTunnel3D FanCoupled\nBINARY\nDATASET STRUCTURED_POINTS\nDIMENSIONS %u %u %u\nORIGIN %.8E %.8E %.8E\nSPACING 1.00000000 1.00000000 1.00000000\nPOINT_DATA %llu\nSCALARS data float %u\nLOOKUP_TABLE default\n", Nx, Ny, Nz, -0.5*(double)(Nx-1u), -0.5*(double)(Ny-1u), -0.5*(double)(Nz-1u), (unsigned long long)Nx*Ny*Nz, ncomp);
	fputs(hdr, f);
	const ulong Ncell = (ulong)Nx*Ny*Nz;
	float* buf = new float[Ncell*ncomp];
	parallel_for(Ncell*ncomp, [&](ulong n) { union { float f; unsigned int u; } v; v.f = data[n]; v.u = (v.u>>24u)|((v.u>>8u)&0xFF00u)|((v.u<<8u)&0xFF0000u)|(v.u<<24u); buf[n] = v.f; });
	fwrite(buf, 4u*Ncell*ncomp, 1u, f);  fclose(f);  delete[] buf;
}
