function blade2D = fan_blade_mask(bl, dx, Y3, Z3, RC, TH)
% fan_blade_mask — 叶片 2D 掩码 (θ=0, 格点归约到叶片0系, 与 C++ bladeTest 同法)
% 单一真源: fan_voxelize 与 WindTunnel3D 耦合导出共用
% 输入: bl=fan_geometry 的 blade 结构 | dx 晶格 | Y3/Z3/RC/TH 广播坐标场 (1×ny×nz)
tbh = max(bl.tB/2, 0.6*dx);                  % 薄壁自适应: 叶片半厚 ≥0.6 格
pitchRad = deg2rad(360/bl.Z);
wrapDeg  = mod(bl.th(2)-bl.th(1), 360);
mMaxL    = ceil((wrapDeg + 360/bl.Z + 4)/(360/bl.Z));
blade2D = false(size(Y3));
for m = 0:mMaxL
    psim = mod(mod(TH, pitchRad) + m*pitchRad, 2*pi);
    Yr = RC.*cos(psim);
    Zr = RC.*sin(psim);
    Dk = sqrt((Yr-bl.C(1)).^2 + (Zr-bl.C(2)).^2);
    Ak = mod(atan2d(Zr-bl.C(2), Yr-bl.C(1)) - bl.th(1), 360);
    blade2D = blade2D | (abs(Dk-bl.Ra) <= tbh & Ak <= wrapDeg);
end
end
