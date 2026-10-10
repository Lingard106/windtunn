function [x0, Guse, isFan, objVal, rotVal] = gpu_fan_compat(G, P, meta)
% gpu_fan_compat — 风机耦合模式 (export_fluidx3d_fan 产物) 的可视化兼容头
% 标准管线: x0=试验段前缘-margins, 测试物=raw 2, 无旋转叶轮
% 耦合模式: x0=meta.x0 (风机增压箱), 涵道壁用 meta.Gext (含风机段), 测试物=raw 3, 叶轮=raw 2
if isfield(meta, 'isFanCoupled') && meta.isFanCoupled
    x0 = meta.x0;
    Guse = meta.Gext;
    isFan = true;
    objVal = 3;
    rotVal = 2;
else
    x0 = G.xSec(3) - P.lbm.margins(1);
    Guse = G;
    isFan = false;
    objVal = 2;
    rotVal = 0;   % 无叶轮标记
end
end
