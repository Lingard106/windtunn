function gpu_mov(fld, stride)
% gpu_mov — GPU 风场结果延时动画一键导出 (重运算的标准后处理, 无需 UI 交互)
%   每次风洞.exe 重运算完成后跑一次, 把 20 个时间快照渲染成回放动画:
%     gpu_mov            → Q 准则涡结构 (默认, 涡脱演化最直观)
%     gpu_mov('spd')     → 流速场 (spd/vort/q/rho/mean/turb 任选)
%     gpu_mov('turb',2)  → 每 2 个快照取 1 (渲染更快)
%   输出: gpu_timelapse.mp4 (编码失败自动回退 gpu_timelapse.gif)
%   原理: 设置 WT_FIELD/WT_MOV 环境变量后复用 explore_gpu 的渲染与逐帧捕获
if nargin < 1 || isempty(fld),    fld = 'q'; end
if nargin < 2 || isempty(stride), stride = 1; end
valid = {'spd','vort','q','rho','mean','turb'};
if ~any(strcmp(valid,fld)), error('未知场 "%s" (可选: %s)', fld, strjoin(valid,'/')); end
setenv('WT_FIELD',fld);  setenv('WT_MOV',num2str(stride));
cleaner = onCleanup(@() setenv('WT_MOV',''));   % 无论成功/出错都清理 (explore_gpu 无 clear, 安全)
explore_gpu;
fprintf('--- gpu_mov 完成: gpu_timelapse.mp4 (场=%s, 步长=%d) ---\n', fld, stride);
end
