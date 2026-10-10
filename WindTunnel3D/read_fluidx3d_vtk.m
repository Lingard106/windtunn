function [A, dims, ncomp] = read_fluidx3d_vtk(fname)
% read_fluidx3d_vtk — 读取 FluidX3D 输出的二进制 VTK (STRUCTURED_POINTS)
% 返回 A: 单分量 (nx,ny,nz) 或矢量 (nx,ny,nz,3); dims/ncomp 元信息
% 性能: 只读头部 (≤4KB) 解析后 fseek 直读数据体 — 旧版整文件 slurp 转 char
%       再 regexp, 单次调用多耗 ~3GB 内存与数秒 CPU (600MB 场); 保留回退路径
fid = fopen(fname, 'rb');
if fid < 0, error('打不开 %s', fname); end
cleaner = onCleanup(@() fclose(fid));
hdr = fread(fid, 4096, 'uint8=>uint8');
txt = char(hdr(:)');                            % 仅头部的文本视图
i1 = regexp(txt, 'DIMENSIONS\s+(\d+)\s+(\d+)\s+(\d+)', 'tokens', 'once');
c0 = regexp(txt, 'SCALARS\s+\S+\s+\S+\s+(\d+)', 'tokens', 'once');
p0 = strfind(txt, 'LOOKUP_TABLE default');
if isempty(i1) || isempty(c0) || isempty(p0)
    % 头超出 4KB (非本管线模板) — 回退整读解析
    frewind(fid);
    raw = fread(fid, inf, 'uint8=>uint8');
    txt = char(raw(:)');
    i1 = regexp(txt, 'DIMENSIONS\s+(\d+)\s+(\d+)\s+(\d+)', 'tokens', 'once');
    c0 = regexp(txt, 'SCALARS\s+\S+\s+\S+\s+(\d+)', 'tokens', 'once');
    p0 = strfind(txt, 'LOOKUP_TABLE default');
    if isempty(i1) || isempty(p0), error('%s 不是 STRUCTURED_POINTS 标量 VTK', fname); end
    pNL = find(raw(p0(1):end)==10, 1) + p0(1) - 1;   % 二进制数据起点
    dims = str2double(i1);
    ncomp = 1; if ~isempty(c0), ncomp = str2double(c0{1}); end
else
    pNL = find(hdr(p0(1):end)==10, 1) + p0(1) - 1;
    dims = str2double(i1);
    ncomp = str2double(c0{1});
end
n = prod(dims)*ncomp;
fseek(fid, pNL, 'bof');
A = fread(fid, n, 'float32=>single', 'b');      % VTK 经典二进制为大端
if numel(A) < n, error('%s 数据不足 (期望 %d float, 实得 %d)', fname, n, numel(A)); end
% FluidX3D 矢量场为 AoS 交错排布 (每格点 x,y,z 相连), 分量是最快变化维
A = reshape(A, [ncomp, dims(1), dims(2), dims(3)]);
A = permute(A, [2, 3, 4, 1]);                  % → (nx,ny,nz,ncomp)
if ncomp == 1, A = A(:,:,:,1); end
end
