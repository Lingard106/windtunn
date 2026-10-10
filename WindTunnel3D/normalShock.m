function out = normalShock(M1, gamma)
% 正激波前后关系 (高速工况/教学演示; 25 m/s 时 M1~0.07, 无激波)
% M1: 波前马赫数 (>=1); gamma: 比热比
if nargin < 2, gamma = 1.4; end
M1 = max(M1, 1.0001);
M2 = sqrt((1 + (gamma-1)/2*M1.^2) ./ (gamma*M1.^2 - (gamma-1)/2));  % 波后马赫数
p2p1  = 1 + 2*gamma/(gamma+1).*(M1.^2-1);                           % 静压比
rho2p1 = (gamma+1).*M1.^2 ./ ((gamma-1).*M1.^2 + 2);                % 密度比
p02p01 = (((gamma+1)*M1.^2/2) ./ (1+(gamma-1)/2*M1.^2)).^(gamma/(gamma-1)) ...
       .* ((gamma+1)./(2*gamma*M1.^2-(gamma-1))).^(1/(gamma-1));      % 总压恢复
out = struct('M2',M2,'p2p1',p2p1,'rho2p1',rho2p1,'p02p01',p02p01);
end
