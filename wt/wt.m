function wt(cmd, varargin)
% WT  Thin wrapper calling the suite's data-vault tool (wt/wt.sh) from MATLAB.
%
%   wt('doctor')              environment & vault health check
%   wt('list')                list archived runs on the vault
%   wt('list','-all')
%   wt('push','vtk')          archive local run output (shorthand or full path)
%   wt('push','vtk','--hash') archive with full SHA256 manifest
%   wt('pull','vtk_2026...')  restore a run from the vault
%   wt('verify','vtk_...','--hash')
%   wt('migrate')             legacy data dry-run plan
%
% ASCII-only I/O (AUTOPILOT rule 6: shell round-trips mangle CJK).
% Requires Git Bash (bash on PATH) and WT3D_VAULT (env or wt/wt.conf).

root = fileparts(mfilename('fullpath'));   % .../WindTunnelSuite/wt
sh = fullfile(root, 'wt.sh');
sh = strrep(sh, '\', '/');

args = '';
for i = 1:numel(varargin)
    a = char(string(varargin{i}));
    if any(a == ' ') || any(a == '"')
        a = ['"' replace(a, '"', '\"') '"'];
    end
    args = [args ' ' a]; %#ok<AGROW>
end
cmdline = sprintf('bash "%s" %s %s', sh, char(string(cmd)), args);

[out, st] = system(cmdline);
disp(out);
if st ~= 0
    error('wt:commandFailed', 'wt %s failed (exit %d)', char(string(cmd)), st);
end
end
