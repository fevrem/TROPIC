function TOPIC_add_path()
% TOPIC_add_path: add required path to CasADi
%   - must run this from /TOPIC root at least once
%   - configured for CasADi v3.5.1 (released October 2019) 



% make sure it's running from root (/TOPIC)
cur = fullfile(pwd);

% available on Mac, Linux, and Windows
if ismac
    % Code to run on Mac platform
    addpath(fullfile(cur, 'casadi', 'casadi-osx-matlabR2015a-v3.5.1'));
    
elseif isunix
    % Code to run on Linux platform
    addpath(fullfile(cur, 'casadi', 'casadi-linux-matlabR2014b-v3.5.1'));
    
elseif ispc
    % Code to run on Windows platform
    addpath(fullfile(cur, 'casadi', 'casadi-windows-matlabR2016a-v3'));
    
else
    error('Platform not supported');
    
end
import casadi.*

% add spatial vector algebra suite
addpath(fullfile(cur, 'dynamics', 'spatial_v2'));

% make sure the machine is set up with valid C compiler
mex -setup C


end