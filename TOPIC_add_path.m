function TOPIC_add_path()

% make sure it's running from root
cur = pwd;
if ~strcmp(cur(end-10:end),'PROSIMFROST')
    error('Program must run from root folder /PROSIMFROST folder.') 
end


% Add system related functions paths
cur = fileparts(mfilename('fullpath'));

if isempty(cur)
    cur = pwd;
end

warning('off', 'MATLAB:rmpath:DirNotFound')
rmpath(genpath(cur))
warning('on', 'MATLAB:rmpath:DirNotFound')


addpath(cur)
addpath(genpath(fullfile(cur, 'Spatial_v2')))

os_status = determine_os;

if nargin == 1
    Sim_or_Opt = varargin{1};
    if strcmpi(Sim_or_Opt,'casadi')
        if strcmp(os_status,'osx')
            addpath(fullfile(cur, 'casadi-osx-matlabR2015a-v3.4.5'));
            addpath(genpath([cur filesep 'casadi-osx-matlabR2015a-v3.4.5']))
        elseif strcmp(os_status,'lin')
            addpath(fullfile(cur, 'casadi-linux-matlabR2014b-v3.4.4'));
            addpath(genpath([cur filesep 'casadi-linux-matlabR2014b-v3.4.4']))
        elseif strcmp(os_status,'win')
            error('CasADi is not available for Windows machine.')
        else
            error('Platform not supported')
        end
        % CasADi 3.4.5
        import casadi.*
    end        
end


% make sure the machine has a valid C compiler setup
mex -setup C


end
