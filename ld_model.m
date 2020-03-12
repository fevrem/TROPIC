function rbm = ld_model(varargin)

%% Argument Validation
arguments (Repeating)
    varargin (1,:) cell
end


% can add arguments to ld_model in any order
for argNum = 1:nargin
        
    if size(varargin{argNum},2) ~= 2
        error('MyComponent:incorrectSize',...
            'Error. \nArgument must be a 2-element vector. Got %d by %d.', size(varargin{argNum},1), size(varargin{argNum},2))
    end
    
    tempArg = varargin{argNum};
    argIn.(tempArg{1}) = tempArg{2};
    
    
end


if ~isfield(argIn, 'model')
    error('''model'' is a required argument')
end

% turn on debug flag to output model only
if isfield(argIn, 'debug')
    if argIn.debug
        robot = argIn.model();
        rbm.Model = robot;
        return;
    end
end


if ~isfield(argIn, 'dynamics')
    argIn.dynamics = 'continuous';
end


switch argIn.dynamics
    case 'continuous'
        rbm = DynamicalSystem(argIn.model);

    %case 'hybrid'
        %rbm = HybridDynamicalSystem(argIn.model);

    otherwise
        error('MyComponent:incorrectType',...
            'Error. \n''dynamics'' must be either ''continuous'' or ''hybrid'', not %s.', argIn.dynamics)

end



end










