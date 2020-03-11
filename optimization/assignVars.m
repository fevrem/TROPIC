
function [variables, variablesID] = assignVars(argsIn)

%in = {rbm.States.q, rbm.States.dq, rbm.States.ddq}

if size(argsIn,1) ~= 1
    error('Pass the arguments as comma-separated cell')
end

numArg = size(argsIn, 2);

for i = 1:numArg
    
    variables{i} = argsIn{i}.('sym');

    variablesID{i} = argsIn{i}.('ID');
    
end

end
