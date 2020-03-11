function obj = AddInputs(obj)

% make sure it is SX sym
% same for variables


control = struct();

control.u = obj.Model.u;
%control.u.sym = obj.Model.u;
control.u.ID = 'input';

obj.Inputs = control;

obj.Model = rmfield(obj.Model, 'u');


end