function obj = AddInputs(obj)

control = struct();

control.u = obj.Model.u;

control.u.ID = 'input';

obj.Inputs = control;

obj.Model = rmfield(obj.Model, 'u');


end