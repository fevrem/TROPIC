function obj = AddStates(obj)

states = struct();

states.q = obj.Model.q;
states.dq = obj.Model.qd;
states.ddq = obj.Model.qdd;

states.q.ID = 'pos';
states.dq.ID = 'vel';
states.ddq.ID = 'acc';

obj.States = states;

obj.Model = rmfield(obj.Model, 'q');
obj.Model = rmfield(obj.Model, 'qd');
obj.Model = rmfield(obj.Model, 'qdd');

end