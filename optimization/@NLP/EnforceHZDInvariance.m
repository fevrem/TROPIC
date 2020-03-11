function [obj] = EnforceHZDInvariance(obj, rbm, grid_var)

%% Argument Validation

% declare specific restrictions on function input arguments
arguments
    obj (1,1) NLP
    rbm (1,1) DynamicalSystem
    grid_var (1,1) struct
end



% number of actuated DOF
NA = numel(rbm.Inputs.u.sym);

a = grid_var.a;

q_0  = grid_var.pos_1;
qd_0 = grid_var.vel_1;

% position
[obj] = AddConstraint(obj, rbm.Model.H0*q_0 - a(1:NA), -obj.Settings.ConstraintTolerance*ones(NA,1), obj.Settings.ConstraintTolerance*ones(NA,1), 'HZD Invariance');

% velocity        
switch obj.Problem.Trajectory.PolyPhase
    case 'time-based'
        t_plus = grid_var.p_time_plus;
        t_minus = grid_var.p_time_minus;
        [obj] = AddConstraint(obj, rbm.Model.H0*qd_0 - (a(NA+1:2*NA)-a(1:NA))*obj.Problem.Trajectory.PolyOrder/(t_minus-t_plus), -obj.Settings.ConstraintTolerance*ones(NA,1), obj.Settings.ConstraintTolerance*ones(NA,1), 'HZD Invariance');

    case 'state-based'
        t_plus = grid_var.p_phase_plus;
        t_minus = grid_var.p_phase_minus;        
        [obj] = AddConstraint(obj, rbm.Model.H0*qd_0 - (a(NA+1:2*NA)-a(1:NA))*obj.Problem.Trajectory.PolyOrder*(rbm.Model.c*qd_0)/(t_minus-t_plus), -obj.Settings.ConstraintTolerance*ones(NA,1), obj.Settings.ConstraintTolerance*ones(NA,1), 'HZD Invariance');  

end





end
