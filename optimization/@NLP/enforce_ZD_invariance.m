function [obj] = enforce_ZD_invariance(obj, rbm, states, a, s, t_minus, t_plus, idxArg)
% idx is the number of the collocation point
%   e.g. idx = 0 means t = 0

idx = idxArg{2};


% hybrid invariance enforced using Beziers at the end
%if idx ~= obj.Settings.nfe && idx ~= 0
    
% Bezier coefficients
switch obj.Problem.desired_trajectory.option
    case 'virtual-constraint'
        switch obj.Problem.desired_trajectory.type
            case 'bezier'
                      
                % enforce ZD invariance
                if obj.Problem.zero_dynamics.invariance.bool

                    a = reshape( a , size(rbm.model.B,2) , obj.Problem.desired_trajectory.order+1 );

                    y_k = obj.Functions.Control.y( states.pos, a, s, t_plus, t_minus );
                    dy_k = obj.Functions.Control.dy( states.pos, states.vel, a, s, t_plus, t_minus );
                    ddy_k = obj.Functions.Control.ddy( states.pos, states.vel, states.acc, a, s, t_plus, t_minus );

                    ctrl_gain = obj.Problem.zero_dynamics.invariance.control_gain.epsilon;

                    %obj.Problem.zero_dynamics.invariance.zd_fun(y_k,dy_k,ddy_k,ctrl_gain);

                    % enforce ZD invariance
                    [obj] = add_constraint(obj, obj.Problem.zero_dynamics.invariance.zd_fun(y_k,dy_k,ddy_k,ctrl_gain) , -obj.Settings.constraint_tol*ones(size(rbm.model.B,2),1) , obj.Settings.constraint_tol*ones(size(rbm.model.B,2),1) ,'invariance');

                end

        end
end



end