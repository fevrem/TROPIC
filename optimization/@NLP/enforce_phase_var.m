function [obj] = enforce_phase_var(obj, rbm, states_0, states_T, t_plus, t_minus)

q_0 = states_0.pos;
q_T = states_T.pos;

        % enforce phase variable at t = 0
        switch obj.Problem.desired_trajectory.option
            case 'virtual-constraint'
                switch obj.Problem.desired_trajectory.param
                    case 'state-based'
                        obj = add_constraint(obj, t_plus - rbm.model.c*q_0 , -obj.Settings.constraint_tol , obj.Settings.constraint_tol , 'phase variable@0' );     
                end
        end


        % enforce phase variable at t = T
        switch obj.Problem.desired_trajectory.option
            case 'virtual-constraint'
                switch obj.Problem.desired_trajectory.param
                    case 'state-based'
                        obj = add_constraint(obj, t_minus - rbm.model.c*q_T , -obj.Settings.constraint_tol , obj.Settings.constraint_tol , 'phase variable@T' );     
                end
        end


end