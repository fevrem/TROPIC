function [obj, s_k] = compute_phase_var(obj, rbm, states, t_k, t_minus, t_plus)

    % calculate normalized phase variable
    switch obj.Problem.desired_trajectory.option
        case 'virtual-constraint'
        
            switch obj.Problem.desired_trajectory.param
                case 'time-based'
                    s_k = Control.calculate_phase_var( t_k , t_plus, t_minus );

                case 'state-based'
                    s_k = Control.calculate_phase_var( states.pos , t_plus, t_minus , rbm.model );

            end
        case 'free'
            s_k = [];
        otherwise
            error('not supported')
    end

end