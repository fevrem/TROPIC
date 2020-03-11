    function obj = monotonic_phase_var(obj, rbm, states, idxArg)        
        
        idx = idxArg{2};

        switch rbm.dynamics

            case 'hybrid'
                
                % phase variable monotonically increasing
                if obj.Problem.theta_dot.bool
                    [obj] = add_constraint(obj, rbm.model.c*states.vel, obj.Problem.theta_dot.min , Inf , 'monotonic' );
                end
                
        end
        
        
    end