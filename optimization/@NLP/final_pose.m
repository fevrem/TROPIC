function [obj] = final_pose(obj, rbm, states, idxArg )

    
    
    switch rbm.dynamics

        case 'continuous'

            idx = idxArg{2};
            
            NB = rbm.model.NB;
            
            xf_con = Opt.final_pose;
            
            obj = add_constraint(obj, [states.pos;states.vel] - xf_con , -obj.Settings.constraint_tol*ones(2*NB,1), obj.Settings.constraint_tol*ones(2*NB,1), 'final pose' );
            
    end

    
    
        
    
end