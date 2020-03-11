function [obj] = initial_pose(obj, rbm, states, idxArg )

    idx = idxArg{2};

    switch rbm.dynamics

        case 'continuous'

            NB = rbm.model.NB;
            
            x0_con = Opt.initial_pose;
            
            obj = add_constraint(obj, [states.pos;states.vel] - x0_con , -obj.Settings.constraint_tol*ones(2*NB,1), obj.Settings.constraint_tol*ones(2*NB,1) , 'initial pose' );
            
    end

    
end