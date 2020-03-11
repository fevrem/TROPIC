function obj = torso_constraints(obj, rbm, states, idxArg )


    idx = idxArg{2};


    switch rbm.dynamics

        case 'hybrid'

            [obj] = add_torso( obj, rbm, states.pos, states.vel, {'idx',idx} );

    end

end