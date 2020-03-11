function [obj] = add_torso( obj, rbm, q, qd, idxArg )

idx = idxArg{2};


if isfield( obj.Problem , 'torso' )

    % torso constraints (is there a torso?)
    if obj.Problem.torso.bool

        switch rbm.dimensions

            case '2D' % planar model


                % define torso angle as decision variables
                %[ torso_pitch_k_guess ] = Model.absolute_torso_angles( [ obj.Seed.q(:,idx+1) ; obj.Seed.qd(:,idx+1) ] );

                
                %[problem, torso_pitch_k , tpitch_inds(end+1,:) ] = Optim.add_var(problem, ['torso_pitch_' num2str(idx)] , 1 , torso_pitch_k_guess , nlp.con.torso.pitch(1) , nlp.con.torso.pitch(2) , ['torso pitch@CP' num2str(idx)] );

                %
                % do not add as variable
                %[obj, t_plus]  = add_var( obj, 't_plus'  , 1 , rbm.model.c*obj.Seed.q(:,1) , rbm.model.c*obj.Seed.q(:,1)-pi/4 , rbm.model.c*obj.Seed.q(:,1)+pi/4 , 'phase variable' );
                %
  
                % enforce algebraic constraints
                [ torso_pitch_k ] = Model.absolute_torso_angles( [q;qd] );
                
                %[problem] = Optim.add_constraint(problem, torso_pitch_k - t_pitch_k , -1E-6 , 1E-6 ); 
                obj = add_constraint(obj, torso_pitch_k, obj.Problem.torso.pitch(1) , obj.Problem.torso.pitch(2) , 'torso' );     

                
            case '3D' % spatial model

                % define torso angles as decision variables
                %[ torso_pitch_k_guess , torso_roll_k_guess ] = Model.absolute_torso_angles( [ seed.q(:,idx+1) ; seed.qd(:,idx+1) ] );

                %[problem, torso_pitch_k , tpitch_inds(end+1,:) ] = Optim.add_var(problem, ['torso_pitch_' num2str(idx)] , 1 , torso_pitch_k_guess , nlp.con.torso.pitch(1) , nlp.con.torso.pitch(2) , ['torso pitch@CP' num2str(idx)] );
                %[problem, torso_roll_k  , troll_inds(end+1,:) ]  = Optim.add_var(problem, ['torso_roll_' num2str(idx)]  , 1 , torso_roll_k_guess  , nlp.con.torso.roll(1)  , nlp.con.torso.roll(2) , ['torso roll@CP' num2str(idx)] );

                % enforce algebraic constraints
                [ torso_pitch_k , torso_roll_k ] = Model.absolute_torso_angles( [q;qd] );
                
                obj = add_constraint(obj, torso_pitch_k, obj.Problem.torso.pitch(1) , obj.Problem.torso.pitch(2) , 'torso pitch' );     

                obj = add_constraint(obj, torso_roll_k, obj.Problem.torso.roll(1) , obj.Problem.torso.roll(2) , 'torso roll' );     

                %[problem] = Optim.add_constraint(problem, torso_roll_k - t_roll_k   , -1E-6 , 1E-6 ); 
                %[problem] = Optim.add_constraint(problem, torso_pitch_k - t_pitch_k , -1E-6 , 1E-6 ); 

            otherwise
                error('Not supporting this dimension')

        end


    end


end



end


