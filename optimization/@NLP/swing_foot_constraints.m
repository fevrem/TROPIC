function [obj] = swing_foot_constraints(obj, rbm, states, idxArg )

    idx = idxArg{2};

    switch rbm.dynamics

        case 'hybrid'

            if idx == 0 % beginning of step

                q_0 = states.pos;

                % position of swing foot
                p_swing_foot_0 = obj.Functions.P_end_effector( q_0 );

                % swing foot starts on the ground
                [obj] = add_constraint(obj, p_swing_foot_0(3), -obj.Settings.constraint_tol , obj.Settings.constraint_tol ,'gnd swing foot');

                % velocity of swing foot at t = 0 
                %[problem] = Optim.add_swing_foot_vel( nlp , problem , rbm , seed , casadi.functions , [q_k;dq_k] , 0 );

                
                
            elseif idx == obj.Settings.nfe % end of step

                q_end = states.pos;

                % position of swing foot
                p_swing_foot_end = obj.Functions.P_end_effector( q_end );

                % swing foot starts on the ground
                dh = obj.Problem.step_height.dh;% = 0.0;

                [obj] = add_constraint(obj, p_swing_foot_end(3), -obj.Settings.constraint_tol+dh , obj.Settings.constraint_tol+dh ,'gnd swing foot');

                % velocity of swing foot at t = 0 
                %[problem] = Optim.add_swing_foot_vel( nlp , problem , rbm , seed , casadi.functions , [q_k;dq_k] , 0 );

                % velocity of swing foot at impact
                %[problem] = Optim.add_swing_foot_vel( nlp , problem , rbm , seed , casadi.functions , [q_k;dq_k] , k );

                
                
            else
            

                if isfield(obj.Problem,'swing_foot')
                    if isfield(obj.Problem.swing_foot,'clearance')
                        if obj.Problem.swing_foot.clearance.bool
                           if isa(obj.Problem.swing_foot.clearance.timing,'struct') % all along the gait cycle

                               'blablabla'
                               'bla'
                               'blabla'
                               error('double check')

                           elseif isa(obj.Problem.swing_foot.clearance.timing,'double') % only at a single point of gait cycle 

                                % is percentage from [0,1] of number of FEs
                                %ceil(obj.Settings.nfe/2)
                                % obj.Problem.swing_foot.clearance.timing*obj.Settings.nfe 
                               if ceil( obj.Problem.swing_foot.clearance.timing*obj.Settings.nfe ) == idx

                                    % position of swing foot
                                    p_swing_foot_k = obj.Functions.P_end_effector( states.pos );

                                    % swing foot starts on the ground
                %                     [obj] = add_constraint(obj, p_swing_foot_k(3) - obj.Problem.swing_foot.clearance.val, 0 , Inf );
                %                     [obj] = add_constraint(obj, p_swing_foot_k(3) , obj.Problem.swing_foot.clearance.val , Inf);%Inf );
                                    [obj] = add_constraint(obj, p_swing_foot_k(3) , obj.Problem.swing_foot.clearance.val , Inf , 'swing foot height' );

                               end

                           else
                               error('must be one or the other')
                           end

                        end
                    end

                    
                    % do not add a redundant constraint
                    if ceil( obj.Problem.swing_foot.clearance.timing*obj.Settings.nfe ) ~= idx

                        if isfield(obj.Problem.swing_foot,'above_ground')
                            if obj.Problem.swing_foot.above_ground.bool
                                % position of swing foot
                                p_swing_foot_k = obj.Functions.P_end_effector( states.pos );

                                % swing foot starts on the ground

                                [obj] = add_constraint(obj, p_swing_foot_k(3), 0 , Inf, 'swing foot height' );%Inf);%Inf );

                            end
                        else
                            error('watch this')
                        end

                    end


                end


                

            end
 
            
            
    end

end