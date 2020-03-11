function [obj] = enforce_hybrid_invariance(obj, rbm, states, a, t_plus, t_minus , timing, idxArg )


%% Argument Validation

% declare specific restrictions on function input arguments
arguments
    obj (1,1) NLP
    rbm (1,1) RobotMotion
%     q_k 
%     dq_k
%     ddq_k
    states
    a
    t_plus
    t_minus
    timing (1,:) char {mustBeMember(timing,{'start','end'})}
    idxArg
end



% Bezier coefficients
switch obj.Problem.desired_trajectory.option
    case 'virtual-constraint'
        switch obj.Problem.desired_trajectory.type
            case 'bezier'

                    % number of actuated DOF
                    NA = size(rbm.model.B,2);

                    switch timing

                        case 'end'

                            % enforce hybrid invariance via Beziers
                            if obj.Problem.zero_dynamics.hybrid_invariance.end

                                q_T  = states.pos;%q_k;
                                qd_T = states.vel;%dq_k;

                                % position
                                [obj] = add_constraint(obj, rbm.model.H0*q_T - a(end-NA+1:end) , -obj.Settings.constraint_tol*ones(NA,1) , obj.Settings.constraint_tol*ones(NA,1) , 'hybrid invariance');         

                                % velocity        
                                switch obj.Settings.desired_trajectory.poly_phase
                                    case 'time-based'

                                        [obj] = add_constraint(obj, rbm.model.H0*qd_T - (a(end-NA+1:end)-a(end-2*NA+1:end-NA))*obj.Settings.desired_trajectory.poly_order/(t_minus-t_plus) , -obj.Settings.constraint_tol*ones(NA,1) , obj.Settings.constraint_tol*ones(NA,1) , 'hybrid invariance' );  

                                    case 'state-based'
                                        [obj] = add_constraint(obj, rbm.model.H0*qd_T - (a(end-NA+1:end)-a(end-2*NA+1:end-NA))*obj.Settings.desired_trajectory.poly_order*(rbm.model.c*qd_T)/(t_minus-t_plus) , -obj.Settings.constraint_tol*ones(NA,1) , obj.Settings.constraint_tol*ones(NA,1) , 'hybrid invariance' );  

                                    otherwise
                                        error('not supported')
                                end
                                
        
                            end

                        %elseif idx == 0
                        case 'start'
                            
                            % enforce hybrid invariance via Beziers
                            if obj.Problem.zero_dynamics.hybrid_invariance.start
                            
                                q_0  = states.pos;%q_k;
                                qd_0 = states.vel;%dq_k;

                                % position
                                [obj] = add_constraint(obj, rbm.model.H0*q_0 - a(1:NA) , -obj.Settings.constraint_tol*ones(NA,1) , obj.Settings.constraint_tol*ones(NA,1) , 'hybrid invariance');   

                                % velocity        
                                switch obj.Problem.desired_trajectory.param
                                    case 'time-based'
                                        [obj] = add_constraint(obj, rbm.model.H0*qd_0 - (a(NA+1:2*NA)-a(1:NA))*obj.Problem.desired_trajectory.order/(t_minus-t_plus), -obj.Settings.constraint_tol*ones(NA,1) , obj.Settings.constraint_tol*ones(NA,1) , 'hybrid invariance' );  

                                    case 'state-based'
                                        [obj] = add_constraint(obj, rbm.model.H0*qd_0 - (a(NA+1:2*NA)-a(1:NA))*obj.Problem.desired_trajectory.order*(rbm.model.c*qd_0)/(t_minus-t_plus) , -obj.Settings.constraint_tol*ones(NA,1) , obj.Settings.constraint_tol*ones(NA,1) , 'hybrid invariance' );  

                                    otherwise
                                        error('not supported')
                                end

                            end


                    end

        end
end




end
