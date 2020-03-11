function [obj, phase_var] = add_phase_variable(obj, rbm, t, T)

%% Argument Validation

% declare specific restrictions on function input arguments
arguments
    obj (1,1) NLP
    rbm (1,1) RobotMotion
    t (1,1) casadi.MX
    T (1,:) char {mustBeMember(T,{'t_minus','t_plus'})}
end



switch obj.Problem.desired_trajectory.option
    case 'virtual-constraint'

        switch T
            case 't_plus'


                
                    switch obj.Problem.desired_trajectory.param
                        case 'time-based'    
                            %if isfield(obj.Seed,'t0') % frost seed
                                %[obj, t0] = add_var( obj , 't0' , 1 , obj.Seed.t0 , obj.Seed.t0_lb , obj.Seed.t0_ub , 't0' );
                             %   t0 = 0
                            %else
                                %[obj, t0] = add_var( obj , 't0' , 1 , 0 , -1E-12 , 1E-12 , 't0' );                    
                            %    [obj, t0] = add_var( obj , 't0' , 1 , 0 , -1E-12 , 1E-12 , 't0' );                    
                            %end
                            %t_plus = 0;
                            phase_var = t;

                            %t_plus = 0

                        case 'state-based'


                            %[obj, t_plus]  = add_var( obj, 't_plus'  , 1 , rbm.model.c*obj.Seed.q(:,1) , rbm.model.c*obj.Seed.q(:,1)-pi/4 , rbm.model.c*obj.Seed.q(:,1)+pi/4 , 'phase variable' );
                            [obj, phase_var] = add_var( obj, 't_plus' , 1 , obj.Seed.t_plus , obj.Problem.t_plus_lb , obj.Problem.t_plus_ub , 'phase variable' );


                            % enforce phase variable at t = 0
                            %obj = add_constraint(obj, t_plus - rbm.model.c*q_0 , -obj.Settings.constraint_tol , obj.Settings.constraint_tol , 'phase variable@0' );     
                    end


                
                


            case 't_minus'
                    switch obj.Problem.desired_trajectory.param
                        case 'time-based' 
                            phase_var = t;
                        case 'state-based'
                            %[obj, t_minus] = add_var( obj, 't_minus' , 1 , rbm.model.c*obj.Seed.q(:,end) , rbm.model.c*obj.Seed.q(:,end)-pi/4 , rbm.model.c*obj.Seed.q(:,end)+pi/4 , 'phase variable' );
                            [obj, phase_var] = add_var( obj, 't_minus' , 1 , obj.Seed.t_minus , obj.Problem.t_minus_lb , obj.Problem.t_minus_ub , 'phase variable' );
                    end

            otherwise 
                error('only enforce this at beginning or end of gait cycle')

        end


    otherwise
        
        phase_var = [];
        
end




end
