function [obj] = step_variables(obj, rbm, states, tf, idxArg)


q = states.pos;
qd = states.vel;
qdd = states.acc;

idx = idxArg{2};

switch rbm.dynamics
    case 'hybrid'
        
        %step_length_guess = obj.Seed.step_length;
        
        p_swing_foot = obj.Functions.P_end_effector( q );
        step_length = p_swing_foot(1);

        if obj.Problem.step_length.bool
            
            [obj] = add_constraint(obj, step_length , obj.Problem.step_length.SL_lb, obj.Problem.step_length.SL_ub, 'step length' );

        end

        [obj] = add_constraint(obj, step_length/tf , obj.Problem.walking_speed.val-obj.Problem.walking_speed.tol , obj.Problem.walking_speed.val+obj.Problem.walking_speed.tol , 'walking_speed' );

        % step width
        switch rbm.dimensions

            case '3D' % spatial model

                step_width = p_swing_foot(2);
                
                if obj.Problem.step_width.bool
            
                    [obj] = add_constraint(obj, step_width , obj.Problem.step_width.SW_lb, obj.Problem.step_width.SW_ub, 'step width' );

                end




        end
        
        
        
        
end







% enforce walking speed
% [obj] = add_constraint(obj, step_length/step_period - obj.Problem.walking_speed.val , -obj.Settings.constraint_tol , obj.Settings.constraint_tol , 'walking_speed' );
%[obj] = add_constraint(obj, step_length/step_period - obj.Problem.walking_speed.val , -obj.Settings.constraint_tol , obj.Settings.constraint_tol , 'walking_speed' );
% [obj] = add_constraint(obj, step_length/step_period , obj.Problem.walking_speed.val-obj.Settings.constraint_tol , obj.Problem.walking_speed.val+obj.Settings.constraint_tol , 'walking_speed' );


%     case 'end'
%         
%         % enforce step length
%         [problem] = Optim.add_constraint(problem, p_swing_foot(1) - step_length , -1E-6 , 1E-6 ); 
% 
%         % enforce walking speed
%         [problem] = Optim.add_constraint(problem, step_length/step_period - nlp.con.walking_speed.val , -nlp.con.walking_speed.tol , nlp.con.walking_speed.tol );
% 
%         
%         % step width
%         switch rbm.dimensions
% 
%             case '2D' % planar model
% 
%                if length(p_swing_foot) ~=2
%                    error('mistake here')
%                end
%                
%                
%                [problem] = Optim.add_constraint(problem, p_swing_foot(2) , -1E-9, 1E-9);
% 
% 
%                 
%             case '3D' % spatial model
% 
%                 % enforce step width
%                 [problem] = Optim.add_constraint(problem, p_swing_foot(2) - step_width , -1E-6 , 1E-6 );  
% 
%                 
%                 
%                 if length(p_swing_foot) ~=3
%                    error('mistake here')
%                 end
% 
% 
%                 [problem] = Optim.add_constraint(problem, p_swing_foot(3) , -1E-9, 1E-9);


end
