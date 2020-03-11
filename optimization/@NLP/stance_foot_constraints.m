function [obj] = stance_foot_constraints(obj, rbm, states, idxArg )

    idx = idxArg{2};
    
    % this was implemented for FROST comparison
    % because constraining q(1) and q(2) to 0 seemed unfair
    q = states.pos;
    
    %position of swing foot
    p_stance_foot = [q(1); 0; q(2)];
    
    % stance foot always on the ground
    [obj] = add_constraint(obj, p_stance_foot(3), 0 , 0 ,'gnd stance foot');
    %[obj] = add_constraint(obj, p_stance_foot(3), -obj.Settings.constraint_tol , obj.Settings.constraint_tol ,'gnd stance foot');

    [obj] = add_constraint(obj, p_stance_foot(1), 0 , 0 ,'gnd stance foot');
    %[obj] = add_constraint(obj, p_stance_foot(1), -obj.Settings.constraint_tol , obj.Settings.constraint_tol ,'gnd stance foot');

    % velocity of swing foot at t = 0 
    %[problem] = Optim.add_swing_foot_vel( nlp , problem , rbm , seed , casadi.functions , [q_k;dq_k] , 0 );

end