function [obj] = configure_nlp( obj , rbm , plt_opt )

%% Argument Validation
% declare specific restrictions on function input arguments
arguments
    obj (1,1) NLP
    rbm (1,1) RobotMotion
    plt_opt (1,:) logical = false 
end


%% Begin NLP Formulation

import casadi.* 

% unscaled time horizon
T = 1;   

% initial time     
%t0 = 0;
[obj, t0k] = add_var( obj , 't0' , 1 , 0 , 0 , 0 , 't0' );
%[obj, t0next] = add_var( obj , 't0n' , 1 , 0 , 0 , 0 , 't0n' );

if 0
    % phase variable at t = 0
    [obj, t_plus] = add_phase_variable(obj, rbm, t0k, 't_plus');
end

% final time         
[obj, tfk] = add_var( obj , 'tf' , 1 , obj.Seed.t(end) , obj.Problem.time.tf_lb , obj.Problem.time.tf_ub , 'tf' );
%[obj, tfnext] = add_var( obj , 'tfn' , 1 , obj.Seed.t(end) , obj.Problem.time.tf_lb , obj.Problem.time.tf_ub , 'tfn' );


TINIT = [t0k;tfk];

if 0
    % phase variable at t = T
    [obj, t_minus] = add_phase_variable(obj, rbm, tfk, 't_minus');
end

%% States & Constraints @ 1st CP

% add states and accelerations at t = 0 
[obj, states] = add_states(obj, rbm, {'idx',0} );

% store initial conditions
states_0 = states;

% scaled dynamics
f_k = (tfk-t0k)*[states.vel; states.acc]; 

% add control at t = 0
[obj, control] = add_control(obj, rbm, {'idx',0} );

% enforce dynamics at t = 0
%[obj] = enforce_dynamics(obj, rbm, states, control, {'idx',0} );

%idxArg = 0;
[obj, Fc] = add_contact_force(obj, rbm, {'idx',0});

Fc

if 0
    % enforce friction constraints  
    [obj] = enforce_friction(obj, rbm, Fc );        
end



%
% 
% FROST MATCH - declare stance foot variables
[obj, pst_k] = add_var( obj , 'pst_0' , 2 , [0;0] , [-0;-0] , [0;0] , 'pst' );
%obj = add_constraint(obj, [states.pos(1);states.pos(2)] - pst_k , -obj.Settings.constraint_tol*ones(2,1) , obj.Settings.constraint_tol*ones(2,1) , 'pRight' );     
%
%
%

%
% 
% FROST MATCH - Bezier coefficients
% add bezier coefficients (a)
%[obj, a] = add_bezier_coefficients( obj, rbm, plt_opt );
alpha_guess = obj.Seed.alpha;
alpha_lb = obj.Problem.desired_trajectory.alpha_lb;
alpha_ub = obj.Problem.desired_trajectory.alpha_ub;
n_coeff = numel(obj.Seed.alpha);
[obj , a_k] = add_var(obj, 'alpha_0' , n_coeff, reshape(alpha_guess, n_coeff, 1) , reshape(alpha_lb, n_coeff, 1) , reshape(alpha_ub, n_coeff, 1) , 'alpha');
%
%
%

%
%
% FROST MATCH - This would be the phase variable if state based
%{
    tau_0 = SymFunction(['tau_0_',domain.Name], T(1) - p(2), {T,p});
    tau_F = SymFunction(['tau_F_',domain.Name], T(2) - p(1), {T,p});
    addNodeConstraint(nlp, tau_0, {T_name,p_name}, 'first', 0, 0, 'Linear');
    addNodeConstraint(nlp, tau_F, {T_name,p_name}, 'last', 0, 0, 'Linear');
%}
%[obj, pTime] = add_var( obj , 'pTime_0' , 2 , [0;0.75] , [0;0.25] , [0;0.75] , 'pTime' );
%obj = add_constraint(obj, pTime - [t0k;tfk] , zeros(2,1) , zeros(2,1) , 'ptimeCont' );     
%
%
% pTime (equivalent of theta)
switch obj.Problem.desired_trajectory.param
    case 'time-based' 
        [obj, t_plus] = add_var( obj, 't_plus' , 1 , 0 , 0 , 0 , 'ptime1' );
        [obj, t_minus] = add_var( obj, 't_minus' , 1 , 0.75 , 0.25 , 0.75 , 'ptime2' );
        PTIME0 = [t_plus;t_minus];
    case 'state-based'
        error('fix')
end




if 0
    % enforce invariance of ZD manifold
    [obj] = enforce_ZD_invariance(obj, rbm, states, a_k, 0, t_minus, t_plus, {'idx',0} );
end

if 0
    % enforce hybrid invariance 
    [obj] = enforce_hybrid_invariance( obj, rbm, states, a_k, t_plus, t_minus , 'start', {'idx',0} );
end


if 0
    % enforce monotonic phase variable
    [obj] = monotonic_phase_var(obj, rbm, states, {'idx',0});

    % enforce torso constraints
    [obj] = torso_constraints(obj, rbm, states, {'idx',0} );

    % enforce swing foot constraints 
    [obj] = swing_foot_constraints(obj, rbm, states, {'idx',0} );

    % enforce stance foot
    [obj] = stance_foot_constraints(obj, rbm, states, {'idx',0} ); 

    % enforce initial pose 
    [obj] = initial_pose(obj, rbm, states, {'idx',0} );
end

% compute cost at t = 0
cost = obj.Objective(rbm, states, control);


%% Parse Trajectory

% length of finite elements
h = T/obj.Settings.nfe;

% initialize the running cost
running_cost = 0;

for k = 1:obj.Settings.nfe
    

    
    %
    t0kprev = t0k;
    tfkprev = tfk;
    t_minusprev = t_minus;
    t_plusprev = t_plus;
    pst_prev = pst_k;
    %
    % FROST-MATCH initial time     
    [obj, t0k] = add_var( obj , 't0k' , 1 , 0 , 0 , 0 , 't0' );
    %
    % FROST-MATCH final time         
    [obj, tfk] = add_var( obj , 'tfk' , 1 , obj.Seed.t(end) , obj.Problem.time.tf_lb , obj.Problem.time.tf_ub , 'tf' );
    %
    %
    %
    obj = add_constraint(obj, [t0kprev;tfkprev] - [t0k;tfk] , zeros(2,1) , zeros(2,1) , 'tCont' );
    
    if k == 1
        obj = add_constraint(obj, tfkprev - t0kprev , 0.25 , 0.75 , 'timeDuration' );
    end
    
    %
    %
    %
 
    

    

    
    % time at end of FE_k
    tprev = ((k-1)*h)*tfkprev;
    t_k = (k*h)*tfkprev;

    % store for integration
    states_previous = states;
    f_previous      = f_k;
    cost_previous   = cost;
    control_previous = control;
    Fc_previous = Fc;
    a_previous = a_k;
    
    % add states at end of FE_k
    [obj, states] = add_states(obj, rbm, {'idx',k} );

    % scaled dynamics
    f_k = (tfkprev-t0kprev)*[states.vel; states.acc]; 

    % add control at end of FE_k
    [obj, control] = add_control(obj, rbm, {'idx',k} );

    % enforce dynamics at end of FE_k
    %[obj] = enforce_dynamics(obj, rbm, states, control, {'idx',k} );

    [obj, Fc] = add_contact_force(obj, rbm, {'idx',k});

    
    
 
    
    
    
%
% 
% FROST MATCH - declare stance foot variables
[obj, pst_k] = add_var( obj , ['pst_' num2str(k)] , 2 , [0;0] , [-0;-0] , [0;0] , 'pst' );
%obj = add_constraint(obj, [states.pos(1);states.pos(2)] - pst_k , -obj.Settings.constraint_tol*ones(2,1) , obj.Settings.constraint_tol*ones(2,1) , 'pRight' );     

%     obj = add_constraint(obj, [states.pos(1);states.pos(2)] - pst_k , zeros(2,1) , zeros(2,1) , 'pRightToe' );     
    obj = add_constraint(obj, pst_prev - pst_k , zeros(2,1) , zeros(2,1) , 'pRightToe' );     

    

% FROST MATCH - Bezier coefficients
% add bezier coefficients (a)
%[obj, a] = add_bezier_coefficients( obj, rbm, plt_opt );
[obj , a_k] = add_var(obj, ['alpha_' num2str(k)] , n_coeff, reshape(alpha_guess, n_coeff, 1) , reshape(alpha_lb, n_coeff, 1) , reshape(alpha_ub, n_coeff, 1) , 'alpha');
%
%

% cont Alpha
obj = add_constraint(obj, a_k - a_previous , zeros(n_coeff,1) , zeros(n_coeff,1) , 'atimeCont' );     


  

%
%
%   
%
% FROST MATCH 
%{
    tau_0 = SymFunction(['tau_0_',domain.Name], T(1) - p(2), {T,p});
    tau_F = SymFunction(['tau_F_',domain.Name], T(2) - p(1), {T,p});
    addNodeConstraint(nlp, tau_0, {T_name,p_name}, 'first', 0, 0, 'Linear');
    addNodeConstraint(nlp, tau_F, {T_name,p_name}, 'last', 0, 0, 'Linear');
%}
% [obj, pTime] = add_var( obj , ['pTime_' num2str(k)] , 2 , [0;0.75] , [0;0.25] , [0;0.75] , 'pTime' );
% obj = add_constraint(obj, pTime - [t0k;tfk] , zeros(2,1) , zeros(2,1) , 'ptimeCont' );     
% %
%
% pTime (equivalent of theta)
switch obj.Problem.desired_trajectory.param
    case 'time-based' 
        [obj, t_plus] = add_var( obj, 't_plus' , 1 , 0 , 0 , 0 , 'ptime1' );
        [obj, t_minus] = add_var( obj, 't_minus' , 1 , 0.75 , 0.25 , 0.75 , 'ptime2' );
    case 'state-based'
        error('fix')
end
obj = add_constraint(obj, [t_plus;t_minus] - [t_plusprev;t_minusprev] , zeros(2,1) , zeros(2,1) , 'ptimeCont' );     


    % compute cost at end of FE_k
    cost = obj.Objective(rbm, states, control);

    % integrate the finite element
 
    if k == 1
        [obj, running_cost] = integrate_FE(obj, rbm, h, running_cost,...
            cost, cost_previous,...
            states, states_previous,...
            control, control_previous,...
            f_k, f_previous, {'idx',k} );

    else
        
        f_k = (tfkprev-t0kprev)*[states.vel; states.acc]; 
        f_previous = (tfkprev-t0kprev)*[states_previous.vel; states_previous.acc]; 
        
        [obj, running_cost] = integrate_FE(obj, rbm, h, running_cost,...
            cost, cost_previous,...
            states, states_previous,...
            control, control_previous,...
            f_k, f_previous, {'idx',k} );

    
    
    end

                               

% enforce equations of motion implicitly
if 0
    obj = add_constraint(obj, obj.Functions.Dynamics.Equations([states_previous.pos; states_previous.vel; states_previous.acc], control_previous.u, Fc_previous) , -obj.Settings.constraint_tol*ones(rbm.model.NB,1) , obj.Settings.constraint_tol*ones(rbm.model.NB,1) , 'dynamics (EOM)' );     
else
    obj = add_constraint(obj, rbm.model.newdynamics([states_previous.pos; states_previous.vel; states_previous.acc], control_previous.u , Fc_previous ) , -obj.Settings.constraint_tol*ones(rbm.model.NB,1) , obj.Settings.constraint_tol*ones(rbm.model.NB,1) , 'dynamics (EOM)' );
end



if k == 1 % FROST- THIS IS USED TO MATCH FROST

    %obj = add_constraint(obj, states_previous.pos(1:2) , -obj.Settings.constraint_tol*ones(2,1) , obj.Settings.constraint_tol*ones(2,1) , 'h_RightToe' );     
    %obj = add_constraint(obj, pst_prev - obj.Functions.J_contact( states_previous.pos )*states_previous.pos , -obj.Settings.constraint_tol*ones(2,1) , obj.Settings.constraint_tol*ones(2,1) , 'h_RightToe' );     

    if 0
        obj = add_constraint(obj, pst_prev - obj.Functions.eta( states_previous.pos ) , -obj.Settings.constraint_tol*ones(2,1) , obj.Settings.constraint_tol*ones(2,1) , 'h_RightToe' );     
    else
        obj = add_constraint(obj, rbm.model.h_RightToe_RightStance([states_previous.pos;states_previous.vel;states_previous.acc], pst_prev ) , -obj.Settings.constraint_tol*ones(2,1) , obj.Settings.constraint_tol*ones(2,1) , 'h_RightToe' );     
    end
    
    
    if 0
        ctrEq = obj.Functions.etadot( [states_previous.pos; states_previous.vel] )
        %ctrEq = obj.Functions.J_contact( states_previous.pos )*states_previous.vel;

        obj = add_constraint(obj, ctrEq(1) , -obj.Settings.constraint_tol , obj.Settings.constraint_tol , 'dh_RightToe' );     
        obj = add_constraint(obj, ctrEq(2) , -obj.Settings.constraint_tol , obj.Settings.constraint_tol , 'dh_RightToe' );     
    else
        obj = add_constraint(obj, rbm.model.dh_RightToe_RightStance([states_previous.pos;states_previous.vel;states_previous.acc], pst_prev ) , -obj.Settings.constraint_tol*ones(2,1) , obj.Settings.constraint_tol*ones(2,1) , 'dh_RightToe' );     

    end

        

    
    
    if 0

        %obj = add_constraint(obj, states_previous.vel(1:2) , -obj.Settings.constraint_tol*ones(2,1) , obj.Settings.constraint_tol*ones(2,1) , 'dh_RightToe' );     
        %obj = add_constraint(obj, states_previous.acc(1:2) , -obj.Settings.constraint_tol*ones(2,1) , obj.Settings.constraint_tol*ones(2,1) , 'ddh_RightToe' );     


        % number of holonomic constraints
        NHC = size(rbm.J_contact,1);


        % enforce constraint equations 
        % CHANGED
        %obj = add_constraint(obj, obj.Functions.Dynamics.Constraints([states_previous.pos; states_previous.vel; states_previous.acc]) , -obj.Settings.constraint_tol*ones(NHC,1) , obj.Settings.constraint_tol*ones(NHC,1) , 'ddh dynamics (HC)' );     
        obj = add_constraint(obj, obj.Functions.Dynamics.Constraints([states_previous.pos; states_previous.vel; states_previous.acc],pst_prev ) , -obj.Settings.constraint_tol*ones(NHC,1) , obj.Settings.constraint_tol*ones(NHC,1) , 'ddh dynamics (HC)' );     
    else

        obj = add_constraint(obj, rbm.model.ddh_RightToe_RightStance([states_previous.pos;states_previous.vel;states_previous.acc], pst_prev ) , -obj.Settings.constraint_tol*ones(2,1) , obj.Settings.constraint_tol*ones(2,1) , 'ddh_RightToe' );     
        
    end


                
    
else
    
    
    if 0
        
        % number of holonomic constraints
        NHC = size(rbm.J_contact,1);


        % enforce constraint equations
        %obj = add_constraint(obj, obj.Functions.Dynamics.Constraints([states_previous.pos; states_previous.vel; states_previous.acc]) , -obj.Settings.constraint_tol*ones(NHC,1) , obj.Settings.constraint_tol*ones(NHC,1) , 'ddh dynamics (HC)' );     

            % enforce constraint equations 
        % CHANGED
        %obj = add_constraint(obj, obj.Functions.Dynamics.Constraints([states_previous.pos; states_previous.vel; states_previous.acc]) , -obj.Settings.constraint_tol*ones(NHC,1) , obj.Settings.constraint_tol*ones(NHC,1) , 'ddh dynamics (HC)' );     
        obj = add_constraint(obj, obj.Functions.Dynamics.Constraints([states_previous.pos; states_previous.vel; states_previous.acc],pst_prev ) , -obj.Settings.constraint_tol*ones(NHC,1) , obj.Settings.constraint_tol*ones(NHC,1) , 'ddh dynamics (HC)' );     

    else
    
        obj = add_constraint(obj, rbm.model.ddh_RightToe_RightStance([states_previous.pos;states_previous.vel;states_previous.acc], pst_prev ) , -obj.Settings.constraint_tol*ones(2,1) , obj.Settings.constraint_tol*ones(2,1) , 'ddh_RightToe' );     

    end
    
    
end

obj = add_constraint( obj , Fc_previous(2) , 0 , Inf , 'ufriction');
mu = obj.Problem.contact.friction.mu;
obj = add_constraint( obj , -Fc_previous(1) + mu*Fc_previous(2) , 0 , Inf ,'ufriction');
obj = add_constraint( obj , Fc_previous(1) + mu*Fc_previous(2) , 0 , Inf ,'ufriction');


if k == 1 % FROST- THIS IS USED TO MATCH FROST

    [obj] = enforce_hybrid_invariance( obj, rbm, states_previous, a_previous, t_plusprev, t_minusprev , 'start', {'idx',0} );

end

% calculate phase variable at end of FE_k       
[obj, s_k] = compute_phase_var(obj, rbm, states_previous, tprev, t_minusprev, t_plusprev);

if 0
    % enforce invariance of ZD manifold
    [obj] = enforce_ZD_invariance(obj, rbm, states_previous, a_previous, s_k, t_minusprev, t_plusprev, {'idx',k} );
end


avar = reshape( a_previous , size(rbm.model.B,2) , obj.Problem.desired_trajectory.order+1 );

y_k = obj.Functions.Control.y( states_previous.pos, avar, s_k, t_plusprev, t_minusprev);
dy_k = obj.Functions.Control.dy( states_previous.pos, states_previous.vel, avar, s_k, t_plusprev, t_minusprev);
ddy_k = obj.Functions.Control.ddy( states_previous.pos, states_previous.vel, states_previous.acc, avar, s_k, t_plusprev, t_minusprev);
ctrl_gain = obj.Problem.zero_dynamics.invariance.control_gain.epsilon;

% enforce ZD invariance
[obj] = add_constraint(obj, obj.Problem.zero_dynamics.invariance.zd_fun(y_k,dy_k,ddy_k,ctrl_gain) , -obj.Settings.constraint_tol*ones(size(rbm.model.B,2),1) , obj.Settings.constraint_tol*ones(size(rbm.model.B,2),1) ,'d2y invariance');


if k == 1
    
    %PTIME0 = [t_plus;t_minus];
    %TINIT = [t0k;tfk];

    tau_0 = PTIME0(1) - TINIT(1);

    [obj] = add_constraint(obj, tau_0 , -obj.Settings.constraint_tol , obj.Settings.constraint_tol , 'tau_0');

% elseif obj.Settings.nfe == k
%    
%     %PTIME0 = [t_plus;t_minus];
%     %TINIT = [t0k;tfk];
%         
%     tau_F = PTIME0(2) - TINIT(2);
%     
%     [obj] = add_constraint(obj, tau_F , -obj.Settings.constraint_tol , obj.Settings.constraint_tol , 'tau_F');
%     
end


%{
I thought output boundary meant y_k
but FROST simply defines it as q_a (the actuated degrees of freedom) 
H0*q
%}
%[obj] = add_constraint(obj, y_k , -2*pi*ones(size(rbm.model.B,2),1) , 2*pi*ones(size(rbm.model.B,2),1) ,'output boundary');
[obj] = add_constraint(obj, rbm.model.H0*states_previous.pos , -2*pi*ones(size(rbm.model.B,2),1) , 2*pi*ones(size(rbm.model.B,2),1) ,'output boundary');


    
% position of swing foot
if 0
    p_swing_foot_k = obj.Functions.P_end_effector( states_previous.pos );

else
    
    p_swing_foot_k = [0;0;rbm.model.u_leftFootHeight_RightStance( states_previous.pos )];

    
end

if ceil( obj.Problem.swing_foot.clearance.timing*obj.Settings.nfe ) == k
    [obj] = add_constraint(obj, p_swing_foot_k(3) , obj.Problem.swing_foot.clearance.val , Inf , 'swing foot height' );
% elseif obj.Settings.nfe == k
%     % do nothing
%     % enforce 0 after walking speed (like in FROST) 
% else
%     % swing foot starts on the ground
%     [obj] = add_constraint(obj, p_swing_foot_k(3) , 0 , Inf , 'u left foot height' );
end
 
% swing foot starts on the ground
[obj] = add_constraint(obj, p_swing_foot_k(3) , 0 , Inf , 'u left foot height' );

    
    
if 0
    % enforce monotonic phase variable
    [obj] = monotonic_phase_var(obj, rbm, states, {'idx',k});

    % enforce torso constraints
    [obj] = torso_constraints(obj, rbm, states, {'idx',k} );

    % enforce swing foot constraints     
    [obj] = swing_foot_constraints(obj, rbm, states, {'idx',k} );    

    % enforce stance foot
    [obj] = stance_foot_constraints(obj, rbm, states, {'idx',k} ); 
end
    
    



if k == obj.Settings.nfe 
    
    % enforce equations of motion implicitly
    if 0
        obj = add_constraint(obj, obj.Functions.Dynamics.Equations([states.pos; states.vel; states.acc], control.u, Fc) , -obj.Settings.constraint_tol*ones(rbm.model.NB,1) , obj.Settings.constraint_tol*ones(rbm.model.NB,1) , 'dynamics (EOM)' );     
    else
        obj = add_constraint(obj, rbm.model.newdynamics([states.pos; states.vel; states.acc], control.u , Fc ) , -obj.Settings.constraint_tol*ones(rbm.model.NB,1) , obj.Settings.constraint_tol*ones(rbm.model.NB,1) , 'dynamics (EOM)' );
    end
    
    % enforce constraint equations
    %obj = add_constraint(obj, obj.Functions.Dynamics.Constraints([states.pos; states.vel; states.acc]) , -obj.Settings.constraint_tol*ones(NHC,1) , obj.Settings.constraint_tol*ones(NHC,1) , 'ddh dynamics (HC)' );     

    
    
    if 0
            % enforce constraint equations 
        % CHANGED
        %obj = add_constraint(obj, obj.Functions.Dynamics.Constraints([states_previous.pos; states_previous.vel; states_previous.acc]) , -obj.Settings.constraint_tol*ones(NHC,1) , obj.Settings.constraint_tol*ones(NHC,1) , 'ddh dynamics (HC)' );     
        obj = add_constraint(obj, obj.Functions.Dynamics.Constraints([states.pos; states.vel; states.acc],pst_k ) , -obj.Settings.constraint_tol*ones(NHC,1) , obj.Settings.constraint_tol*ones(NHC,1) , 'ddh dynamics (HC)' );     


    else
    
        obj = add_constraint(obj, rbm.model.ddh_RightToe_RightStance([states.pos;states.vel;states.acc], pst_k ) , -obj.Settings.constraint_tol*ones(2,1) , obj.Settings.constraint_tol*ones(2,1) , 'ddh_RightToe' );     

    end
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    obj = add_constraint( obj , Fc(2) , 0 , Inf , 'ufriction');
    mu = obj.Problem.contact.friction.mu;
    obj = add_constraint( obj , -Fc(1) + mu*Fc(2) , 0 , Inf ,'ufriction');
    obj = add_constraint( obj , Fc(1) + mu*Fc(2) , 0 , Inf ,'ufriction');


    %t_k
    
    % calculate phase variable at end of FE_k       
    %[obj, s_k] = compute_phase_var(obj, rbm, states, t_k, t_minus, t_plus);
    [obj, s_k] = compute_phase_var(obj, rbm, states, tfk, t_minus, t_plus);

    %(tfkprev-t0kprev)
    
    avar = reshape( a_k , size(rbm.model.B,2) , obj.Problem.desired_trajectory.order+1 );

    y_k = obj.Functions.Control.y( states.pos, avar, s_k, t_plus, t_minus);
    dy_k = obj.Functions.Control.dy( states.pos, states.vel, avar, s_k, t_plus, t_minus);
    ddy_k = obj.Functions.Control.ddy( states.pos, states.vel, states.acc, avar, s_k, t_plus, t_minus);
    ctrl_gain = obj.Problem.zero_dynamics.invariance.control_gain.epsilon;

    % enforce ZD invariance
    [obj] = add_constraint(obj, obj.Problem.zero_dynamics.invariance.zd_fun(y_k,dy_k,ddy_k,ctrl_gain) , -obj.Settings.constraint_tol*ones(size(rbm.model.B,2),1) , obj.Settings.constraint_tol*ones(size(rbm.model.B,2),1) ,'d2y invariance');

   
    %PTIME0 = [t_plus;t_minus];
    %TINIT = [t0k;tfk];
        
    %tau_F = PTIME0(2) - TINIT(2);
    tau_F = t_minus - tfk
    
    
    [obj] = add_constraint(obj, tau_F , -obj.Settings.constraint_tol , obj.Settings.constraint_tol , 'tau_F');
    
%{
I thought output boundary meant y_k
but FROST simply defines it as q_a (the actuated degrees of freedom) 
H0*q
%}
%[obj] = add_constraint(obj, y_k , -2*pi*ones(size(rbm.model.B,2),1) , 2*pi*ones(size(rbm.model.B,2),1) ,'output boundary');
[obj] = add_constraint(obj, rbm.model.H0*states.pos , -2*pi*ones(size(rbm.model.B,2),1) , 2*pi*ones(size(rbm.model.B,2),1) ,'output boundary');


    if 0          
        p_swing_foot_f = obj.Functions.P_end_effector( states.pos );

        [obj] = add_constraint(obj, p_swing_foot_f(1)/(TINIT(2)-TINIT(1))  , obj.Problem.walking_speed.val-1E-4 , obj.Problem.walking_speed.val+1E-4 , 'average_velocity' );
    else
        
        
        
        [obj] = add_constraint(obj, (states.pos(1) - states_0.pos(1))/(TINIT(2)-TINIT(1))  , obj.Problem.walking_speed.val-1E-4 , obj.Problem.walking_speed.val+1E-4 , 'average_velocity' );
        
    end
    
    
    if 0
        % swing foot starts on the ground
        [obj] = add_constraint(obj, p_swing_foot_f(3) , 0 , 0 , 'u left foot height' );

    else
        
        [obj] = add_constraint(obj, rbm.model.u_leftFootHeight_RightStance( states.pos ) , 0 , 0 , 'u left foot height' );
        

    end
    
end
                  
    


end




   
%% Constraints @ Last CP

if 0
    % enforce phase variable
    [obj] = enforce_phase_var(obj, rbm, states_0, states, t_plus, t_minus);

    % enforce final pose 
    [obj] = final_pose(obj, rbm, states, {'idx',k} );
end




if 0
    
    % enforce step characteristics
    [obj] = step_variables(obj, rbm, states, tfk, {'idx',k} );

%else
              
    p_swing_foot_k = obj.Functions.P_end_effector( states.pos );

    %[obj] = add_constraint(obj, p_swing_foot_k(1)/(TINIT(2)-TINIT(1))  , obj.Problem.walking_speed.val , obj.Problem.walking_speed.val , 'average_velocity' );

end





    
    
    
    
if 0
    % enforce hybrid invariance 
    [obj] = enforce_hybrid_invariance( obj, rbm, states, a_k, t_plus, t_minus , 'end', {'idx',k} );
end

if 0
    % enforce periodicity constraints
    [obj] = one_step_periodic(obj, rbm, states_0, states, {'idx',k} );
end







%% ENFORCE PERIODICITY A LA FROST










    %states_0, states

    %
    % FROST MATCH
    %
    %
    NB = rbm.model.NB;

    q_guess = obj.Seed.q(:,end);
    qd_guess = obj.Seed.qd(:,end);

    q_lb = obj.Problem.states.q_lb;
    q_ub = obj.Problem.states.q_ub;

    qd_lb = obj.Problem.states.qd_lb;
    qd_ub = obj.Problem.states.qd_ub;

    % slack for gait.states{2}.x/dx (verified)
    [obj, x]  = add_var( obj , 'x'  , NB , q_guess , q_lb , q_ub , 'x' );
    [obj, dx] = add_var( obj , 'dx' , NB , qd_guess , qd_lb , qd_ub , 'dx' );

    % slack for gait.states{2}.xn/dn (verified)
    [obj, xn]  = add_var( obj , 'xn'  , NB , zeros(NB,1) , -Inf*ones(NB,1) , Inf*ones(NB,1) , 'xn' );
    [obj, dxn] = add_var( obj , 'dxn' , NB , zeros(NB,1) , -Inf*ones(NB,1) , Inf*ones(NB,1) , 'dxn' );

    
    if 0
        
        % inertia matrix needed for impact
        %H = obj.Functions.H_matrix( xn );

        % contact jacobian for impact
        %J = obj.Functions.J_contact( xn );

    else
            
        H = rbm.model.Hmat(xn);
        J = rbm.model.Jcon(xn);
        
    end
    
    
    % delta Fc
%     [obj, fimp] = add_var( obj , 'fimp' , size(J,1) , zeros(2,1) , -Inf*ones(2,1) , Inf*ones(2,1) , 'fimp' );
    [obj, fimp] = add_var( obj , 'fimp' , 2 , zeros(2,1) , -Inf*ones(2,1) , Inf*ones(2,1) , 'fimp' );

    % xMinusCont (verified)
    obj = add_constraint(obj, states.pos - x , zeros(NB,1) , zeros(NB,1) , 'xMinusCont' );     

    % xPlusCont (verified)
    obj = add_constraint(obj, states_0.pos - xn , zeros(NB,1) , zeros(NB,1) , 'xPlusCont' );     

    % dxMinusCont (verified)
    obj = add_constraint(obj, states.vel - dx , zeros(NB,1) , zeros(NB,1) , 'dxMinusCont' );    

    % dxPlusCont (verified)
    obj = add_constraint(obj, states_0.vel - dxn , zeros(NB,1) , zeros(NB,1) , 'dxPlusCont' );    

    qm  = obj.Functions.q_minus( x );
    dqm = obj.Functions.qd_minus( x , dx );

if 1
    
    % dxDiscreteMap
    dxMap = H*(dxn - dqm) - (J'*fimp);
    obj = add_constraint(obj, dxMap , -obj.Settings.constraint_tol*ones(7,1) , obj.Settings.constraint_tol*ones(7,1) , 'dxDiscreteMap' );     

    % xDiscreteMap
    xMap = qm - xn;
    obj = add_constraint(obj, xMap(3:7) , -obj.Settings.constraint_tol*ones(5,1) , obj.Settings.constraint_tol*ones(5,1) , 'xDiscreteMap' );     


else
    
    
    rbm.model.dxDiscreteMapRightImpact( qm, xn, dqm, dxn, fimp );
    rbm.model.xDiscreteMapRightImpact( qm, xn, dqm, dxn, fimp );
    



end














% % % 
% % % x   : edge.OptVarTable.x(1)   -> gait.states{2}.x
% % % xn  : edge.OptVarTable.xn(1)  -> gait.states{2}.xn 
% % % dx  : edge.OptVarTable.dx(1)  -> gait.states{2}.dx
% % % dxn : edge.OptVarTable.dxn(1) -> gait.states{2}.dxn
% % % 
% % % 
% % % xMap:  R*x - xn == 0
% % % dxMap: H*(dxn - R*dx) - J'*fimp
% % % where H = H(xn), fimp is slack, and J = J(xn)
% % % 
% % % dxMap = f(dx,xn,dxn,fimp)
% % % 
% % % dxMap(1:7) == 0
% % % xMap(3:7) == 0
% % % 
% % %   
% % %  
% % % 
% % % % D(q) -> D(q^+)
% % %         M = subs(obj.Mmat, x, xn);
% % %         Gvec = subs(Gvec, x, xn);
% % %         % D(q^+)*(dq^+ - R*dq^-) = sum(J_i'(q^+)*deltaF_i)
% % %         delta_dq = M*(dxn - obj.R*dx) - Gvec;
% % %         obj.dxMap = SymFunction(['dxDiscreteMap' obj.Name],delta_dq,[{dx},{xn},{dxn},deltaF]);
% % % 
% % % 
% % %    
% % % % inertia matrix needed for impact
% % % H = obj.Functions.H_matrix( qminus );
% % % 
% % % % contact jacobian for impact
% % % J = obj.Functions.J_contact( qminus );
% % % 
% % % LHS = [ H , -J' ; J , zeros(size(J,1)) ];
% % % RHS = [ H*qdminus ; zeros(size(J,1),1) ];
% % % 
% % % 
% % % 
% % % 
% % % 
% % % [obj, q_plus] = add_var( obj , 'QPlus' , NB , zeros(NB,1) , -Inf*ones(NB,1), Inf*ones(NB,1) , 'q_plus' );
% % % %[obj, qd_plus] = add_var( obj , 'dQPlus' , NB , qd_guess , qd_lb , qd_ub , 'qd_plus' );
% % % [obj, qd_plus] = add_var( obj , 'dQPlus' , NB , zeros(NB,1) , -Inf*ones(NB,1), Inf*ones(NB,1) , 'qd_plus' );
% % % 
% % % 
% % % 
% % % [obj, f_imp] = add_var( obj , 'fc_imp' , size(J,1) , zeros(size(J,1),1) , -Inf*ones(size(J,1),1) , Inf*ones(size(J,1),1) , 'fc_imp' );
% % % 
% % % 
% % % obj = add_constraint(obj, LHS*[qd_plus;f_imp] - RHS , zeros(NB+size(J,1),1) , zeros(NB+size(J,1),1) , 'qdPlusCont' );     
% % % 
% % % %LHS*yFROST = RHS
% % % %LHS*[qd_plus;f_imp] - RHS = 0
% % % %obj = add_constraint(obj, yFROST(1:NB) - qd_plus , zeros(NB,1) , zeros(NB,1) , 'qPlusCont' );     
% % % 
% % % 
% % % obj = add_constraint(obj, qminus - q_plus , zeros(NB,1) , zeros(NB,1) , 'qPlusCont' );     
% % % %obj = add_constraint(obj, yFROST(1:NB) - qd_plus , zeros(NB,1) , zeros(NB,1) , 'qdPlusCont' );     
% % % 
% % % 
% % % 
% % % if 0
% % % %{
% % %     FROST INVERTS THE IMPACT MATRIX FOR 
% % %     DISCRETE DYNAMICS, I DO THAT HERE TO 
% % %     HAVE THE SAME FORMULATION FOR COMPARISON 
% % %     BUT PROBABLY BETTER TO GO BACK TO 
% % %     IMPLICIT DEFINITION IN THE FUTURE
% % % %}
% % % % xPlusCont 
% % % % -> xPlus = x0
% % % 
% % % % xMinusCont
% % % % -> xMinus swapped 
% % % 
% % % %
% % % %
% % % %FROST MATCH
% % % 
% % % %[obj, f_imp] = add_var( obj , 'fc_imp' , size(J,1) , 100*ones(size(J,1),1) , -1000*ones(size(J,1),1) , 1000*ones(size(J,1),1) , 'fc_imp' );
% % % 
% % % % IF USING EXPLICIT MAP, FC SHOULDNT BE DEFINED SINCE IT IS CALCULATED
% % % [obj, f_imp] = add_var( obj , 'fc_imp' , size(J,1) , zeros(size(J,1),1) , -Inf*ones(size(J,1),1) , Inf*ones(size(J,1),1) , 'fc_imp' );
% % % %
% % % %obj = add_constraint(obj, LHS*[qd_plus;f_imp] - RHS , -obj.Settings.constraint_tol*ones(NB+size(J,1),1) , obj.Settings.constraint_tol*ones(NB+size(J,1),1) , 'impact' );     
% % % 
% % % obj = add_constraint(obj, q_plus - states_0.pos , zeros(NB,1) , zeros(NB,1) , 'qPlusCont' );     
% % % obj = add_constraint(obj, qd_plus - states_0.vel , zeros(NB,1) , zeros(NB,1) , 'qdPlusCont' );     
% % % 
% % % 
% % % %
% % % obj = add_constraint(obj, q_plus - qminus , -obj.Settings.constraint_tol*ones(NB+size(J,1),1) , obj.Settings.constraint_tol*ones(NB+size(J,1),1) , 'impact' );     
% % % % obj = add_constraint(obj, LHS*[qd_plus;f_imp] - RHS , -obj.Settings.constraint_tol*ones(NB+size(J,1),1) , obj.Settings.constraint_tol*ones(NB+size(J,1),1) , 'impact' );     
% % % %
% % % %
% % % end
% % %             
% % %             
% % % if 0    
% % %     % impact
% % %     [obj,q_plus,qd_plus] = apply_impact(obj, rbm, qminus, qdminus);        
% % % 
% % % end
% % %             
% % %             
% % %             % periodicity constraints
% % %             [obj] = enforce_periodicity(obj, rbm, [states_0.pos; states_0.vel], [q_plus;qd_plus] );
% % %             
% % %             
% % %             















%% Wrap Up NLP

% add final cost
running_cost = final_cost(obj, rbm, running_cost, states, control, tfk);

% set NLP cost 
obj.Cost = running_cost;


end  