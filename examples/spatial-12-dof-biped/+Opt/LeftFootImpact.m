function [cons, vars, cost] = LeftFootImpact(rbm, ncp)

arguments
    rbm (1,1) DynamicalSystem
    ncp (1,1) double
end

    
import casadi.*

% list of constraints enforced at every collocation point

%{
    define the dynamics equations (2nd-order ode + holonomic 
    constraint) that will be enforced implicitly
%}

% opt.settings.ncp = ncp;
% opt.settings.tol = 1E-4;

%{
    VARIABLES ARE DEFINED SEQUENTIALLY, PARSING THROUGH
    EACH CP AND ADDING VARIABLES AND CONSTRAINTS APPROPRIATELY 
%}
% equations of motion

NB = rbm.Model.nd;

cons = []

vars = []

cost = []




%%

if 0
    % options are 'point', 'line', or 'plane' contact
    cons = Opt.RightStance(cons, rbm, ncp);


    % could say one instance of it
    tf = Var('tf', 1);
    tf.LowerBound = 0.1;
    tf.UpperBound = 1.5;
    tf.Seed = 0.5;
    vars.tf = tf;



    % phase variable at beginning and end (could be time or could be mechanical phase variable)
    %theta = Var('theta')
    theta = Var('theta', 2);
    %theta.sym = SX.sym(theta.ID, 2);
    %theta.ID = 'theta';
    theta.LowerBound = [-10; -10];
    theta.UpperBound = [10; 10];
    theta.Seed = zeros(2,1);
    vars.theta = theta;


    % phase variable at beggining of step
    cons = Constraint.addConstraint(cons);
    cons{end}.Name = 'tau(0) = 0';
    cons{end}.SymbolicExpression = theta.sym(1) - rbm.Model.c*rbm.States.q.sym;
    [cons{end}.DependentVariables, cons{end}.DependentVariablesID] = assignVars({theta, rbm.States.q});
    cons{end}.Function = Function('f', cons{end}.DependentVariables, {cons{end}.SymbolicExpression});
    cons{end}.Occurrence = 1;



    % phase variable at end of step
    cons = Constraint.addConstraint(cons);
    cons{end}.Name = 'tau(end) = 1';
    cons{end}.SymbolicExpression = theta.sym(2) - rbm.Model.c*rbm.States.q.sym;
    [cons{end}.DependentVariables, cons{end}.DependentVariablesID] = assignVars({theta, rbm.States.q});
    cons{end}.Function = Function('f', cons{end}.DependentVariables, {cons{end}.SymbolicExpression});
    cons{end}.Occurrence = ncp;



    % normalized phase variable
    tau = Var('tau', 1);
    tau.LowerBound = -1;
    tau.UpperBound = 2;
    tau.Seed = linspace(0, 1, ncp);
    vars.tau = tau;


    % normalized phase variable throughout step
    cons = Constraint.addConstraint(cons);
    cons{end}.Name = 'tau_i = Normalized Phase Variable';
    cons{end}.SymbolicExpression = tau.sym - (rbm.Model.c*rbm.States.q.sym - theta.sym(1))/(theta.sym(2) - theta.sym(1));
    [cons{end}.DependentVariables, cons{end}.DependentVariablesID] = assignVars({tau, theta, rbm.States.q});
    cons{end}.Function = Function('f', cons{end}.DependentVariables, {cons{end}.SymbolicExpression});



    % monotonic phase variable
    cons = Constraint.addConstraint(cons);
    cons{end}.Name = 'Monotonic Phase Variable';
    cons{end}.SymbolicExpression = rbm.Model.c*rbm.States.dq.sym;
    [cons{end}.DependentVariables, cons{end}.DependentVariablesID] = assignVars({rbm.States.dq});
    cons{end}.Function = Function('f', cons{end}.DependentVariables, {cons{end}.SymbolicExpression});
    cons{end}.UpperBound = Inf;
    cons{end}.LowerBound = 0;






    %% Enforce state constraints during stance

    rbm.States.q.LowerBound = -2*pi*ones(NB,1);
    rbm.States.q.UpperBound = 2*pi*ones(NB,1);
    rbm.States.q.Seed = 0*ones(NB,ncp);

    rbm.States.dq.LowerBound = -20*ones(NB,1);
    rbm.States.dq.UpperBound = 20*ones(NB,1);
    rbm.States.dq.Seed = 0*ones(NB,ncp);

    rbm.States.ddq.LowerBound = -100*ones(NB,1);
    rbm.States.ddq.UpperBound = 100*ones(NB,1);
    rbm.States.ddq.Seed = 0*ones(NB,ncp);

    %rbm.Inputs.u.LowerBound = -100*ones(6,1);
    %rbm.Inputs.u.UpperBound = 100*ones(6,1);
    %rbm.Inputs.u.Seed = 0*ones(6,ncp);
    
end
    
% no matter what, enforce continuity in states
% if states not used, remove them
% define them as Vars, but not MX



    R = Model.RelabelingMatrix()
    
    NB = rbm.Model.nd;
    
    
%     cons = Constraint.addConstraint(cons);
%     cons{end}.Name = 'Impact 1';
%     cons{end}.SymbolicExpression = rbm.States.q.sym;
%     [cons{end}.DependentVariables, cons{end}.DependentVariablesID] = assignVars({rbm.States.q});
%     cons{end}.Function = Function('f', cons{end}.DependentVariables, {cons{end}.SymbolicExpression});
%     cons{end}.UpperBound = 0;
%     cons{end}.LowerBound = 0;
%     
%     
%     
%     cons = Constraint.addConstraint(cons);
%     cons{end}.Name = 'Impact 2';
%     cons{end}.SymbolicExpression = rbm.States.dq.sym;
%     [cons{end}.DependentVariables, cons{end}.DependentVariablesID] = assignVars({rbm.States.dq});
%     cons{end}.Function = Function('f', cons{end}.DependentVariables, {cons{end}.SymbolicExpression});
%     cons{end}.UpperBound = 0;
%     cons{end}.LowerBound = 0;
    
    
% RIDIG IMPACT
% function [cons, vars, post_impact_states] = InelasticImpact(rbm, cons, vars)
% post_impact_states.pos
% post_impact_states.pos

    % pre impact states
    qminus = Var('qminus', rbm.Model.nd);
    qminus.LowerBound = rbm.States.q.LowerBound;
    qminus.UpperBound = rbm.States.q.UpperBound;
    qminus.Seed = rbm.States.q.Seed(:,1);
    vars.qminus = qminus;
    
    dqminus = Var('dqminus', rbm.Model.nd);
    dqminus.LowerBound = rbm.States.dq.LowerBound;
    dqminus.UpperBound = rbm.States.dq.UpperBound;
    dqminus.Seed = rbm.States.dq.Seed(:,1);
    vars.dqminus = dqminus;    

    % pre impact states
    qplus = Var('qplus', rbm.Model.nd);
    qplus.LowerBound = rbm.States.q.LowerBound;
    qplus.UpperBound = rbm.States.q.UpperBound;
    qplus.Seed = rbm.States.q.Seed(:,1);
    vars.qplus = qplus;
    
    dqplus = Var('dqplus', rbm.Model.nd);
    dqplus.LowerBound = rbm.States.dq.LowerBound;
    dqplus.UpperBound = rbm.States.dq.UpperBound;
    dqplus.Seed = rbm.States.dq.Seed(:,1);
    vars.dqplus = dqplus;
    

    cons = Constraint.addConstraint(cons);
    cons{end}.Name = 'Relabel Positions';
    cons{end}.SymbolicExpression = (R*rbm.States.q.sym) - qminus.sym;
    [cons{end}.DependentVariables, cons{end}.DependentVariablesID] = assignVars({rbm.States.q, qminus});
    cons{end}.Function = Function('f', cons{end}.DependentVariables, {cons{end}.SymbolicExpression});
    cons{end}.UpperBound = zeros(12,1);
    cons{end}.LowerBound = zeros(12,1);  
  

    cons = Constraint.addConstraint(cons);
    cons{end}.Name = 'Relabel Velocities';
    cons{end}.SymbolicExpression = (R*rbm.States.dq.sym) - dqminus.sym;
    [cons{end}.DependentVariables, cons{end}.DependentVariablesID] = assignVars({rbm.States.dq, dqminus});
    cons{end}.Function = Function('f', cons{end}.DependentVariables, {cons{end}.SymbolicExpression});
    cons{end}.UpperBound = zeros(12,1);
    cons{end}.LowerBound = zeros(12,1);
    
    
    dxMap = rbm.Dynamics.H_matrix*(dqplus.sym - dqminus.sym) - (rbm.Contacts{1}.Jac_contact'*rbm.Contacts{1}.Fc.sym);
    xMap = qplus.sym - qminus.sym;

    cons = Constraint.addConstraint(cons);
    cons{end}.Name = 'Impact 1';
    cons{end}.SymbolicExpression = xMap;
    cons{end}.DependentVariables = {qminus.sym, qplus.sym};
    cons{end}.DependentVariablesID = {qminus.ID, qplus.ID};
    cons{end}.Function = Function('f', cons{end}.DependentVariables, {cons{end}.SymbolicExpression});
    cons{end}.UpperBound = zeros(12,1);
    cons{end}.LowerBound = zeros(12,1);
    
    
    cons = Constraint.addConstraint(cons);
    cons{end}.Name = 'Impact 2';
    cons{end}.SymbolicExpression = dxMap;
    cons{end}.DependentVariables = {rbm.States.q.sym, dqminus.sym, dqplus.sym};
    cons{end}.DependentVariablesID = {qminus.ID, dqminus.ID, dqplus.ID};
    cons{end}.Function = Function('f', cons{end}.DependentVariables, {cons{end}.SymbolicExpression});
    cons{end}.UpperBound = zeros(12,1);
    cons{end}.LowerBound = zeros(12,1);
    
    
    % from the function
    post_impact_states.pos = vars.qplus;
    post_impact_states.vel = vars.dqplus;
    
    % Periodicity constraints
    % TargetPhase (integer, default = current)
    % TargetState (integer, 1:ncp)
    

    cons = Constraint.addConstraint(cons);
    cons{end}.Name = 'Periodicity (Position)';
    cons{end}.SymbolicExpression = post_impact_states.pos.sym(4:12) - rbm.States.q.sym(4:12);
    cons{end}.DependentVariables = {post_impact_states.pos.sym, rbm.States.q.sym};
    cons{end}.DependentVariablesID = {post_impact_states.pos.ID, rbm.States.q.ID};
    cons{end}.DependentVariablesPhase = {2, 1};
    cons{end}.DependentVariablesCP = {ncp, 1};
    cons{end}.Function = Function('f', cons{end}.DependentVariables, {cons{end}.SymbolicExpression});
    cons{end}.Occurrence = ncp;
    
    cons = Constraint.addConstraint(cons);
    cons{end}.Name = 'Periodicity (Velocity)';
    cons{end}.SymbolicExpression = post_impact_states.vel.sym - rbm.States.dq.sym;
    cons{end}.DependentVariables = {post_impact_states.vel.sym, rbm.States.dq.sym};
    cons{end}.DependentVariablesID = {post_impact_states.vel.ID, rbm.States.dq.ID};
    cons{end}.DependentVariablesPhase = {2, 1};
    cons{end}.DependentVariablesCP = {ncp, 1};
    cons{end}.Function = Function('f', cons{end}.DependentVariables, {cons{end}.SymbolicExpression});
    cons{end}.Occurrence = ncp;    
    
    
    
    cons
    cons{1}
    cons{2}
    cons{3}
    cons{4}
    cons{5}
    cons{6}
    
    vars
    
    vars.qplus
    vars.dqplus
    vars.qminus
    vars.dqminus
    
    %frffr
    
    return
    

    
    qm = Var('qm', rbm.Model.nd);
    qm.LowerBound = rbm.States.q.LowerBound;
    qm.UpperBound = rbm.States.q.UpperBound;
    qm.Seed = rbm.States.q.Seed()
    qm.Seed = zeros(3,1);
    vars.fimp = qm;

    q
    qm
    %R*
    
    fimp = Var('fimp', 3);
    fimp.LowerBound = [-1000; -1000; -1000];
    fimp.UpperBound = [1000; 1000; 1000];
    fimp.Seed = zeros(3,1);
    vars.fimp = fimp;

    
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

    % dxDiscreteMap
    dxMap = H*(dxn - dqm) - (J'*fimp);
    obj = add_constraint(obj, dxMap , -obj.Settings.constraint_tol*ones(7,1) , obj.Settings.constraint_tol*ones(7,1) , 'dxDiscreteMap' );     

    % xDiscreteMap
    xMap = qm - xn;
    obj = add_constraint(obj, xMap(3:7) , -obj.Settings.constraint_tol*ones(5,1) , obj.Settings.constraint_tol*ones(5,1) , 'xDiscreteMap' );     

    
    
    
    
    
    
    
    
    
    
    
    if 0
    % RidigImpact
    q_minus = Var('q_minus', NB);
    
    
    q_minus = Var('q_minus', NB);
    
    q_minus.LowerBound = 0.1;
    q_minus.UpperBound = 1.5;
    q_minus.Seed = 0.5;
    vars.tf = tf;
    
    
    
    %if 0
    % phase variable at beggining of step
    cons = ContinuityConstraint.addConstraint(cons)
    
    
    cons{end}.TargetPhase = 2;
    cons{end}.TargetVars = rbm.States.q;
    cons{end}.TargetTiming = 'start'
    
    cons{end}.SourcePhase = 1;
    cons{end}.SourceVars = rbm.States.q;
    cons{end}.SourceTiming = 'end'
    
    
    
    cons{1}
    
    %rbm = ContinuityConstraint.addConstraint(cons)
    %cons = ContinuityConstraint.addConstraint(cons)
    
    
    
    ffrkfr
    
   % if 0
    
    
    
    % set impact
    guard = RigidImpact('RightImpact',domain,'leftFootHeight');
    
    % Relabeling Matrix
    guard.R = guard.R(:,[1:3,6:7,4:5]);
    
    % set the impact constraint
    % we will compute the impact map every time you add an impact
    % constraints, so it would be helpful to load expressions directly
    guard.addImpactConstraint(struct2array(domain.HolonomicConstraints), load_path);
    
    
    
    


    
    %states_0, states

    %
    % FROST MATCH
    %
    %
    NB = rbm.model.NB;

    
    rbm.States.q.LowerBound = -2*pi*ones(NB,1);
    rbm.States.q.UpperBound = 2*pi*ones(NB,1);
    rbm.States.q.Seed = 0*ones(NB,ncp);

    rbm.States.dq.LowerBound = -20*ones(NB,1);
    rbm.States.dq.UpperBound = 20*ones(NB,1);
    rbm.States.dq.Seed = 0*ones(NB,ncp);
    
    
    % slack for gait.states{2}.x/dx (verified)
    [obj, x]  = add_var( obj , 'x'  , NB , q_guess , q_lb , q_ub , 'x' );
    [obj, dx] = add_var( obj , 'dx' , NB , qd_guess , qd_lb , qd_ub , 'dx' );

    
    
    % slack for gait.states{2}.xn/dn (verified)
    [obj, xn]  = add_var( obj , 'xn'  , NB , zeros(NB,1) , -Inf*ones(NB,1) , Inf*ones(NB,1) , 'xn' );
    [obj, dxn] = add_var( obj , 'dxn' , NB , zeros(NB,1) , -Inf*ones(NB,1) , Inf*ones(NB,1) , 'dxn' );

    

            
    H = rbm.model.Hmat(xn);
    J = rbm.model.Jcon(xn);



    
    % delta Fc
%     [obj, fimp] = add_var( obj , 'fimp' , size(J,1) , zeros(2,1) , -Inf*ones(2,1) , Inf*ones(2,1) , 'fimp' );
%    [obj, fimp] = add_var( obj , 'fimp' , 2 , zeros(2,1) , -Inf*ones(2,1) , Inf*ones(2,1) , 'fimp' );

    fimp = Var('fimp', 3);
    fimp.LowerBound = [-1000; -1000; -1000];
    fimp.UpperBound = [1000; 1000; 1000];
    fimp.Seed = zeros(3,1);
    vars.fimp = fimp;

    
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


    % dxDiscreteMap
    dxMap = H*(dxn - dqm) - (J'*fimp);
    obj = add_constraint(obj, dxMap , -obj.Settings.constraint_tol*ones(7,1) , obj.Settings.constraint_tol*ones(7,1) , 'dxDiscreteMap' );     

    % xDiscreteMap
    xMap = qm - xn;
    obj = add_constraint(obj, xMap(3:7) , -obj.Settings.constraint_tol*ones(5,1) , obj.Settings.constraint_tol*ones(5,1) , 'xDiscreteMap' );     

    
    end





end
