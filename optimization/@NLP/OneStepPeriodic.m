function [nlp] = OneStepPeriodic(nlp, rbm, grid_var)

%% Argument Validation
arguments
    nlp (1,1) NLP
    rbm (1,1) DynamicalSystem
    grid_var (1,1) struct
    %T (1,:) char {mustBeMember(T, {'t_minus','t_plus'})}
end

% % % x   : edge.OptVarTable.x(1)   -> gait.states{2}.x
% % % xn  : edge.OptVarTable.xn(1)  -> gait.states{2}.xn 
% % % dx  : edge.OptVarTable.dx(1)  -> gait.states{2}.dx
% % % dxn : edge.OptVarTable.dxn(1) -> gait.states{2}.dxn
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
% % % % D(q) -> D(q^+)
% % %         M = subs(nlp.Mmat, x, xn);
% % %         Gvec = subs(Gvec, x, xn);
% % %         % D(q^+)*(dq^+ - R*dq^-) = sum(J_i'(q^+)*deltaF_i)
% % %         delta_dq = M*(dxn - nlp.R*dx) - Gvec;
% % %         nlp.dxMap = SymFunction(['dxDiscreteMap' nlp.Name],delta_dq,[{dx},{xn},{dxn},deltaF]);

if nlp.Problem.OneStepPeriodic.Bool
    
    % slack for q_minus
    %[nlp, x]  = add_var( nlp , 'x'  , NB , q_guess , q_lb , q_ub , 'x' );
    %[nlp, dx] = add_var( nlp , 'dx' , NB , qd_guess , qd_lb , qd_ub , 'dx' );

    x = grid_var.(['pos_', num2str(nlp.Settings.ncp)]);
    dx = grid_var.(['vel_', num2str(nlp.Settings.ncp)]);
    
    % slack for gait.states{2}.xn/dn (verified)
    %[nlp, xn]  = add_var( nlp , 'xn'  , NB , zeros(NB,1) , -Inf*ones(NB,1) , Inf*ones(NB,1) , 'xn' );
    %[nlp, dxn] = add_var( nlp , 'dxn' , NB , zeros(NB,1) , -Inf*ones(NB,1) , Inf*ones(NB,1) , 'dxn' );

    xn = grid_var.pos_1;
    dxn = grid_var.vel_1;
    
    % inertia matrix needed for impact
    H = nlp.Functions.InertiaMatrix(xn);

    nc = numel(rbm.Contacts);
    for i = 1:nc

        % contact jacobian for impact
        Ji = nlp.Functions.ContactJacobian{i}(xn);

        % define fimp here % delta Fc
        [nlp, fimpi] = add_var(nlp, ['fimp_', num2str(i)], numel(rbm.Contacts{i}.Fc.sym), zeros(numel(rbm.Contacts{i}.Fc.sym),1), rbm.Contacts{i}.Fc.LowerBound(:,1), rbm.Contacts{i}.Fc.UpperBound(:,1), ['fimp_', num2str(i)]);
        
        if i == 1
            impulseContribution = Ji'*fimpi;
        else
            impulseContribution = impulseContribution + Ji'*fimpi;
        end
    end
    
    
    
    
    if 1
        
        R_map = Model.RelabelingMatrix();
        %qm  = nlp.Functions.q_minus( x );
        %dqm = nlp.Functions.qd_minus( x , dx );
        qm = R_map*x;
        dqm = R_map*dx;

    end
    
    %[position2enforce,velocity2enforce] = Opt.PeriodicityConstraints(rbm, H, impulseContribution, [xn; dxn], [x; dx], [qm; dqm]);
    [PosPeriodicity,VelPeriodicity] = Opt.PeriodicityConstraints(rbm, H, impulseContribution, [xn; dxn], [x; dx], [qm; dqm]);

    
    
    % dxDiscreteMap
    %dxMap = H*(dxn - dqm) - (impulseContribution);
    %nlp = add_constraint(nlp, dxMap(velocity2enforce), -nlp.Settings.ConstraintTolerance*ones(numel(velocity2enforce),1), nlp.Settings.ConstraintTolerance*ones(numel(velocity2enforce),1), 'Periodicity (velocities)');     
    nlp = add_constraint(nlp, VelPeriodicity, -nlp.Settings.ConstraintTolerance*ones(numel(VelPeriodicity),1), nlp.Settings.ConstraintTolerance*ones(numel(VelPeriodicity),1), 'Periodicity (velocities)');     

    % xDiscreteMap
    %xMap = qm - xn;
    %nlp = add_constraint(nlp, xMap(position2enforce), -nlp.Settings.ConstraintTolerance*ones(numel(position2enforce),1), nlp.Settings.ConstraintTolerance*ones(numel(position2enforce),1), 'Periodicity (positions)');     

    nlp = add_constraint(nlp, PosPeriodicity, -nlp.Settings.ConstraintTolerance*ones(numel(PosPeriodicity),1), nlp.Settings.ConstraintTolerance*ones(numel(PosPeriodicity),1), 'Periodicity (positions)');     


    
    
end

    
end