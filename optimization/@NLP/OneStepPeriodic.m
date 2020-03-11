function [nlp] = OneStepPeriodic(nlp, rbm, grid_var)

%% Argument Validation
arguments
    nlp (1,1) NLP
    rbm (1,1) DynamicalSystem
    grid_var (1,1) struct
end


if nlp.Problem.OneStepPeriodic.Bool
    
    % q_end
    x = grid_var.(['pos_', num2str(nlp.Settings.ncp)]);
    dx = grid_var.(['vel_', num2str(nlp.Settings.ncp)]);
    
    % q_start
    xn = grid_var.pos_1;
    dxn = grid_var.vel_1;
    
    % inertia matrix needed for impact
    H = nlp.Functions.InertiaMatrix(xn);

    nc = numel(rbm.Contacts);
    for i = 1:nc

        % contact jacobian for impact
        Ji = nlp.Functions.ContactJacobian{i}(xn);

        % define fimp here % delta Fc
        [nlp, fimpi] = AddVar(nlp, ['fimp_', num2str(i)], numel(rbm.Contacts{i}.Fc.sym), zeros(numel(rbm.Contacts{i}.Fc.sym),1), rbm.Contacts{i}.Fc.LowerBound(:,1), rbm.Contacts{i}.Fc.UpperBound(:,1), ['fimp_', num2str(i)]);
        
        if i == 1
            impulseContribution = Ji'*fimpi;
        else
            impulseContribution = impulseContribution + Ji'*fimpi;
        end
    end
    
    
   
    R_map = Model.RelabelingMatrix();

    qm = R_map*x;
    dqm = R_map*dx;

    [PosPeriodicity,VelPeriodicity] = Opt.PeriodicityConstraints(rbm, H, impulseContribution, [xn; dxn], [x; dx], [qm; dqm]);

    
    
    % dxDiscreteMap
    nlp = AddConstraint(nlp, VelPeriodicity, -nlp.Settings.ConstraintTolerance*ones(numel(VelPeriodicity),1), nlp.Settings.ConstraintTolerance*ones(numel(VelPeriodicity),1), 'Periodicity (velocities)');

    % xDiscreteMap
    nlp = AddConstraint(nlp, PosPeriodicity, -nlp.Settings.ConstraintTolerance*ones(numel(PosPeriodicity),1), nlp.Settings.ConstraintTolerance*ones(numel(PosPeriodicity),1), 'Periodicity (positions)');     


    
    
end

    
end
