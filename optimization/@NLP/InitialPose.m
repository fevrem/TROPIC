function [nlp] = InitialPose(nlp, rbm, grid_var)

%% Argument Validation
arguments
    nlp (1,1) NLP
    rbm (1,1) DynamicalSystem
    grid_var (1,1) struct
end


if nlp.Problem.InitialPositions.Bool
    
    [nlp] = AddConstraint(nlp, nlp.Problem.InitialPositions.Function(grid_var.pos_1), -nlp.Settings.ConstraintTolerance*ones(numel(rbm.States.q.sym),1), nlp.Settings.ConstraintTolerance*ones(numel(rbm.States.q.sym),1), 'Initial Position');
    
end


if nlp.Problem.InitialVelocities.Bool
    
    [nlp] = AddConstraint(nlp, nlp.Problem.InitialVelocities.Function(grid_var.vel_1), -nlp.Settings.ConstraintTolerance*ones(numel(rbm.States.q.sym),1), nlp.Settings.ConstraintTolerance*ones(numel(rbm.States.q.sym),1), 'Initial Velocity');      
    
end


    
end
