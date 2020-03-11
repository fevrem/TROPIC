function [nlp] = InitialConditions(nlp, rbm, grid_var)

%% Argument Validation
arguments
    nlp (1,1) NLP
    rbm (1,1) DynamicalSystem
    grid_var (1,1) struct
end


if nlp.Problem.InitialPositions.Bool
    
    [nlp] = AddConstraint(nlp, grid_var.pos_1, nlp.Problem.InitialPositions.LowerBound, nlp.Problem.InitialPositions.UpperBound, 'Initial Position');
    
end


if nlp.Problem.InitialVelocities.Bool
    
    [nlp] = AddConstraint(nlp, grid_var.vel_1, nlp.Problem.InitialVelocities.LowerBound, nlp.Problem.InitialVelocities.UpperBound, 'Initial Velocity');      
    
end


    
end
