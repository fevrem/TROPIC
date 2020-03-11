function [nlp] = FinalConditions(nlp, rbm, grid_var)

%% Argument Validation
arguments
    nlp (1,1) NLP
    rbm (1,1) DynamicalSystem
    grid_var (1,1) struct
end


if nlp.Problem.FinalPositions.Bool
    
    [nlp] = AddConstraint(nlp, grid_var.(['pos_', num2str(nlp.Settings.ncp)]), nlp.Problem.FinalPositions.LowerBound, nlp.Problem.FinalPositions.UpperBound, 'Final Position');
    
end


if nlp.Problem.FinalVelocities.Bool
    
    [nlp] = AddConstraint(nlp, grid_var.(['vel_', num2str(nlp.Settings.ncp)]), nlp.Problem.FinalVelocities.LowerBound, nlp.Problem.FinalVelocities.UpperBound, 'Final Velocity');      
    
end


    
end
