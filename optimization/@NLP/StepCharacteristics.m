function [nlp] = StepCharacteristics(nlp, rbm, grid_var)

arguments
    nlp (1,1) NLP
    rbm (1,1) DynamicalSystem
    grid_var (1,1) struct
end
    

if nlp.Problem.StepLength.Bool
    [nlp] = AddConstraint(nlp, nlp.Problem.StepLength.Function(grid_var.(['pos_', num2str(nlp.Settings.ncp)])), nlp.Problem.StepLength.LowerBound, nlp.Problem.StepLength.UpperBound, 'step length');
end


if nlp.Problem.StepWidth.Bool
    [nlp] = AddConstraint(nlp, nlp.Problem.StepWidth.Function(grid_var.(['pos_', num2str(nlp.Settings.ncp)])), nlp.Problem.StepWidth.LowerBound, nlp.Problem.StepWidth.UpperBound, 'step width');
end


if nlp.Problem.StepHeight.Bool
    [nlp] = AddConstraint(nlp, nlp.Problem.StepHeight.Function(grid_var.(['pos_', num2str(nlp.Settings.ncp)])), nlp.Problem.StepHeight.LowerBound, nlp.Problem.StepHeight.UpperBound, 'step height');
end


if nlp.Problem.ForwardWalkingSpeed.Bool
    [nlp] = AddConstraint(nlp, nlp.Problem.StepLength.Function(grid_var.(['pos_', num2str(nlp.Settings.ncp)]))/grid_var.tf, nlp.Problem.ForwardWalkingSpeed.LowerBound, nlp.Problem.ForwardWalkingSpeed.UpperBound, 'walking speed');        
end


end
