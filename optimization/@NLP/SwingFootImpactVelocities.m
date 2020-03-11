function [nlp] = SwingFootImpactVelocities(nlp, rbm, grid_var)

%% Argument Validation
arguments
    nlp (1,1) NLP
    rbm (1,1) DynamicalSystem
    grid_var (1,1) struct
    %T (1,:) char {mustBeMember(T, {'t_minus','t_plus'})}
end


if nlp.Problem.SwingFootVerticalImpactVelocity.Bool       
    [nlp] = add_constraint(nlp, nlp.Problem.SwingFootVerticalImpactVelocity.Function(grid_var.(['pos_', num2str(nlp.Settings.ncp)]), grid_var.(['vel_', num2str(nlp.Settings.ncp)])), nlp.Problem.SwingFootVerticalImpactVelocity.LowerBound, nlp.Problem.SwingFootVerticalImpactVelocity.UpperBound, nlp.Problem.SwingFootVerticalImpactVelocity.Name);
end

if nlp.Problem.SwingFootLateralImpactVelocity.Bool
    [nlp] = add_constraint(nlp, nlp.Problem.SwingFootLateralImpactVelocity.Function(grid_var.(['pos_', num2str(nlp.Settings.ncp)]), grid_var.(['vel_', num2str(nlp.Settings.ncp)])), nlp.Problem.SwingFootLateralImpactVelocity.LowerBound, nlp.Problem.SwingFootLateralImpactVelocity.UpperBound, nlp.Problem.SwingFootLateralImpactVelocity.Name);
end

if nlp.Problem.SwingFootForwardImpactVelocity.Bool
    [nlp] = add_constraint(nlp, nlp.Problem.SwingFootForwardImpactVelocity.Function(grid_var.(['pos_', num2str(nlp.Settings.ncp)]), grid_var.(['vel_', num2str(nlp.Settings.ncp)])), nlp.Problem.SwingFootForwardImpactVelocity.LowerBound, nlp.Problem.SwingFootForwardImpactVelocity.UpperBound, nlp.Problem.SwingFootForwardImpactVelocity.Name);
end


end