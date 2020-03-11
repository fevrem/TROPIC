function [nlp] = StanceFoot(nlp, rbm, grid_var)

%% Argument Validation
arguments
    nlp (1,1) NLP
    rbm (1,1) DynamicalSystem
    grid_var (1,1) struct
    %T (1,:) char {mustBeMember(T, {'t_minus','t_plus'})}
end


if nlp.Problem.StanceFootInitialPosition.Bool
    [nlp] = add_constraint(nlp, nlp.Problem.StanceFootInitialPosition.Function(grid_var.pos_1), nlp.Problem.StanceFootInitialPosition.LowerBound, nlp.Problem.StanceFootInitialPosition.UpperBound, nlp.Problem.StanceFootInitialPosition.Name);
end

if nlp.Problem.StanceFootInitialVelocity.Bool
    [nlp] = add_constraint(nlp, nlp.Problem.StanceFootInitialVelocity.Function(grid_var.pos_1, grid_var.vel_1), nlp.Problem.StanceFootInitialVelocity.LowerBound, nlp.Problem.StanceFootInitialVelocity.UpperBound, nlp.Problem.StanceFootInitialVelocity.Name);
end


% foot interference
if nlp.Problem.FootInterference.Bool
    
    for i = 1:nlp.Settings.ncp
        [nlp] = add_constraint(nlp, nlp.Problem.FootInterference.Function(grid_var.(['pos_', num2str(i)])), nlp.Problem.FootInterference.LowerBound, nlp.Problem.FootInterference.UpperBound, nlp.Problem.FootInterference.Name);
    end

end

end