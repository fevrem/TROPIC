function [nlp] = AbsTorsoAngles(nlp, rbm, grid_var)

%% Argument Validation
arguments
    nlp (1,1) NLP
    rbm (1,1) DynamicalSystem
    grid_var (1,1) struct
end


% absolute torso 
if nlp.Problem.TorsoAngles.Bool
    
    for i = 1:nlp.Settings.ncp
        [nlp] = AddConstraint(nlp, nlp.Problem.TorsoAngles.Function(grid_var.(['pos_', num2str(i)])), nlp.Problem.TorsoAngles.LowerBound(:,i), nlp.Problem.TorsoAngles.UpperBound(:,i), nlp.Problem.TorsoAngles.Name);
    end

end

end
