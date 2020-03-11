function [nlp] = FootHeight(nlp, rbm, grid_var)


%% Argument Validation
arguments
    nlp (1,1) NLP
    rbm (1,1) DynamicalSystem
    grid_var (1,1) struct
end



% ground clearance 
if nlp.Problem.GroundClearance.Bool

    for i = 1:nlp.Settings.ncp

        % swing foot height 
        if nlp.Problem.SwingFootHeight.Bool
            idx_cp = nlp.Problem.SwingFootHeight.Timing;
            if idx_cp == i
                mustBeInteger(idx_cp)
                mustBeGreaterThan(idx_cp,1)
                mustBeLessThan(idx_cp,nlp.Settings.ncp)

                [nlp] = AddConstraint(nlp, nlp.Problem.SwingFootHeight.Function(grid_var.(['pos_', num2str(idx_cp)])), nlp.Problem.SwingFootHeight.LowerBound, nlp.Problem.SwingFootHeight.UpperBound, [nlp.Problem.SwingFootHeight.Name, '@', num2str(idx_cp)]);
                continue; % do not enforce redundant constraints
            end

        end

        [nlp] = AddConstraint(nlp, nlp.Problem.GroundClearance.Function(grid_var.(['pos_', num2str(i)])), nlp.Problem.GroundClearance.LowerBound, nlp.Problem.GroundClearance.UpperBound, [nlp.Problem.GroundClearance.Name, '@', num2str(i)]);

    end

% not sure why it would make sense to have swing foot height but not ground clearance    
else

    % swing foot height 
    if nlp.Problem.SwingFootHeight.Bool
        idx_cp = nlp.Problem.SwingFootHeight.Timing;
        mustBeInteger(idx_cp)
        mustBeGreaterThan(idx_cp,1)
        mustBeLessThan(idx_cp,nlp.Settings.ncp)

        [nlp] = AddConstraint(nlp, nlp.Problem.SwingFootHeight.Function(grid_var.(['pos_', num2str(idx_cp)])), nlp.Problem.SwingFootHeight.LowerBound, nlp.Problem.SwingFootHeight.UpperBound, [nlp.Problem.SwingFootHeight.Name, '@', num2str(idx_cp)]);

    end


end




end

