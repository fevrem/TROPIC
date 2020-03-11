function [nlp , grid_var] = AddVCParameters(nlp, rbm, grid_var)


arguments
    nlp (1,1) NLP
    rbm (1,1) DynamicalSystem
    grid_var (1,1) struct
end



% idx is the number of the collocation point
%   e.g. idx = 0 means t = 0


if nlp.Problem.Trajectory.Bool

    switch nlp.Problem.Trajectory.PolyType
        case 'Bezier'
            
            

            a_ub = nlp.Problem.Trajectory.PolyCoeff.UpperBound;
            a_lb = nlp.Problem.Trajectory.PolyCoeff.LowerBound;
            a_guess = nlp.Problem.Trajectory.PolyCoeff.Seed;

            n_coeff = numel(nlp.Problem.Trajectory.PolyCoeff.sym);

            [nlp, a] = add_var(nlp, 'a_', n_coeff, reshape(a_guess, n_coeff, 1), reshape(a_lb, n_coeff, 1), reshape(a_ub, n_coeff, 1), 'a');
            grid_var.a = a;


        otherwise
            error('MyComponent:incorrectType',...
                'Error. \nVC type must be ''Bezier'', not %s.', nlp.Problem.Trajectory.PolyType)
   
    end
    
    
end


 

end




