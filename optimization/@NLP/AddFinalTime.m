function [nlp , grid_var] = AddFinalTime(nlp, rbm, grid_var)


arguments
    nlp (1,1) NLP
    rbm (1,1) DynamicalSystem
    grid_var (1,1) struct
end


tf_guess = nlp.Problem.FinalTime.Value;
tf_lb = nlp.Problem.FinalTime.LowerBound;
tf_ub = nlp.Problem.FinalTime.UpperBound;


% final time         
[nlp, tf] = add_var(nlp, 'tf', 1, tf_guess, tf_lb, tf_ub, 'tf');


grid_var.tf = tf;

 

end




