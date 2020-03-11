function [nlp, grid_var] = AddStates(nlp, rbm, grid_var)


arguments
    nlp (1,1) NLP
    rbm (1,1) DynamicalSystem
    grid_var (1,1) struct
end
    

NB = numel(rbm.States.q.sym);


% idx is the number of the collocation point
%   e.g. idx = 0 means t = 0
for idx = 1:nlp.Settings.ncp
    
    q_ub = rbm.States.q.UpperBound(:, idx);
    q_lb = rbm.States.q.LowerBound(:, idx);
    q_guess = rbm.States.q.Seed(:, idx);

    dq_ub = rbm.States.dq.UpperBound(:, idx);
    dq_lb = rbm.States.dq.LowerBound(:, idx);
    dq_guess = rbm.States.dq.Seed(:, idx);

    ddq_ub = rbm.States.ddq.UpperBound(:, idx);
    ddq_lb = rbm.States.ddq.LowerBound(:, idx);
    ddq_guess = rbm.States.ddq.Seed(:, idx);


    [nlp, q_idx]   = AddVar(nlp, ['Q_', num2str(idx)]  , NB, q_guess  , q_lb  , q_ub  , 'pos');
    [nlp, dq_idx]  = AddVar(nlp, ['dQ_', num2str(idx)] , NB, dq_guess , dq_lb , dq_ub , 'vel');
    [nlp, ddq_idx] = AddVar(nlp, ['ddQ_', num2str(idx)], NB, ddq_guess, ddq_lb, ddq_ub, 'acc');


    grid_var.(['pos_', num2str(idx)]) = q_idx;
    grid_var.(['vel_', num2str(idx)]) = dq_idx;
    grid_var.(['acc_', num2str(idx)]) = ddq_idx;


end




end
