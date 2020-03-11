function [obj,position2enforce,velocity2enforce] = periodicity_constraints(obj,~, ~, ~ )
% x0 are the initial conditions
% xPlus are the values after impact

% NB = rbm.model.NB;
% 
% q_0  = x0(1:NB);
% qd_0 = x0(NB+1:2*NB);
% 
% q_plus  = xPlus(1:NB);
% qd_plus = xPlus(NB+1:2*NB);

%tol_per = 1E-12;

position2enforce = [3:7];
velocity2enforce = [1:7];

% % periodicity constraints
% [problem] = Optim.AddConstraint(problem, q_plus(3:7) - q_0(3:7) , -tol_per*ones(5,1), tol_per*ones(5,1) );
% [problem] = Optim.AddConstraint(problem, qd_plus - qd_0 , -tol_per*ones(NB,1), tol_per*ones(NB,1) );
% 




end
