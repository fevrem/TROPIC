% function [holonomic_constraints, variables, variable_names, hol_fun] = ConstrainedDynamics(rbm, contactD)%JcArg, dJcArg)
function hol_fun = ConstrainedDynamics(nlp, rbm)%, contactD)%JcArg, dJcArg)


arguments
    nlp (1,1) NLP
    rbm (1,1) DynamicalSystem
    %contactD (1,1) ContactDynamics
end


import casadi.*


% number of DOF
NB = rbm.Model.nd;

% states
q   = rbm.States.q.sym;
qd  = rbm.States.dq.sym;
qdd = rbm.States.ddq.sym;


nContact = size(rbm.Contacts, 2);
mustBePositive(nContact)

holonomic_constraints = q(1)*[];

%sumContactForces = q(1)*zeros(NB,1);
%Fc_vars_sym = [];
%Fc_vars_ID = {};
for i = 1:nContact

    Jc = rbm.Contacts{i}.Jac_contact;
    
    dJc = rbm.Contacts{i}.dJac_contact;
    
    holonomic_constraints = [holonomic_constraints; Jc*qdd + dJc*qd];
    

end




hol_fun = Function('f', {q, qd, qdd}, {holonomic_constraints} );                





    
end