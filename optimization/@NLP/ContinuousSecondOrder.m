function odefun = ContinuousSecondOrder(nlp, rbm)

arguments
    nlp (1,1) NLP
    rbm (1,1) DynamicalSystem
end


import casadi.*


% number of DOF
NB = rbm.Model.nd;

% states
q   = rbm.States.q.sym;
qd  = rbm.States.dq.sym;
qdd = rbm.States.ddq.sym;


% define control as decision variables
u = rbm.Inputs.u.sym;

% inertia matrix
H = rbm.Dynamics.H_matrix;

% vector of coriolis and gravitational terms
C = rbm.Dynamics.C_terms;

% input matrix
B = rbm.InputMap;


second_order_ode = H*qdd + C - B*u;
variables = {q,qd,qdd,u};


nContact = size(rbm.Contacts, 2);

if nContact ~= 0
    

    for i = 1:nContact

        Jc_i = rbm.Contacts{i}.Jac_contact;
        Fc_i = rbm.Contacts{i}.Fc.sym;

        second_order_ode = second_order_ode - Jc_i'*Fc_i;
        
        variables{end+1} = rbm.Contacts{i}.Fc.sym;
        

    end

end

odefun = Function('f', variables, {second_order_ode} );


    
    
end