function con = ContinuousSecondOrder(con, rbm)%, contactDyna)%varargin)

arguments
    con (1,:) cell
    rbm (1,1) DynamicalSystem
    %contactDyna (1,1) ContactDynamics
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

[variables, variablesID] = assignVars({rbm.States.q,...
    rbm.States.dq,...
    rbm.States.ddq,...
    rbm.Inputs.u});


nContact = size(rbm.Contacts, 2);

if nContact ~= 0
sumContactForces = q(1)*zeros(NB,1);
Fc_vars_sym = [];
Fc_vars_ID = {};
for i = 1:nContact
    
    Jc_i = rbm.Contacts{i}.Jac_contact;
    Fc_i = rbm.Contacts{i}.Fc.sym;
    
    sumContactForces = sumContactForces + Jc_i'*Fc_i;
    
    %Fc_variables = {}
    Fc_vars_sym = [Fc_vars_sym; rbm.Contacts{i}.Fc.sym];
    
    Fc_vars_ID{end+1} = rbm.Contacts{i}.Fc.ID;
    
end



for i = 2:nContact
    if ~strcmp(Fc_vars_ID{i}, Fc_vars_ID{1})
        error('Treating all the contact forces together but they have different ID.')
    end
end
%Fc_vars_ID = Fc_vars_ID{1};

second_order_ode = second_order_ode - sumContactForces;
variables{end+1} = Fc_vars_sym;
variablesID{end+1} = Fc_vars_ID{1};


end

odefun = Function('f', variables, {second_order_ode} );

con{end}.SymbolicExpression = second_order_ode;
con{end}.DependentVariables = variables;
con{end}.DependentVariablesID = variablesID;
con{end}.Function = odefun;

    
    
end