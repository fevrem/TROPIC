function [obj] = ConfigFunctions(obj, rbm)

arguments
    obj (1,1) NLP
    rbm (1,1) DynamicalSystem
end

import casadi.*

% 2nd-order dynamics
obj.Functions.DynamicsODE = ContinuousSecondOrder(obj, rbm);

% Constrained 2nd-order dynamics
nContact = size(rbm.Contacts, 2);
if nContact ~= 0
    obj.Functions.ConstrainedDynamicsODE = ConstrainedDynamics(obj, rbm);
end



% inertia matrix needed for impact
obj.Functions.InertiaMatrix = Function('f', {rbm.States.q.sym} , {rbm.Dynamics.H_matrix} );


% contact jacobian for impact
if nContact ~= 0
    for i = 1:nContact
        
        obj.Functions.ContactJacobian{i} = Function('f', {rbm.States.q.sym}, {rbm.Contacts{i}.Jac_contact});

    end
end


% position and velocity of center of mass        
obj.Functions.PositionCOM = Function('f', {rbm.States.q.sym} , {rbm.Dynamics.p_com});
obj.Functions.VelocityCOM = Function('f', {rbm.States.q.sym, rbm.States.dq.sym} , {rbm.Dynamics.v_com});




end