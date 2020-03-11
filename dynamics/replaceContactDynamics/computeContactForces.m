function [Fc, ForceVector] = computeContactForces(obj, rbm)

arguments
    obj (1,1) ContactDynamics
    rbm (1,1) DynamicalSystem
end


NB = rbm.Model.nd;

import casadi.*

JcArg = obj.Jac_contact;
%mustBeMember(JcArg{1},'ContactJacobian')

dJcArg = obj.dJac_contact;
%mustBeMember(dJcArg{1},'ContactJacobianDer')

if size(JcArg(:),1) ~= size(dJcArg(:),1) || size(JcArg(:),2) ~= size(dJcArg(:),2)
    error('Error. There must be an equal number of Jc and dJc.')
end


% number of contacts
nc = numel(JcArg);

Fvec = rbm.States.q.sym(1)*zeros(NB,1);
for i = 1:nc
   
    nf_i = size(JcArg{i},1);

    Fc{i}.sym = SX.sym(['Fc_' num2str(i)], nf_i);
    Fc{i}.ID = 'cForce'; % every wrench has the same ID because they are defined within each phase
    % and the number of contacts is constant during each phase 
    
    Fvec_i = JcArg{i}'*Fc{i}.sym;
    
    
    Fvec = Fvec + Fvec_i;
    
end

ForceVector = Fvec;




if 0 % this lumps all forces together but can't enforce friction individually then
    % works for single contact too
    Jc_bloc = blkdiag(JcArg{:});

    % number of contact forces
    nf = size(Jc_bloc, 1);

    Fc = SX.sym('Fc',nf);

    Fmatrix = Jc_bloc'*Fc;

    Fvec = reshape(Fmatrix, NB, size(Fmatrix,1)/NB);

    ForceVector = sum(Fvec,2);

    size(ForceVector)

    obj.ForceVector = ForceVector;

end



end


