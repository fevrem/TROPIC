function vel_body = GetBodyVelocities(obj)

arguments
    obj (1,1) DynamicalSystem
end
  

import casadi.*

model = obj.Model;
nd = model.nd;

% T = obj.HTransforms; 


% Martin Fevre (2019)
for i = 1:nd
    
    Jac_1 = jacobian(obj.BodyPositions{i,1}, obj.States.q.sym);
    Jac_2 = jacobian(obj.BodyPositions{i,2}, obj.States.q.sym);
    
    vel_body{i,1} = Jac_1*obj.States.dq.sym;
    vel_body{i,2} = Jac_2*obj.States.dq.sym;
    
end




end