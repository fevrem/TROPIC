function [con] = RightStance(con, rbm, ncp)%, opt)

arguments
    con (1,:) cell
    rbm (1,1) DynamicalSystem
    ncp (1,1) double
end
% arguments (Repeating)
%     opt (1,1) double
% end



% if nargin == 2
%     tol = 1E-4;
% else
%     tol = varargin{1};
% end
% 
% tol

%import casadi.*



% right leg end
rbm.Contacts{1} = Contact(rbm, 'Point',...
    {'Friction', true},...
    {'FrictionCoefficient', 0.6},...
    {'FrictionType', 'Pyramid'},...
    {'ContactFrame', rbm.BodyPositions{12,2}});

rbm.Contacts{1}.Fc.LowerBound = -1000*ones(3,1);
rbm.Contacts{1}.Fc.UpperBound = 1000*ones(3,1);
rbm.Contacts{1}.Fc.Seed = 100*ones(3, ncp);



% left leg end
% rbm.Contacts{2} = Contact(rbm, 'Point',...
%     {'Friction', true},...
%     {'FrictionCoefficient', 0.6},...
%     {'FrictionType', 'Cone'},...
%     {'ContactFrame', rbm.BodyPositions{9,2}});
% 
% rbm.Contacts{2}.Fc.LowerBound = -1000*ones(3,1);
% rbm.Contacts{2}.Fc.UpperBound = 1000*ones(3,1);
% rbm.Contacts{2}.Fc.Seed = 100*ones(3, ncp);



%{
    Obtain contact dynamics
%}
%contactD = ContactDynamics(rbm, contacts);


con = Constraint.addConstraint(con);
con{end}.Name = 'Equations of Motion';
con = ContinuousSecondOrder(con, rbm);%, contactD); % contactD is optional

% add holonomic constraints in one stroke, regardless of number of contacts
con = Constraint.addConstraint(con);
con{end}.Name = 'Equations of Motion (Holonomic Constraints)';
con = ConstrainedDynamics(con, rbm);%, contactD); 


con = Constraint.addConstraint(con);
con{end}.Name = 'Friction';
con = EnforceFriction(con, rbm);%, contactD);


end
