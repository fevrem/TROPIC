function C = CoriolisAndGravityTerms(obj, sys, q, S, Xup, fvp)

% arguments
%     obj (1,1) %RobotMotion
%     q (:,1) casadi.SX
%     S
%     Xup
%     fvp
% end

model = sys.Model;

%q = model.q;
parent = model.parent;

nd = model.nd;

% Martin Fevre (2019)
C = q(1)*0+zeros(nd,1);
for i = nd:-1:1
    C(i,1) = S{i}' * fvp{i};
    if parent(i) ~= 0
        fvp{parent(i)} = fvp{parent(i)} + Xup{i}'*fvp{i};
    end
end

end