function alpha = AddBezierCoefficients(VirtualCon, rbm)%bezier_order, n_aDOF)

arguments
    VirtualCon (1,1) VirtualConstraint
    rbm (1,1) DynamicalSystem
end


import casadi.* % CasADi 3.4.5

    
% number of actuated DOF
number_actuated_DOF = size(rbm.InputMap,2);

% order of bezier polynomial
bezier_order = VirtualCon.PolyOrder;

    
        

%fprintf(['\n' num2str(M) 'th-order Bezier polynomials (' num2str(n_aDOF*(M+1)) ' coefficients)'])

alpha = [];
for i = 1:number_actuated_DOF

    a{i} = SX.sym(['a' num2str(i)],bezier_order+1);
    
    %a{i} = Var(['a' num2str(i)],bezier_order+1)
    
    alpha = [ alpha ; a{i}' ];

    
end

%fprintf('\nalpha = ')
%disp(alpha)

end