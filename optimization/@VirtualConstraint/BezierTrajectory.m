function [phi,dphi_dtheta,d2phi_dtheta2] = BezierTrajectory(VirtualCon, alpha, s, theta_minus, theta_plus)

arguments
    VirtualCon (1,1) VirtualConstraint
    alpha (:,:) casadi.SX
    s (1,1) casadi.SX
    theta_minus (1,1) casadi.SX
    theta_plus (1,1) casadi.SX
end



% here theta could be time or a mechanical phase variable

[K,M] = size(alpha);
M = M-1;

phi = alpha(1,1)*zeros(K,1);
for k = 1:K
for m = 0:M
    phi(k) = phi(k) + alpha(k,m+1)*factorial(M)/(factorial(m)*factorial(M-m))*s^m*(1-s)^(M-m);
end
end



dphi_dtheta = alpha(1,1)*zeros(K,1);
for k = 1:K
for m = 0:M
    dphi_dtheta(k) = dphi_dtheta(k) + m*(1-s)^(M-m)*s^(m-1)*factorial(M)*alpha(k,m+1)/(factorial(m)*factorial(M-m)) - (M-m)*(1-s)^(M-m-1)*s^m*factorial(M)*alpha(k,m+1)/(factorial(m)*factorial(M-m));
end
end
dphi_dtheta = 1/(theta_minus-theta_plus)*dphi_dtheta;

    


d2phi_dtheta2 = alpha(1,1)*zeros(K,1);
for k = 1:K
for m = 0:M
    d2phi_dtheta2(k) = d2phi_dtheta2(k) + alpha(k,m+1) * factorial(M)/(factorial(m)*factorial(M-m)) * ( m*(m-1)*s^(m-2)*(1-s)^(M-m) - 2*(M-m)*m*s^(m-1)*(1-s)^(M-m-1) + (M-m)*(M-m-1)*s^m*(1-s)^(M-m-2) );
end
end
d2phi_dtheta2 = 1/(theta_minus-theta_plus)^2*d2phi_dtheta2;


end