function cost = RunningCost(rbm, q, qd, qdd, u)

arguments
   rbm (1,1) DynamicalSystem
   q (:,1) casadi.MX
   qd (:,1) casadi.MX
   qdd (:,1) casadi.MX
   u (:,1) casadi.MX
end
                

cost = u'*u;



end