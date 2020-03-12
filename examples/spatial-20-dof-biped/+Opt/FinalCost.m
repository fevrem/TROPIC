function cost = FinalCost(rbm, cost, tf, q, qd, qdd, u)

arguments
   rbm (1,1) DynamicalSystem
   cost (1,1) casadi.MX
   tf (1,1) casadi.MX
   q (:,1) casadi.MX
   qd (:,1) casadi.MX
   qdd (:,1) casadi.MX
   u (:,1) casadi.MX
end
                
cost = tf*cost;

end