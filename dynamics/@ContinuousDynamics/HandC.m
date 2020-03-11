function [H,C] = HandC(obj, sys)

% arguments
%     obj (1,1) {ContinuousDynamics,HybridDynamics}
%     sys (1,1) {DynamicalSystem,HybridDynamicalSystem}
% end





% HandC  Calculate coefficients of equation of motion
% [H,C]=HandC(model,q,qd,f_ext)  calculates the coefficients of the
% joint-space equation of motion, tau=H(q)qdd+C(d,qd,f_ext), where q, qd
% and qdd are the joint position, velocity and acceleration vectors, H is
% the joint-space inertia matrix, C is the vector of gravity,
% external-force and velocity-product terms, and tau is the joint force
% vector.  Algorithm: recursive Newton-Euler for C, and
% Composite-Rigid-Body for H.  f_ext is an optional argument specifying the
% external forces acting on the bodies.  It can be omitted if there are no
% external forces.  The format of f_ext is explained in the source code of
% apply_external_forces.
% 
% if nargin == 4
%     fprintf('\n==> Derive equations of motion in excess coordinates given by:')
%     fprintf('\nH(q)qdd + C(q,qd) = B(q)u + J^T(q)F')
%     fprintf('\nRNEA for C(q,qd) and CRBA for H(q)... ')
%     tic
% elseif nargin == 5
%     fprintf('\n==> Derive equations of motion in minimal coordinates given by:')
%     fprintf('\nHm(qm)qddm + Cm(qm,qdm) = Bm(qm)u')
%     fprintf('\nRNEA for Cm(qm,qdm) and CRBA for Hm(qm)... ')
%     tic
% else
%     error('Wrong number of arguments')
% end


model = sys.Model;
Xtree = sys.Model.Xtree;

q = sys.States.q.sym;
qd = sys.States.dq.sym;


nd = model.nd;

a_grav = get_gravity( sys.Model );

for i = 1:nd
  [ XJ, S{i} ] = jcalc( model.jtype{i}, q(i) );
  vJ = S{i}*qd(i);
  Xup{i} = XJ * Xtree{i};
  if model.parent(i) == 0
    v{i} = vJ;
    avp{i} = Xup{i} * -a_grav;
  else
    v{i} = Xup{i}*v{model.parent(i)} + vJ;
    avp{i} = Xup{i}*avp{model.parent(i)} + crm(v{i})*vJ;
  end
  fvp{i} = model.I{i}*avp{i} + crf(v{i})*model.I{i}*v{i};
end



C = CoriolisAndGravityTerms(obj, sys, q, S, Xup, fvp);
   
H = InertiaMatrix(obj, sys, q, S, Xup);



end
