function  He = He( model, q )

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




for i = 1:model.NB
  [ XJ, S{i} ] = jcalc( model.jtype{i}, q(i) );
  Xup{i} = XJ * model.Xtree{i};
end



IC = model.I;				% composite inertia calculation

for i = model.NB:-1:1
  if model.parent(i) ~= 0
    IC{model.parent(i)} = IC{model.parent(i)} + Xup{i}'*IC{i}*Xup{i};
  end
end


% Martin Fevre (2019)
He = q(1)*0 + zeros(model.NB);
for i = 1:model.NB
  
  fh = IC{i} * S{i};
  
  He(i,i) = S{i}' * fh;
  j = i;
  while model.parent(j) > 0
    fh = Xup{j}' * fh;
    j = model.parent(j);
    He(i,j) = S{j}' * fh;
    He(j,i) = He(i,j);
  end
end


end
