function  ret = EnerMo( model, q, qd )

% EnerMo  calculate energy, momentum and related quantities
% EnerMo(robot,q,qd)  returns a structure containing the fields KE, PE,
% htot, Itot, mass, cm and vcm.  These fields contain the kinetic and
% potential energies of the whole system, the total spatial momentum, the
% total spatial inertia, total mass, position of centre of mass, and the
% linear velocity of centre of mass, respectively.  Vector quantities are
% expressed in base coordinates.  PE is defined to be zero when cm is
% zero.

KE = q(1)*0+zeros(model.NB,1);

for i = 1:model.NB
  [ XJ, S ] = jcalc( model.jtype{i}, q(i) );
  vJ = S*qd(i);
  Xup{i} = XJ * model.Xtree{i};
  %Xup{i}
  if model.parent(i) == 0
    v{i} = vJ;
  else
    v{i} = Xup{i}*v{model.parent(i)} + vJ;
  end
  Ic{i} = model.I{i};
  hc{i} = Ic{i} * v{i};
  KE(i) = 0.5 * v{i}' * hc{i};
end

ret.Itot = zeros(size(Ic{1}));
ret.htot = zeros(size(hc{1}));



% xval = rand;
% yval = rand;
% zval = rand;
% phizval = rand;
% q1val = rand;
% q2val = rand;
% q3val = rand;
% q4val = rand;
% syms x y z phiz q1 q2 q3 q4 real

% Martin Fevre (2019)
for i = 1:model.NB
    XJ = jcalc( model.jtype{i}, q(i) );
    Xa{i} = XJ * model.Xtree{i};
    if model.parent(i) ~= 0
      Xa{i} = Xa{i} * Xa{model.parent(i)};
      Xa{i} = simplify(Xa{i});
    end
    if size(Xa{i},1) == 3		% Xa{i} is a planar coordinate xform
      [theta,r] = plnr(Xa{i});
      X = rotz(theta) * xlt([r;0]);
      T = pluho(X);
    else
      T = pluho(Xa{i});
    end

    % Martin Fevre 
    % this is super inefficient
    %tic
    %Tdisp = inv(T);		% displacement is inverse of coord xform
    %toc
    %tic
    Rtranspose = T(1:3,1:3)';
    Rtranspose_times_p = Rtranspose*[T(1:3,4)];
    Tdisp2 = [Rtranspose(1,1),Rtranspose(1,2),Rtranspose(1,3),-Rtranspose_times_p(1);
             Rtranspose(2,1),Rtranspose(2,2),Rtranspose(2,3),-Rtranspose_times_p(2);
             Rtranspose(3,1),Rtranspose(3,2),Rtranspose(3,3),-Rtranspose_times_p(3);
             0,0,0,1];
    %syms x y z phiz q1 q2 q3 q4 q5 q6 q7 q8 real
    %double(subs(Tdisp,{x,y,z,phiz,q1,q2,q3,q4,q5,q6,q7,q8},{1,2,3,4,5,6,7,8,9,10,11,12}))
    %double(subs(Tdisp2,{x,y,z,phiz,q1,q2,q3,q4,q5,q6,q7,q8},{1,2,3,4,5,6,7,8,9,10,11,12}))
    %double(subs(Tdisp,{x,y,z,phiz,q1,q2,q3,q4,q5,q6,q7,q8},{1,2,3,4,5,6,7,8,9,10,11,12}))-double(subs(Tdisp2,{x,y,z,phiz,q1,q2,q3,q4,q5,q6,q7,q8},{1,2,3,4,5,6,7,8,9,10,11,12}))
    %toc

    Tr{i} = T;
    
    
    Pjt{i} = Tdisp2(1:3,end);

    %double(subs(Pjt,{x,y,z,phiz,q1,q2,q3,q4},{xval,yval,zval,phizval,q1val,q2val,q3val,q4val}))

    % where is the swing foot?
    if i == model.NB
        Psw_f = [model.l{i};0;0;1];
        Pjt_temp = Tdisp2*Psw_f;
        Pjt{i+1} = Pjt_temp(1:3);
    end

    
end
ret.Pjt = Pjt; 
ret.Tr = Tr;



for i = model.NB:-1:1
  if model.parent(i) ~= 0
    Ic{model.parent(i)} = Ic{model.parent(i)} + Xup{i}'*Ic{i}*Xup{i};
    hc{model.parent(i)} = hc{model.parent(i)} + Xup{i}'*hc{i};
  else
    ret.Itot = ret.Itot + Xup{i}'*Ic{i}*Xup{i};
    ret.htot = ret.htot + Xup{i}'*hc{i};
  end
end

a_grav = get_gravity(model);

if length(a_grav) == 6
  g = a_grav(4:6);			% 3D linear gravitational accn
  h = ret.htot(4:6);			% 3D linear momentum
else
  g = a_grav(2:3);			% 2D gravity
  h = ret.htot(2:3);			% 2D linear momentum
end


% homepathhack = pwd;
% cd([homepathhack filesep 'dynamics'])
[mass, cm] = mcI(ret.Itot);
% cd(homepathhack)

ret.KE = sum(KE);
ret.PE = - mass * dot(cm,g);
%ret.mass = mass;
ret.cm = cm;
ret.vcm = h / mass;
