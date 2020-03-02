function  [px,py,pz,vx,vy,vz] = impSwingVel( model, q, qd )

% Objective of impSwingVel( model, q, qd ): to get the linear velocity
% of the swing foot required by the impact map
% 
% Martin Fevre (2019)
% 
% modified from Roy Featherston 
% inputs
%   model: the robot
%   q: the position states
%   qd: the velocity states








    % CasADi 3.4.5
    %import casadi.*

    
    % ----------- Geometric and inertial parameters ------------- %
    L_lower_leg = 0.5; % m
    L_upper_leg = 0.5; % m 
    L_pelvis    = 0.5; % m
    
    l{1} = 0;
    l{2} = 0;
    l{3} = 0;
    l{4} = 0;
    l{5} = 0;
    l{6} = L_lower_leg;
    l{7} = L_upper_leg;
    l{8} = 0;
    l{9} = L_pelvis;
    l{10} = 0;
    l{11} = L_upper_leg;
    l{12} = L_lower_leg;
    
    M_lower_leg = 0.0;  % kg
    M_upper_leg = 0.0;  % kg
    M_pelvis    = 0.0;  % kg
    
    m{1} = 0;
    m{2} = 0;
    m{3} = 0;
    m{4} = 0;
    m{5} = 0;
    m{6} = M_lower_leg;
    m{7} = M_upper_leg;
    m{8} = 0;
    m{9} = M_pelvis;
    m{10} = 0;
    m{11} = M_upper_leg;
    m{12} = M_lower_leg;

    
    % ------------ Denavit Hartenberg parameters ---------------- %
    % {0}-frame to {1}-frame
    d_0 = 0;
    theta_0 = pi/2;
    
    % {4}-frame to {5}-frame
    alpha{5} = -pi/2;
    a{5}     = 0;
    d{5}     = 0;
    
    % {5}-frame to {6}-frame
    d{6}     = 0;
    alpha{6} = pi/2;
    a{6}     = l{5};

    % {6}-frame to {7}-frame
    d{7}     = 0;
    a{7}     = l{6};
    alpha{7} = 0;
    
    % {7}-frame to {8}-frame
    d{8}     = 0;
    a{8}     = l{7};
    alpha{8} = 0;
    
    % {8}-frame to {9}-frame
    d{9}     = 0;
    alpha{9} = -pi/2;
    a{9}     = l{8};
    
    % {9}-frame to {10}-frame
    d{10}     = 0;
    alpha{10} = pi;
    a{10}     = l{9};
    
    % {10}-frame to {11}-frame
    d{11}     = 0;
    alpha{11} = pi/2;
    a{11}     = l{10};
    
    % {11}-frame to {12}-frame
    d{12}     = 0;
    alpha{12} = 0;
    a{12}     = l{11};
    
    % ----------- Tree transforms from DH parameters ------------- %    
    for i = 1:model.NB
        if i == 1 || i == 2 || i == 3 || i == 4
            model.Xtree{i} = eye(1);
        elseif i == 5
            model.Xtree{i} = xlt([a{i} 0 d{i}]')*round(rotx(alpha{i}))*round(rotz(theta_0))*xlt([0 0 d_0]');
        else
            model.Xtree{i} = xlt([a{i} 0 d{i}]')*round(rotx(alpha{i}));
        end
    end
    
    % -------------------- Spatial inertias ---------------------- %    
    for i = 1:model.NB
        if i == model.NB 
            m{i} = 1.0; % now the center of mass is at the end effector
            model.I{i} = mcI( m{i},...
                [l{i} 0 0],...
                diag([0,1/12*m{i}*l{i}^2,1/12*m{i}*l{i}^2]) );
        else
            model.I{i} = mcI( m{i},...
                [l{i}/2 0 0],...
                diag([0,1/12*m{i}*l{i}^2,1/12*m{i}*l{i}^2]) );
        end
    end  
    




KE = q(1)*0+zeros(model.NB,1);

for i = 1:model.NB
  [ XJ, S ] = jcalc( model.jtype{i}, q(i) );
  vJ = S*qd(i);
  Xup{i} = XJ * model.Xtree{i};
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

%ret.KE = sum(KE);
%ret.PE = - mass * dot(cm,g);
%ret.mass = mass;
%ret.cm = cm;
%ret.vcm = h / mass;
vcm = h / mass;

px = cm(1);
py = cm(2);
pz = cm(3);

vx = vcm(1);
vy = vcm(2);
vz = vcm(3);

















    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    













% 
% 
% 
% model.NB = model.NB + 1;
% model.parent = [model.parent,model.parent(end)+1];
% 
% % CasADi 3.4.5
% import casadi.*
% 
% % add orientation of end effector
% q_nul    = SX.sym('q_nul');
% qd_nul   = SX.sym('qd_nul');
% 
% q  = [ q ; q_nul ];
% qd = [ qd ; qd_nul ];
% 
% % using Denavit Hartenberg convention, R joints are in +Z-direction    
% model.jtype{length(model.jtype)+1} = 'Rz'; % q_nul
% 
% % {12}-frame to {nul}-frame
% d{13}     = 0;
% alpha{13} = 0;
% a{13}     = model.l{end};
% 
% % ----------- Tree transforms from DH parameters ------------- %    
% model.Xtree{13} = xlt([a{13} 0 d{13}]')*round(rotx(alpha{13}));
% 
% for i = 1:model.NB
%   [ XJ, S ] = jcalc( model.jtype{i}, q(i) );
%   vJ = S*qd(i);
%   Xup{i} = XJ * model.Xtree{i};
%   if model.parent(i) == 0
%     v{i} = vJ;
%   else
%     v{i} = Xup{i}*v{model.parent(i)} + vJ;
%   end
%   i
%   v{i}
% end


% vx = v{end}(4);
% vy = v{end}(5);
% vz = v{end}(6);

%x_extended = [ q ; qd ];

end

