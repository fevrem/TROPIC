function  T = EnerMo_sym( model, q )

% EnerMo  calculate energy, momentum and related quantities
% EnerMo(robot,q,qd)  returns a structure containing the fields KE, PE,
% htot, Itot, mass, cm and vcm.  These fields contain the kinetic and
% potential energies of the whole system, the total spatial momentum, the
% total spatial inertia, total mass, position of centre of mass, and the
% linear velocity of centre of mass, respectively.  Vector quantities are
% expressed in base coordinates.  PE is defined to be zero when cm is
% zero.

% KE = q(1)*0+zeros(model.NB,1);
% 
% for i = 1:model.NB
%   [ XJ, S ] = jcalc( model.jtype{i}, q(i) );
%   vJ = S*qd(i);
%   Xup{i} = XJ * model.Xtree{i};
%   %Xup{i}
%   if model.parent(i) == 0
%     v{i} = vJ;
%   else
%     v{i} = Xup{i}*v{model.parent(i)} + vJ;
%   end
% 
% end








model.NB = 8;

model.parent = 0:7;
    
model.gravity = [0 0 -9.81]';

q = q(5:12)

% using Denavit Hartenberg convention, R joints are in +Z-direction
model.jtype{1}  = 'Rz'; % q1
model.jtype{2}  = 'Rz'; % q2
model.jtype{3}  = 'Rz'; % q3
model.jtype{4}  = 'Rz'; % q4
model.jtype{5}  = 'Rz'; % q5
model.jtype{6} = 'Rz'; % q6
model.jtype{7} = 'Rz'; % q7
model.jtype{8} = 'Rz'; % q8
    
% ----------- Geometric and inertial parameters ------------- %
L_lower_leg = 0.5; % m
L_upper_leg = 0.5; % m 
L_pelvis    = 0.5; % m
L_torso     = 0.5; % m

l{1} = 0;
l{2} = L_lower_leg;
l{3} = L_upper_leg;
l{4} = 0;
l{5} = L_pelvis;
l{6} = 0;
l{7} = L_upper_leg;
l{8} = L_lower_leg;
model.l = l;

M_lower_leg = 5.0;  % kg
M_upper_leg = 5.0;  % kg
M_pelvis    = 5.0;  % kg
M_torso     = 15.0; % kg

m{1} = 0;
m{2} = M_lower_leg;
m{3} = M_upper_leg;
m{4} = 0;
m{5} = M_pelvis;
m{6} = 0;
m{7} = M_upper_leg;
m{8} = M_lower_leg;
model.m = m;

model.total_mass = sum([model.m{:}]) + M_torso;

% ------------ Denavit Hartenberg parameters ---------------- %
% {0}-frame to {1}-frame
d_0 = 0;
theta_0 = pi/2;

% {4}-frame to {5}-frame
alpha{1} = -pi/2;
a{1}     = 0;
d{1}     = 0;

% {5}-frame to {6}-frame
d{2}     = 0;
alpha{2} = pi/2;
a{2}     = l{1};

% {6}-frame to {7}-frame
d{3}     = 0;
a{3}     = l{2};
alpha{3} = 0;

% {7}-frame to {8}-frame
d{4}     = 0;
a{4}     = l{3};
alpha{4} = 0;

% {8}-frame to {9}-frame
d{5}     = 0;
alpha{5} = -pi/2;
a{5}     = l{4};

% {9}-frame to {10}-frame
d{6}     = 0;
alpha{6} = pi;
a{6}     = l{5};

% {10}-frame to {11}-frame
d{7}     = 0;
alpha{7} = pi/2;
a{7}     = l{6};

% {11}-frame to {12}-frame
d{8}     = 0;
alpha{8} = 0;
a{8}     = l{7};

for i = 1:model.NB
    if i == 1
        model.Xtree{i} = xlt([a{i} 0 d{i}]')*round(rotx(alpha{i}))*round(rotz(theta_0))*xlt([0 0 d_0]');
    else
        model.Xtree{i} = xlt([a{i} 0 d{i}]')*round(rotx(alpha{i}));
    end
end


   





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
      T{i} = pluho(X);
    else
      T{i} = pluho(Xa{i});
    end
end
%Tr = T; 


end