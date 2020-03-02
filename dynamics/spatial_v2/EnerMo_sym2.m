function  [ T , phi_sym , psi_sym ,theta_sym ] = EnerMo_sym2( model, q )
 
% EnerMo  calculate energy, momentum and related quantities
% EnerMo(robot,q,qd)  returns a structure containing the fields KE, PE,
% htot, Itot, mass, cm and vcm.  These fields contain the kinetic and
% potential energies of the whole system, the total spatial momentum, the
% total spatial inertia, total mass, position of centre of mass, and the
% linear velocity of centre of mass, respectively.  Vector quantities are
% expressed in base coordinates.  PE is defined to be zero when cm is
% zero.

  

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



    



% derive flip-map in CasADi syms
%Tr = EnerMo_sym2( robot, q ); % syms equivalent available in 'hom_trans_syms.mat'

% rotation matrices
R_0_to_1  = T{1}(1:3,1:3);
%R_0_to_6  = T{6}(1:3,1:3);     
R_0_to_12 = T{12}(1:3,1:3); % Rot from swing shank to {0}

% I want IK solution from {1}-frame -> where first rotation happens (yaw)
%R_1_to_6 = R_0_to_6 * R_0_to_1';
R_1_to_12 = R_0_to_12 * R_0_to_1';

% introduce {13}-frame "pointing in same direction" as {0}
R_13_to_12 = [-1 , 0 , 0 ;
               0 , 1 , 0 ;
               0 , 0 ,-1 ];

R_1_to_13 = R_13_to_12' * R_1_to_12 ;       

% quantities needed to solve the inverse kinematics problem
r13 = R_1_to_13(1,3);
r23 = R_1_to_13(2,3);
r31 = R_1_to_13(3,1);
r32 = R_1_to_13(3,2);
r33 = R_1_to_13(3,3);

% solve IK problem
phi_sym = atan2( sqrt(r31^2+r32^2) , r33 );  % roll (second rotation, q1)

%
%psi_sym = atan2( r32/sin(phi_sym)  , r31/sin(phi_sym) ); % yaw (first rotation, phi^z_0)
%
psi_sym = atan2( -(r32/sin(phi_sym)) , -(r31/sin(phi_sym)) ) + pi; % yaw (first rotation, phi^z_0)

theta_sym = atan2( -(r23/sin(phi_sym)) , -(-r13/sin(phi_sym)) ) + pi; % pitch (third rotation, q2)





end