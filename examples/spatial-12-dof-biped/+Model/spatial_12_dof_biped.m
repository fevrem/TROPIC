function  robot = spatial_12_dof_biped()
% robot.Xtree{i}: coordinate transform from coordinate system of body parent(i)
%   to coordinate system of the predecessor frame of joint i



import casadi.*

robot.name = '12 DOF 5-Link Spatial Biped';

% number of DOF
robot.nd = 12;
% robot.q   = SX.sym('',robot.nd);
% robot.qd  = SX.sym('',robot.nd);
% robot.qdd = SX.sym('',robot.nd);

% geometric and inertial parameters
model_prms = Model.model_params();

% for plotting reasons
r_link = 0.01;
r_joint = 0.015;

%{
    floating base
%}
robot.parent(1) = 0;  
robot.body_axis{1} = [0 0 0];
robot.body_length{1} = 0;
robot.Xtree{1} = eye(6);
robot.jtype{1}  = 'Px';
robot.I{1} = zeros(6);
robot.appearance.body{1} = Anim.add_appearance('base', [], [], false);
%robot.q(1)   = SX.sym('x_h');
%robot.qd(1)  = SX.sym('dx_h');
%robot.qdd(1) = SX.sym('ddx_h');
%robot.q(1) = Var('x_h', 1);
%robot.qd(1) = Var('dx_h', 1);
%robot.qdd(1) = Var('ddx_h', 1);

robot.parent(2) = 1; 
robot.body_axis{2} = [0 0 0];
robot.body_length{2} = 0;
robot.Xtree{2} = eye(6);
robot.jtype{2}  = 'Py';
robot.I{2} = zeros(6);
robot.appearance.body{2} = Anim.add_appearance('base', [], [], false);
%robot.q(2)   = SX.sym('y_h');
%robot.qd(2)  = SX.sym('dy_h');
%robot.qdd(2) = SX.sym('ddy_h');
%robot.q(2) = Var('y_h', 1);
%robot.qd(2) = Var('dy_h', 1);
%robot.qdd(2) = Var('ddy_h', 1);


robot.parent(3) = 2;    
robot.body_axis{3} = [0 0 0];
robot.body_length{3} = 0;
robot.Xtree{3} = eye(6);
robot.jtype{3}  = 'Pz';
robot.I{3} = zeros(6);
robot.appearance.body{3} = Anim.add_appearance('base', [], [], false);
%robot.q(3)   = SX.sym('z_h');
%robot.qd(3)  = SX.sym('dz_h');
%robot.qdd(3) = SX.sym('ddz_h');
%robot.q(12) = Var('q_k1', 1);
%robot.qd(12) = Var('dq_k1', 1);
%robot.qdd(12) = Var('ddq_k1', 1);

robot.parent(4) = 3;   
robot.body_axis{4} = [0 0 0];
robot.body_length{4} = 0;
robot.Xtree{4} = eye(6);
robot.jtype{4}  = 'Rx';
robot.I{4} = zeros(6);
robot.appearance.body{4} = Anim.add_appearance('base', [], [], false);
%robot.q(4)   = SX.sym('phi_x');
%robot.qd(4)  = SX.sym('dphi_x');
%robot.qdd(4) = SX.sym('ddphi_x');
%robot.q(12) = Var('q_k1', 1);
%robot.qd(12) = Var('dq_k1', 1);
%robot.qdd(12) = Var('ddq_k1', 1);
%robot.q(12) = Var('q_k1', 1);
%robot.qd(12) = Var('dq_k1', 1);
%robot.qdd(12) = Var('ddq_k1', 1);

robot.parent(5) = 4;    
robot.body_axis{5} = [0 0 0];
robot.body_length{5} = 0;
robot.Xtree{5} = eye(6);
robot.jtype{5}  = 'Ry';
robot.I{5} = zeros(6);
robot.appearance.body{5} = Anim.add_appearance('base', [], [], false);
% robot.q(5)   = SX.sym('phi_y');
% robot.qd(5)  = SX.sym('dphi_y');
% robot.qdd(5) = SX.sym('ddphi_y');
% robot.q(12) = Var('q_k1', 1);
% robot.qd(12) = Var('dq_k1', 1);
% robot.qdd(12) = Var('ddq_k1', 1);

% torso
robot.parent(6) = 5;    
robot.body_axis{6} = [0 0 1];
robot.body_length{6} = model_prms.L_torso;
robot.Xtree{6} = eye(6);
robot.jtype{6}  = 'Rz';
robot.I{6} = mcI(model_prms.M_torso, model_prms.Lcom_torso*robot.body_axis{6}, diag(1/12*model_prms.M_torso*model_prms.L_torso^2*[1 1 0])); % torso
body_appearance{1}.length = model_prms.L_torso;
body_appearance{1}.axis = robot.body_axis{6};
body_appearance{1}.color = model_prms.C_torso;
body_appearance{1}.radius = r_link;
joint_appearance{1}.radius = r_joint;
joint_appearance{1}.axis = robot.body_axis{6};
joint_appearance{1}.distance = 0;
joint_appearance{2}.radius = r_joint;
joint_appearance{2}.axis = robot.body_axis{6};
joint_appearance{2}.distance = model_prms.L_torso;
robot.I{6} = robot.I{6} + mcI(model_prms.M_pelvis/2, model_prms.Wcom_pelvis/2*[0 1 0], diag(1/12*model_prms.M_pelvis*(model_prms.W_pelvis/2)^2*[1 0 1])); % half pelvis (1)
body_appearance{2}.length = model_prms.W_pelvis/2;
body_appearance{2}.axis = [0 1 0];
body_appearance{2}.color = model_prms.C_torso;
body_appearance{2}.radius = r_link;
robot.I{6} = robot.I{6} + mcI(model_prms.M_pelvis/2, model_prms.Wcom_pelvis/2*[0 -1 0], diag(1/12*model_prms.M_pelvis*(model_prms.W_pelvis/2)^2*[-1 0 -1])); % half pelvis (2)
body_appearance{3}.length = model_prms.W_pelvis/2;
body_appearance{3}.axis = [0 -1 0];
body_appearance{3}.color = model_prms.C_torso;
body_appearance{3}.radius = r_link;
robot.appearance.body{6} = Anim.add_appearance('slender-rod', body_appearance, joint_appearance, false);
% robot.q(6)   = SX.sym('phi_z');
% robot.qd(6)  = SX.sym('dphi_z');
% robot.qdd(6) = SX.sym('ddphi_z');
% robot.q(12) = Var('q_k1', 1);
% robot.qd(12) = Var('dq_k1', 1);
% robot.qdd(12) = Var('ddq_k1', 1);

% qj1
clear body_appearance joint_appearance
robot.parent(7) = 6;  
robot.body_axis{7} = [0 0 0];
robot.body_length{7} = 0;
robot.Xtree{7} = plux(eye(3), model_prms.W_pelvis/2*[0 1 0]);
robot.jtype{7}  = 'Rx';
robot.I{7} = zeros(6);
robot.appearance.body{7} = Anim.add_appearance('joint', [], [], false);
% robot.q(7)   = SX.sym('q_abad1');
% robot.qd(7)  = SX.sym('dq_abad1');
% robot.qdd(7) = SX.sym('ddq_abad1');
% robot.q(12) = Var('q_k1', 1);
% robot.qd(12) = Var('dq_k1', 1);
% robot.qdd(12) = Var('ddq_k1', 1);

% qj2
clear body_appearance joint_appearance
robot.parent(8) = 7;    
robot.body_axis{8} = [0 0 -1];
robot.body_length{8} = model_prms.L_upper_leg;
robot.Xtree{8} = eye(6);
robot.jtype{8}  = 'Ry';
robot.I{8} = mcI(model_prms.M_upper_leg, model_prms.Lcom_upper_leg*robot.body_axis{8}, diag(1/12*model_prms.M_upper_leg*model_prms.L_upper_leg^2*[-1 -1 0]));
body_appearance{1}.length = model_prms.L_upper_leg;
body_appearance{1}.axis = robot.body_axis{8};
body_appearance{1}.color = model_prms.C_leg1;
body_appearance{1}.radius = r_link;
joint_appearance{1}.radius = r_joint;
joint_appearance{1}.axis = robot.body_axis{8};
joint_appearance{1}.distance = 0;
robot.appearance.body{8} = Anim.add_appearance('slender-rod', body_appearance, joint_appearance, false);
% robot.q(8)   = SX.sym('q_th1');
% robot.qd(8)  = SX.sym('dq_th1');
% robot.qdd(8) = SX.sym('ddq_th1');
% robot.q(12) = Var('q_k1', 1);
% robot.qd(12) = Var('dq_k1', 1);
% robot.qdd(12) = Var('ddq_k1', 1);

% qj3
clear body_appearance joint_appearance
robot.parent(9) = 8;  
robot.body_axis{9} = [0 0 -1];
robot.body_length{9} = model_prms.L_lower_leg;
robot.Xtree{9} = plux(eye(3), model_prms.L_upper_leg*robot.body_axis{9});
robot.jtype{9}  = 'Ry';
robot.I{9} = mcI(model_prms.M_lower_leg, model_prms.Lcom_lower_leg*robot.body_axis{9}, diag(1/12*model_prms.M_lower_leg*model_prms.L_lower_leg^2*[-1 -1 0]));
body_appearance{1}.length = model_prms.L_lower_leg;
body_appearance{1}.axis = robot.body_axis{9};
body_appearance{1}.color = model_prms.C_leg1;
body_appearance{1}.radius = r_link;
joint_appearance{1}.radius = r_joint;
joint_appearance{1}.axis = robot.body_axis{9};
joint_appearance{1}.distance = 0;
joint_appearance{2}.radius = r_joint;
joint_appearance{2}.axis = robot.body_axis{9};
joint_appearance{2}.distance = model_prms.L_lower_leg;
robot.appearance.body{9} = Anim.add_appearance('slender-rod', body_appearance, joint_appearance, false);
% robot.q(9)   = SX.sym('q_k1');
% robot.qd(9)  = SX.sym('dq_k1');
% robot.qdd(9) = SX.sym('ddq_k1');
% robot.q(12) = Var('q_k1', 1);
% robot.qd(12) = Var('dq_k1', 1);
% robot.qdd(12) = Var('ddq_k1', 1);

% qj4
clear body_appearance joint_appearance
robot.parent(10) = 6;    
robot.body_axis{10} = [0 0 0];
robot.body_length{10} = 0;
robot.Xtree{10} = plux(eye(3), model_prms.W_pelvis/2*[0 -1 0]);
robot.jtype{10}  = 'Rx';
robot.I{10} = zeros(6);
robot.appearance.body{10} = Anim.add_appearance('joint', [], [], false);
% robot.q(10)   = SX.sym('q_abad2');
% robot.qd(10)  = SX.sym('dq_abad2');
% robot.qdd(10) = SX.sym('ddq_abad2');
% robot.q(12) = Var('q_k1', 1);
% robot.qd(12) = Var('dq_k1', 1);
% robot.qdd(12) = Var('ddq_k1', 1);

% qj5
clear body_appearance joint_appearance
robot.parent(11) = 10;   
robot.body_axis{11} = [0 0 -1];
robot.body_length{11} = model_prms.L_upper_leg;
robot.Xtree{11} = eye(6);
robot.jtype{11}  = 'Ry';
robot.I{11} = mcI(model_prms.M_upper_leg, model_prms.Lcom_upper_leg*robot.body_axis{11}, diag(1/12*model_prms.M_upper_leg*model_prms.L_upper_leg^2*[-1 -1 0]));
body_appearance{1}.length = model_prms.L_upper_leg;
body_appearance{1}.axis = robot.body_axis{11};
body_appearance{1}.color = model_prms.C_leg2;
body_appearance{1}.radius = r_link;
joint_appearance{1}.radius = r_joint;
joint_appearance{1}.axis = robot.body_axis{11};
joint_appearance{1}.distance = 0;
robot.appearance.body{11} = Anim.add_appearance('slender-rod', body_appearance, joint_appearance, false);
% robot.q(11)   = SX.sym('q_th2');
% robot.qd(11)  = SX.sym('dq_th2');
% robot.qdd(11) = SX.sym('ddq_th2');
% robot.q(12) = Var('q_k1', 1);
% robot.qd(12) = Var('dq_k1', 1);
% robot.qdd(12) = Var('ddq_k1', 1);

% qj6
clear body_appearance joint_appearance
robot.parent(12) = 11;    
robot.body_axis{12} = [0 0 -1];
robot.body_length{12} = model_prms.L_lower_leg;
robot.Xtree{12} = plux(eye(3), model_prms.L_upper_leg*robot.body_axis{12});
robot.jtype{12}  = 'Ry';
robot.I{12} = mcI(model_prms.M_lower_leg, model_prms.Lcom_lower_leg*robot.body_axis{12}, diag(1/12*model_prms.M_lower_leg*model_prms.L_lower_leg^2*[-1 -1 0]));
body_appearance{1}.length = model_prms.L_lower_leg;
body_appearance{1}.axis = robot.body_axis{12};
body_appearance{1}.color = model_prms.C_leg2;
body_appearance{1}.radius = r_link;
joint_appearance{1}.radius = r_joint;
joint_appearance{1}.axis = robot.body_axis{12};
joint_appearance{1}.distance = 0;
joint_appearance{2}.radius = r_joint;
joint_appearance{2}.axis = robot.body_axis{12};
joint_appearance{2}.distance = model_prms.L_lower_leg;
robot.appearance.body{12} = Anim.add_appearance('slender-rod', body_appearance, joint_appearance, false);
% robot.q(12)   = SX.sym('q_k1');
% robot.qd(12)  = SX.sym('dq_k1');
% robot.qdd(12) = SX.sym('ddq_k1');


robot.q = Var('q', 12);
robot.qd = Var('dq', 12);
robot.qdd = Var('ddq', 12);


% floor tiles
robot.appearance.base = {'tiles', [-10 10;-10 10; 0 0], .5};            

% % contact definition: used to derive contact Jacobian(s)
% robot.contact.type = 'point'; % only support point contact so far
% robot.contact.points = [9,12]; % points = [i,...] means distal end of body(i)
% 

% contact definition: used to derive contact Jacobian(s)
%robot.contact(1).type = 'point'; % only support point contact so far (can be point, line, plane)
%robot.contact(1).points = 9; % points = [i,...] means distal end of body(i)
%robot.contact(2).type = 'point';
%robot.contact(2).points = 12; % points = [i,...] means distal end of body(i)


% input torques
robot.u = Var('u', 6);
%robot.u = SX.sym('u',6);
        
% actuation matrix (nd x na)
robot.B = [ zeros(6,6) ; eye(6,6) ];

% selects the actuated DoF
robot.H0 = [zeros(6) , eye(6)];

robot.dimensions = 'spatial';

% define theta (increasing phase variable)
robot.c = [1 zeros(1,11)];%0 0 0 0 -1 -1/2 0 0 0 0 0];

% gravity vector
robot.gravity = [0 0 -9.81]';    
     
%robot.dimensions = '3D';
    
robot.total_mass = model_prms.M_torso + model_prms.M_pelvis + 2*model_prms.M_lower_leg + 2*model_prms.M_upper_leg; 

  
end