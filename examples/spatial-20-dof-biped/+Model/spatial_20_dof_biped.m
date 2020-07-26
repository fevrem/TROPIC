function  robot = spatial_20_dof_biped()

import casadi.*

robot.name = '20 DOF Biped';

% number of DOF
robot.nd = 20;


% geometric and inertial parameters
model_prms = Model.model_params();

% for plotting reasons
r_link = 0.01;
r_joint = 0.015;

%{
    floating base
%}
robot.parent(1) = 0;  
robot.Xtree{1} = eye(6);
robot.jtype{1}  = 'Px';
robot.I{1} = zeros(6);
robot.appearance.body{1} = Anim.add_appearance('base', [], [], false);

robot.parent(2) = 1; 
robot.Xtree{2} = eye(6);
robot.jtype{2}  = 'Py';
robot.I{2} = zeros(6);
robot.appearance.body{2} = Anim.add_appearance('base', [], [], false);

robot.parent(3) = 2;    
robot.Xtree{3} = eye(6);
robot.jtype{3}  = 'Pz';
robot.I{3} = zeros(6);
robot.appearance.body{3} = Anim.add_appearance('base', [], [], false);

robot.parent(4) = 3;   
robot.Xtree{4} = eye(6);
robot.jtype{4}  = 'Rx';
robot.I{4} = zeros(6);
robot.appearance.body{4} = Anim.add_appearance('base', [], [], false);


robot.parent(5) = 4;    
robot.Xtree{5} = eye(6);
robot.jtype{5}  = 'Ry';
robot.I{5} = zeros(6);
robot.appearance.body{5} = Anim.add_appearance('base', [], [], false);


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





% qj1
clear body_appearance joint_appearance
robot.parent(7) = 6;  
% robot.body_axis{7} = [0 0 0];
% robot.body_length{7} = 0;
robot.Xtree{7} = plux(eye(3), model_prms.W_pelvis/2*[0 1 0]);
robot.jtype{7}  = 'Rx';
robot.I{7} = zeros(6);
robot.appearance.body{7} = Anim.add_appearance('joint', [], [], false);


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


% qj3
clear body_appearance joint_appearance
robot.parent(9) = 8;  
robot.body_axis{9} = [0 0 -1];
robot.body_length{9} = model_prms.L_lower_leg;
robot.Xtree{9} = plux(eye(3), model_prms.L_upper_leg*robot.body_axis{8});
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

% qj4
clear body_appearance joint_appearance
robot.parent(10) = 6;    
% robot.body_axis{10} = [0 0 0];
% robot.body_length{10} = 0;
robot.Xtree{10} = plux(eye(3), model_prms.W_pelvis/2*[0 -1 0]);
robot.jtype{10}  = 'Rx';
robot.I{10} = zeros(6);
robot.appearance.body{10} = Anim.add_appearance('joint', [], [], false);


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


% qj6
clear body_appearance joint_appearance
robot.parent(12) = 11;    
robot.body_axis{12} = [0 0 -1];
robot.body_length{12} = model_prms.L_lower_leg;
robot.Xtree{12} = plux(eye(3), model_prms.L_upper_leg*robot.body_axis{11});
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



% add yaw rotation at top of torso for shoulders
% shoulders
clear body_appearance joint_appearance
robot.parent(13) = 6;    
robot.Xtree{13} = plux(eye(3), model_prms.L_torso*[0 0 1]);
robot.jtype{13}  = 'Rz';
robot.I{13} = mcI(model_prms.M_shoulder/2, model_prms.Wcom_shoulder*[0 1 0], diag(1/12*(model_prms.M_shoulder/2)*(model_prms.W_shoulder/2)^2*[1 0 1]));
robot.I{13} = robot.I{13} + mcI(model_prms.M_shoulder/2, model_prms.Wcom_shoulder*[0 -1 0], diag(1/12*(model_prms.M_shoulder/2)*(model_prms.W_shoulder/2)^2*[-1 0 -1]));
body_appearance{1}.length = model_prms.W_shoulder/2;
body_appearance{1}.axis = [0 1 0];
body_appearance{1}.color = [1 0 1];
body_appearance{1}.radius = r_link;
body_appearance{2}.length = model_prms.W_shoulder/2;
body_appearance{2}.axis = [0 -1 0];
body_appearance{2}.color = [1 0 1];
body_appearance{2}.radius = r_link;
robot.appearance.body{13} = Anim.add_appearance('slender-rod', body_appearance, [], false);



% head
clear body_appearance joint_appearance
robot.parent(14) = 13;    
robot.Xtree{14} = eye(6);
robot.jtype{14}  = 'Ry';
robot.I{14} = mcI(model_prms.M_head, model_prms.Lcom_head*[0 0 1], diag(1/12*model_prms.M_head*model_prms.L_head^2*[1 1 0]));
body_appearance{1}.length = model_prms.L_head;
body_appearance{1}.axis = [0 0 1];
body_appearance{1}.color = model_prms.C_head;
body_appearance{1}.radius = r_link;
robot.appearance.body{14} = Anim.add_appearance('slender-rod', body_appearance, [], false);





% shoulder 1 in/out
clear body_appearance joint_appearance
robot.parent(15) = 13;  
robot.Xtree{15} = plux(eye(3), model_prms.W_shoulder/2*[0 1 0]);
robot.jtype{15}  = 'Rx';
robot.I{15} = zeros(6);
robot.appearance.body{15} = Anim.add_appearance('joint', [], [], false);

% shoulder 1 back/forth
clear body_appearance joint_appearance
robot.parent(16) = 15;  
robot.Xtree{16} = eye(6);
robot.jtype{16}  = 'Ry';
robot.I{16} = mcI(model_prms.M_upper_arm, model_prms.Lcom_upper_arm*[0 0 -1], diag(1/12*model_prms.M_upper_arm*model_prms.L_upper_arm^2*[-1 -1 0]));
body_appearance{1}.length = model_prms.L_upper_arm;
body_appearance{1}.axis = [0 0 -1];
body_appearance{1}.color = model_prms.C_arm1;
body_appearance{1}.radius = r_link;
robot.appearance.body{16} = Anim.add_appearance('slender-rod', body_appearance, [], false);

% elbow 1
clear body_appearance 
robot.parent(17) = 16;    
robot.body_axis{17} = [0 0 -1];
robot.body_length{17} = model_prms.L_lower_arm;
robot.Xtree{17} = plux(eye(3), model_prms.L_upper_arm*[0 0 -1]);
robot.jtype{17}  = 'Ry';
robot.I{17} = mcI(model_prms.M_lower_arm, model_prms.Lcom_lower_arm*[0 0 -1], diag(1/12*model_prms.M_lower_arm*model_prms.L_lower_arm^2*[-1 -1 0]));
body_appearance{1}.length = model_prms.L_lower_arm;
body_appearance{1}.axis = robot.body_axis{17};
body_appearance{1}.color = model_prms.C_arm1;
body_appearance{1}.radius = r_link;
robot.appearance.body{17} = Anim.add_appearance('slender-rod', body_appearance, [], false);












% shoulder 2 in/out
clear body_appearance joint_appearance
robot.parent(18) = 13;  
robot.Xtree{18} = plux(eye(3), model_prms.W_shoulder/2*[0 -1 0]);
robot.jtype{18}  = 'Rx';
robot.I{18} = zeros(6);
robot.appearance.body{18} = Anim.add_appearance('joint', [], [], false);

% shoulder 2 back/forth
clear body_appearance joint_appearance
robot.parent(19) = 18;  
robot.Xtree{19} = eye(6);
robot.jtype{19}  = 'Ry';
robot.I{19} = mcI(model_prms.M_upper_arm, model_prms.Lcom_upper_arm*[0 0 -1], diag(1/12*model_prms.M_upper_arm*model_prms.L_upper_arm^2*[-1 -1 0]));
body_appearance{1}.length = model_prms.L_upper_arm;
body_appearance{1}.axis = [0 0 -1];
body_appearance{1}.color = model_prms.C_arm2;
body_appearance{1}.radius = r_link;
robot.appearance.body{19} = Anim.add_appearance('slender-rod', body_appearance, [], false);

% elbow 2
clear body_appearance 
robot.parent(20) = 19;    
robot.body_axis{20} = [0 0 -1];
robot.body_length{20} = model_prms.L_lower_arm;
robot.Xtree{20} = plux(eye(3), model_prms.L_upper_arm*[0 0 -1]);
robot.jtype{20}  = 'Ry';
robot.I{20} = mcI(model_prms.M_lower_arm, model_prms.Lcom_lower_arm*[0 0 -1], diag(1/12*model_prms.M_lower_arm*model_prms.L_lower_arm^2*[-1 -1 0]));
body_appearance{1}.length = model_prms.L_lower_arm;
body_appearance{1}.axis = [0 0 -1];
body_appearance{1}.color = model_prms.C_arm2;
body_appearance{1}.radius = r_link;
robot.appearance.body{20} = Anim.add_appearance('slender-rod', body_appearance, [], false);














robot.q = Var('q', robot.nd);
robot.qd = Var('dq', robot.nd);
robot.qdd = Var('ddq', robot.nd);


% floor tiles
robot.appearance.base = {'tiles', [-10 10;-10 10; 0 0], .5};            


% input torques
robot.u = Var('u', 14);
        
% actuation matrix (nd x na)
robot.B = [ zeros(6,14) ; eye(14) ];

% selects the actuated DoF
robot.H0 = robot.B';

robot.dimensions = 'spatial';

% define theta (increasing phase variable)
robot.c = [1 zeros(1,19)];

% gravity vector
robot.gravity = [0 0 -9.81]';    
     
robot.total_mass = model_prms.M_lower_leg*2 + model_prms.M_upper_leg*2 + model_prms.M_pelvis + model_prms.M_torso + model_prms.M_shoulder + model_prms.M_head + model_prms.M_upper_arm*2 + model_prms.M_lower_arm*2;










end