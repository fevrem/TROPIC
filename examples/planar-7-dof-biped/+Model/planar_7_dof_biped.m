function  robot = planar_7_dof_biped()



import casadi.*

robot.name = '7 DOF 5-Link Planar Biped';


% number of DoF
robot.nd = 7;
    

% ----------- Geometric and inertial parameters ------------- %
model_prms = Model.model_params();


% for plotting reasons
r_link = 0.01;
r_joint = 0.015;




%{
    floating base
%}
robot.parent(1) = 0;  
%robot.body_axis{1} = [0 0 0];
%robot.body_length{1} = 0;
robot.Xtree{1} = eye(6);
robot.jtype{1}  = 'Px';
robot.I{1} = zeros(6);
robot.appearance.body{1} = Anim.add_appearance('base', [], [], false);


robot.parent(2) = 1; 
%robot.body_axis{2} = [0 0 0];
%robot.body_length{2} = 0;
robot.Xtree{2} = eye(6);
robot.jtype{2}  = 'Pz';
robot.I{2} = zeros(6);
robot.appearance.body{2} = Anim.add_appearance('base', [], [], false);


% torso
robot.parent(3) = 2;    
robot.body_axis{3} = [0 0 1];
robot.body_length{3} = model_prms.L_torso;
robot.Xtree{3} = eye(6);
robot.jtype{3}  = 'Ry';
robot.I{3} = mcI(model_prms.M_torso, model_prms.Lcom_torso*robot.body_axis{3}, diag(1/12*model_prms.M_torso*model_prms.L_torso^2*[1 1 0])); % torso
body_appearance{1}.length = model_prms.L_torso;
body_appearance{1}.axis = robot.body_axis{3};
body_appearance{1}.color = model_prms.C_torso;
body_appearance{1}.radius = r_link;
% joint_appearance{1}.radius = r_joint;
% joint_appearance{1}.axis = robot.body_axis{6};
% joint_appearance{1}.distance = 0;
% joint_appearance{2}.radius = r_joint;
% joint_appearance{2}.axis = robot.body_axis{6};
% joint_appearance{2}.distance = model_prms.L_torso;

% robot.I{6} = robot.I{6} + mcI(model_prms.M_pelvis/2, model_prms.Wcom_pelvis/2*[0 1 0], diag(1/12*model_prms.M_pelvis*(model_prms.W_pelvis/2)^2*[1 0 1])); % half pelvis (1)
% body_appearance{2}.length = model_prms.W_pelvis/2;
% body_appearance{2}.axis = [0 1 0];
% body_appearance{2}.color = model_prms.C_torso;
% body_appearance{2}.radius = r_link;
% robot.I{6} = robot.I{6} + mcI(model_prms.M_pelvis/2, model_prms.Wcom_pelvis/2*[0 -1 0], diag(1/12*model_prms.M_pelvis*(model_prms.W_pelvis/2)^2*[-1 0 -1])); % half pelvis (2)
% body_appearance{3}.length = model_prms.W_pelvis/2;
% body_appearance{3}.axis = [0 -1 0];
% body_appearance{3}.color = model_prms.C_torso;
% body_appearance{3}.radius = r_link;
robot.appearance.body{3} = Anim.add_appearance('slender-rod', body_appearance, [], false);




% qj2
clear body_appearance 
robot.parent(4) = 3;    
robot.body_axis{4} = [0 0 -1];
robot.body_length{4} = model_prms.L_upper_leg;
robot.Xtree{4} = eye(6);
robot.jtype{4}  = 'Ry';
robot.I{4} = mcI(model_prms.M_upper_leg, model_prms.Lcom_upper_leg*robot.body_axis{4}, diag(1/12*model_prms.M_upper_leg*model_prms.L_upper_leg^2*[-1 -1 0]));
body_appearance{1}.length = model_prms.L_upper_leg;
body_appearance{1}.axis = robot.body_axis{4};
body_appearance{1}.color = model_prms.C_leg1;
body_appearance{1}.radius = r_link;
% joint_appearance{1}.radius = r_joint;
% joint_appearance{1}.axis = robot.body_axis{8};
% joint_appearance{1}.distance = 0;
robot.appearance.body{4} = Anim.add_appearance('slender-rod', body_appearance, [], false);



% qj3
clear body_appearance joint_appearance
robot.parent(5) = 4;  
robot.body_axis{5} = [0 0 -1];
robot.body_length{5} = model_prms.L_lower_leg;
robot.Xtree{5} = plux(eye(3), model_prms.L_upper_leg*robot.body_axis{5});
robot.jtype{5}  = 'Ry';
robot.I{5} = mcI(model_prms.M_lower_leg, model_prms.Lcom_lower_leg*robot.body_axis{5}, diag(1/12*model_prms.M_lower_leg*model_prms.L_lower_leg^2*[-1 -1 0]));
body_appearance{1}.length = model_prms.L_lower_leg;
body_appearance{1}.axis = robot.body_axis{5};
body_appearance{1}.color = model_prms.C_leg1;
body_appearance{1}.radius = r_link;
% joint_appearance{1}.radius = r_joint;
% joint_appearance{1}.axis = robot.body_axis{9};
% joint_appearance{1}.distance = 0;
% joint_appearance{2}.radius = r_joint;
% joint_appearance{2}.axis = robot.body_axis{9};
% joint_appearance{2}.distance = model_prms.L_lower_leg;
robot.appearance.body{5} = Anim.add_appearance('slender-rod', body_appearance, [], false);



% qj2
clear body_appearance 
robot.parent(6) = 3;    
robot.body_axis{6} = [0 0 -1];
robot.body_length{6} = model_prms.L_upper_leg;
robot.Xtree{6} = eye(6);
robot.jtype{6}  = 'Ry';
robot.I{6} = mcI(model_prms.M_upper_leg, model_prms.Lcom_upper_leg*robot.body_axis{6}, diag(1/12*model_prms.M_upper_leg*model_prms.L_upper_leg^2*[-1 -1 0]));
body_appearance{1}.length = model_prms.L_upper_leg;
body_appearance{1}.axis = robot.body_axis{6};
body_appearance{1}.color = model_prms.C_leg2;
body_appearance{1}.radius = r_link;
% joint_appearance{1}.radius = r_joint;
% joint_appearance{1}.axis = robot.body_axis{8};
% joint_appearance{1}.distance = 0;
robot.appearance.body{6} = Anim.add_appearance('slender-rod', body_appearance, [], false);



% qj3
clear body_appearance joint_appearance
robot.parent(7) = 6;  
robot.body_axis{7} = [0 0 -1];
robot.body_length{7} = model_prms.L_lower_leg;
robot.Xtree{7} = plux(eye(3), model_prms.L_upper_leg*robot.body_axis{7});
robot.jtype{7}  = 'Ry';
robot.I{7} = mcI(model_prms.M_lower_leg, model_prms.Lcom_lower_leg*robot.body_axis{7}, diag(1/12*model_prms.M_lower_leg*model_prms.L_lower_leg^2*[-1 -1 0]));
body_appearance{1}.length = model_prms.L_lower_leg;
body_appearance{1}.axis = robot.body_axis{7};
body_appearance{1}.color = model_prms.C_leg2;
body_appearance{1}.radius = r_link;
% joint_appearance{1}.radius = r_joint;
% joint_appearance{1}.axis = robot.body_axis{9};
% joint_appearance{1}.distance = 0;
% joint_appearance{2}.radius = r_joint;
% joint_appearance{2}.axis = robot.body_axis{9};
% joint_appearance{2}.distance = model_prms.L_lower_leg;
robot.appearance.body{7} = Anim.add_appearance('slender-rod', body_appearance, [], false);






    
robot.q = Var('q', 7);
robot.qd = Var('dq', 7);
robot.qdd = Var('ddq', 7);



% floor tiles
robot.appearance.base = {'tiles', [-10 10;-10 10; 0 0], .5};   



% input torques
robot.u = Var('u', 4);
        

% actuation matrix (nd x na)
robot.B = [zeros(3,4); eye(4)];

% selects the actuated DoF
robot.H0 = robot.B';

robot.dimensions = 'planar';

% define theta (increasing phase variable)
robot.c = [1 zeros(1,6)];%0 0 0 0 -1 -1/2 0 0 0 0 0];

% gravity vector
robot.gravity = [0 0 -9.81]';    
     

robot.total_mass = model_prms.M_torso + 2*model_prms.M_upper_leg + 2*model_prms.M_lower_leg;
robot.leg_length = model_prms.L_lower_leg + model_prms.L_upper_leg ;

 




 
   

                   
    
end