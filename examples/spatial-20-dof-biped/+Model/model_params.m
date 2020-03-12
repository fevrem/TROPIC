function model_prms = model_params()

L_lower_leg = 0.4;  % m
L_upper_leg = 0.4;  % m 
W_pelvis    = 0.20;  % m
L_torso     = 0.45;  % m
W_shoulder  = 0.35;  % m
L_upper_arm = 0.25;  % m
L_lower_arm = 0.3;  % m
L_head = 0.1; % m

model_prms.L_lower_leg = L_lower_leg;
model_prms.L_upper_leg = L_upper_leg;
model_prms.W_pelvis = W_pelvis;
model_prms.L_torso = L_torso;
model_prms.W_shoulder = W_shoulder;
model_prms.L_upper_arm = L_upper_arm;
model_prms.L_lower_arm = L_lower_arm;
model_prms.L_head = L_head;


% distance from proximal joint
Lcom_lower_leg = L_lower_leg*0.2;  % m
Lcom_upper_leg = L_upper_leg*0.2;  % m 
Lcom_torso     = L_torso*0.25;  % m
Wcom_pelvis    = W_pelvis*0.1; % m
Wcom_shoulder = W_shoulder*0.2;
Lcom_upper_arm = L_upper_arm*0.3;
Lcom_lower_arm = L_lower_arm*0.4;
Lcom_head = L_head*0.5;

model_prms.Lcom_lower_leg = Lcom_lower_leg;
model_prms.Lcom_upper_leg = Lcom_upper_leg;
model_prms.Lcom_torso = Lcom_torso;
model_prms.Wcom_pelvis = Wcom_pelvis;
model_prms.Wcom_shoulder = Wcom_shoulder;
model_prms.Lcom_upper_arm = Lcom_upper_arm;
model_prms.Lcom_lower_arm = Lcom_lower_arm;
model_prms.Lcom_head = Lcom_head;


M_lower_leg = 2.5;  % kg
M_upper_leg = 3.5;  % kg
M_pelvis    = 2;  % kg
M_torso     = 15;  % kg
M_shoulder = 1;
M_upper_arm = 2.5;
M_lower_arm = 2.5;
M_head = 2;

model_prms.M_lower_leg = M_lower_leg;
model_prms.M_upper_leg = M_upper_leg;
model_prms.M_pelvis = M_pelvis;
model_prms.M_torso = M_torso;
model_prms.M_shoulder = M_shoulder;
model_prms.M_upper_arm = M_upper_arm;
model_prms.M_lower_arm = M_lower_arm;
model_prms.M_head = M_head;


model_prms.C_leg1 = [1 0 0];
model_prms.C_leg2 = [0 0 1];
model_prms.C_torso = [0 0.7 0];
model_prms.C_arm1 = [1 0 0];
model_prms.C_arm2 = [0 0 1];
model_prms.C_head = [211 117 64]/255; 




end