function model_prms = model_params()

L_lower_leg = 0.4;  % m
L_upper_leg = 0.4;  % m 
W_pelvis    = 0.20;  % m
L_torso     = 0.25;  % m

% distance from proximal joint
Lcom_lower_leg = L_lower_leg*0.5;  % m
Lcom_upper_leg = L_upper_leg*0.5;  % m 
Lcom_torso     = L_torso*0.5;  % m
Wcom_pelvis    = W_pelvis*0.5; % m

M_lower_leg = 1;  % kg
M_upper_leg = 1;  % kg
M_pelvis    = 1;  % kg
M_torso     = 5;  % kg

model_prms.L_lower_leg = L_lower_leg;
model_prms.L_upper_leg = L_upper_leg;
model_prms.W_pelvis = W_pelvis;
model_prms.L_torso = L_torso;

model_prms.Lcom_lower_leg = Lcom_lower_leg;
model_prms.Lcom_upper_leg = Lcom_upper_leg;
model_prms.Lcom_torso = Lcom_torso;
model_prms.Wcom_pelvis = Wcom_pelvis;

model_prms.M_lower_leg = M_lower_leg;
model_prms.M_upper_leg = M_upper_leg;
model_prms.M_pelvis = M_pelvis;
model_prms.M_torso = M_torso;

model_prms.C_leg1 = [1 0 0];
model_prms.C_leg2 = [0 0 1];
model_prms.C_torso = [0 0.7 0];%[211 117 64]/255;
 
end