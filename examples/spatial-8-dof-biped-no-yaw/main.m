%% LOAD DYNAMICAL SYSTEM

clear all; clc; close all; %#ok<*CLALL>

[rbm] = ld_model(...
    {'model',@Model.spatial_8_dof_biped_no_yaw},...
    {'debug',false});


%% SPECIFY CONTACT

% Right leg end
rbm.Contacts{1} = Contact(rbm, 'Point',...
    {'Friction', true},...
    {'FrictionCoefficient', 0.6},...
    {'FrictionType', 'Pyramid'},...
    {'ContactFrame', rbm.BodyPositions{3,1}});


%% CREATE NLP

nlp = NLP(rbm,...
    {'NFE', 50},...
    {'CollocationScheme', 'HermiteSimpson'},... 
    {'LinearSolver', 'mumps'},... 
    {'ConstraintTolerance', 1E-4});

% Create functions for dynamics equations 
nlp = ConfigFunctions(nlp, rbm);


%% VIRTUAL CONSTRAINT

nlp = AddVirtualConstraints(nlp, rbm,...
    {'PolyType', 'Bezier'},...
    {'PolyOrder', 5},...
    {'PolyPhase', 'time-based'});

% load user-defined constraints
[nlp, rbm] = LoadConstraints(nlp, rbm);


%% INITIALIZE NLP

[nlp, rbm] = LoadSeed(nlp, rbm);
%[nlp, rbm] = LoadSeed(nlp, rbm, 'spatial-12-dof-seed.mat');
 

%% PARSE & SOLVE THE NLP

nlp = ParseNLP(nlp, rbm);

nlp = SolveNLP(nlp);


%% EXTRACT SOLUTION

data = ExtractData(nlp, rbm);


%% ANIMATE SOLUTION

qAnim = data.pos;
tAnim = data.t;

% options: true or false 
anim_options.bool = true;

anim_options.axis.x = [-0.1 0.6];
anim_options.axis.y = [-0.1 0.4];
anim_options.axis.z = [0 1.3];

% skips frame to animate faster 
anim_options.skip_frame = 1;

% views to show. Options: {'3D','frontal','sagittal','top'}
anim_options.views = {'3D','frontal','sagittal','top'};
%anim_options.views = {'3D'};

% save options
anim_options.save_movie    = false;
anim_options.movie_name    = 'spatial_12_dof.mp4';
anim_options.movie_quality = 100; % scalar between [0 100], default 75
anim_options.movie_fps     = 30;  % frame rate, default 30

% create a light object or not
anim_options.lights = true;

% can pass figure as 5th argument
Anim.animate(rbm, tAnim, qAnim, anim_options)
%set(gcf,'menubar','figure')
%set(gcf,'menubar','none')


%% SAVE SEED

seed.q = data.pos;
seed.qd = data.vel;
seed.qdd = data.acc;
seed.t = data.t;
seed.Fc_1 = data.Fc1;
seed.a = data.a;
seed.u = data.input;
%seed.du = data.der_input;

% can be used as seed
str2save = 'spatial-12-dof-seed.mat';
save(str2save, 'seed')


%% SAVE GAIT

% can be used as seed
str2save = ['spatial-8-dof-no-yaw-', num2str(data.walking_speed), 'mps.mat'];
save(str2save, 'data')



%% 

close all
clc

%{
    DEBUG
%}

% qAnim = [0.1; 0.05; 0;
%     pi/8; pi/6;
%     -pi/6; 0; -pi/8;
%     0; 0; 0]

qAnim = [0.1; 0.05; 0.1;
    -pi/12; pi/11;
    -pi/10; pi/9; pi/8;
    -pi/7; -pi/6; pi/5];

tAnim = 0;

% options: true or false 
anim_options.bool = true;

% anim_options.axis.x = [-0.2 0.6];
% anim_options.axis.y = [-0.2 0.4];
% anim_options.axis.z = [0 1.3];
anim_options.axis.x = [-0.4 0.4];
anim_options.axis.y = [-0.4 0.4];
anim_options.axis.z = [0 1];


% skips frame to animate faster 
anim_options.skip_frame = 1;

% views to show. Options: {'3D','frontal','sagittal','top'}
anim_options.views = {'frontal','sagittal','top'};
%anim_options.views = {'3D'};

% save options
anim_options.save_movie    = false;
anim_options.movie_name    = 'spatial_12_dof.mp4';
anim_options.movie_quality = 100; % scalar between [0 100], default 75
anim_options.movie_fps     = 30;  % frame rate, default 30

% create a light object or not
anim_options.lights = false;

% can pass figure as 5th argument
Anim.animate(rbm, tAnim, qAnim, anim_options)
set(gcf,'menubar','figure')
%set(gcf,'menubar','none')








% R = [1,0,0,0,0,0,0,0,0,0,0;
%      0,-1,0,0,0,0,0,0,0,0,0;
%      0,0,1,0,0,0,0,0,0,0,0;
%      0,0,0,-1,0,0,0,0,0,0,0;
%      0,0,0,0,1,0,0,0,0,0,0;
%      0,0,0,0,0,0,0,0,0,0,-1;
%      0,0,0,0,0,0,0,0,0,-1,0;
%      0,0,0,0,0,0,0,0,1,0,0;
%      0,0,0,0,0,0,0,1,0,0,0;
%      0,0,0,0,0,0,-1,0,0,0,0;
%      0,0,0,0,0,-1,0,0,0,0,0];
%  
%  
% qAnim = R*qAnim;
%  
% %qAnim(7) = -qAnim(7);
% %qAnim(8) = qAnim(8);
% 
% qAnim(1) = p_stance_new(1);
% qAnim(2) = p_stance_new(2); % use this for easy for animation (wrong)
% %qAnim(2) = -p_stance_new(2); % correct
% qAnim(3) = p_stance_new(3);

%{
    do not solve IK (discontinuity)
    instead, enforce position of knee
%}

bodpos_ex(rbm, qAnim)

q_old = qAnim;

import casadi.*

p111fun = Function('f', {rbm.States.q.sym}, {rbm.BodyPositions{11,1}});
p_swing_knee = full(p111fun(qAnim));

p112fun = Function('f', {rbm.States.q.sym}, {rbm.BodyPositions{11,2}});
p_swing_foot = full(p112fun(qAnim));


% rotation matrix for swing leg
R_0_to_11 = rbm.HTransforms{11}(1:3,1:3);

% quantities needed to solve IK
r11 = R_0_to_11(1,1);
r12 = R_0_to_11(1,2);
r13 = R_0_to_11(1,3);

r21 = R_0_to_11(2,1);
r22 = R_0_to_11(2,2);
r23 = R_0_to_11(2,3);

r31 = R_0_to_11(3,1);
r32 = R_0_to_11(3,2);
r33 = R_0_to_11(3,3);



rbm.HTransforms{5}(1:3,1:3)*(rbm.HTransforms{3}(1:3,1:3))'


if 0
    r23fun = Function('f', {rbm.States.q.sym}, {r23});
    full(r23fun(qAnim))

    r22fun = Function('f', {rbm.States.q.sym}, {r22});
    full(r22fun(qAnim))

    r31fun = Function('f', {rbm.States.q.sym}, {r31});
    full(r31fun(qAnim))

    r11fun = Function('f', {rbm.States.q.sym}, {r11});
    full(r11fun(qAnim))

    q4_new = atan2(full(r23fun(qAnim)), full(r22fun(qAnim)))
    num2str(q4_new,8)
    q5_new = atan2(full(r31fun(qAnim)), full(r11fun(qAnim)))
    num2str(q5_new,8)
end



% solve IK
q4_new_var = atan2(r23, r22);
q5_new_var = atan2(r31, r11);

% sol2
%q4_new_var2 = atan2(-r32/r11, -r13/r31);
%q5_new_var2 = atan2(r12/r23, r33/r22);

% sol3
%q4_new_var = atan2(-r23, r22);
%q5_new_var = atan2(-r31, r11);


%qAnim(4:5)

q4_new_fun = Function('f', {rbm.States.q.sym}, {q4_new_var});
%q4_new_fun2 = Function('f', {rbm.States.q.sym}, {q4_new_var2});
q4_new = full(q4_new_fun(qAnim))
%q4_new = full(q4_new_fun2(qAnim))
%q4_new = q4_new2;
%num2str(q4_new,8)

q5_new_fun = Function('f', {rbm.States.q.sym}, {q5_new_var});
%q5_new_fun2 = Function('f', {rbm.States.q.sym}, {q5_new_var2});
q5_new = full(q5_new_fun(qAnim))
%q5_new = full(q5_new_fun2(qAnim))
%num2str(q5_new,8)


% relabel
if 1
    qAnim(1:3) = p_swing_foot; % xyz
    qAnim(4) = q4_new; % roll (rx)
    qAnim(5) = q5_new; % pitch (ry)

    qAnim(6) = -q_old(11); % stance knee
    qAnim(7) = -q_old(10)+pi; % stance knee
    qAnim(8) = q_old(9)+pi; % stance knee
    qAnim(9) = q_old(8)-pi; % stance knee
    qAnim(10) = -q_old(7)+pi; % stance knee
    qAnim(11) = -q_old(6); % stance knee

    Anim.animate(rbm, tAnim, qAnim, anim_options)
end

bodpos_ex(rbm, qAnim)




% relabel
if 1
    %qAnim(1:3) = p_swing_foot; % xyz
    qAnim(1) = p_swing_foot(1); % xyz
    qAnim(2) = -p_swing_foot(2); % xyz
    qAnim(3) = p_swing_foot(3); % xyz
    qAnim(4) = -q4_new; % roll (rx)
    qAnim(5) = q5_new; % pitch (ry)

    qAnim(6) = -q_old(11); % stance knee (ry)
    qAnim(7) = -q_old(10); % stance hip (ry)
    qAnim(8) = q_old(9); % stance ab/ad (rx)
    qAnim(9) = q_old(8); % stance knee
    qAnim(10) = -q_old(7); % stance knee
    qAnim(11) = -q_old(6); % stance knee

    Anim.animate(rbm, tAnim, qAnim, anim_options)
end

bodpos_ex(rbm, qAnim)



%%

% flip (different than relabel)
qAnim(1) = p_swing_foot(1); % x
qAnim(2) = -p_swing_foot(2); % y
qAnim(3) = p_swing_foot(3); % z
qAnim(4) = -q4_new+pi; % roll (rx)
qAnim(5) = -q5_new+pi; % pitch (ry)
qAnim(6) = -q_old(11); % stance knee (ry)

qAnim(7) = -q_old(10); % stance hip (ry)
qAnim(8) = q_old(9); % stance ab/ad (rx)
    
qAnim(9) = q_old(8); % swing ab/ad (rx)

qAnim(10) = -q_old(7); % swing hip (ry)
qAnim(11) = -q_old(6); % swing knee (ry)

bodpos_ex(rbm, qAnim)

Anim.animate(rbm, tAnim, qAnim, anim_options)
set(gcf,'menubar','figure')

%%

function bodpos_ex(rbm, qAnim)

    import casadi.*

    p11fun = Function('f', {rbm.States.q.sym}, {rbm.BodyPositions{1,1}});
    p12fun = Function('f', {rbm.States.q.sym}, {rbm.BodyPositions{1,2}});                
    p1 = [full(p11fun(qAnim)), full(p12fun(qAnim))']

    p21fun = Function('f', {rbm.States.q.sym}, {rbm.BodyPositions{2,1}});
    p22fun = Function('f', {rbm.States.q.sym}, {rbm.BodyPositions{2,2}});
    p2 = [full(p21fun(qAnim)), full(p22fun(qAnim))']

    p31fun = Function('f', {rbm.States.q.sym}, {rbm.BodyPositions{3,1}});
    p32fun = Function('f', {rbm.States.q.sym}, {rbm.BodyPositions{3,2}});
    p3 = [full(p31fun(qAnim)), full(p32fun(qAnim))']

    p41fun = Function('f', {rbm.States.q.sym}, {rbm.BodyPositions{4,1}});
    p42fun = Function('f', {rbm.States.q.sym}, {rbm.BodyPositions{4,2}});
    p4 = [full(p41fun(qAnim)), full(p42fun(qAnim))']

    p51fun = Function('f', {rbm.States.q.sym}, {rbm.BodyPositions{5,1}});
    p52fun = Function('f', {rbm.States.q.sym}, {rbm.BodyPositions{5,2}});
    p5 = [full(p51fun(qAnim)), full(p52fun(qAnim))]

    p61fun = Function('f', {rbm.States.q.sym}, {rbm.BodyPositions{6,1}});
    p62fun = Function('f', {rbm.States.q.sym}, {rbm.BodyPositions{6,2}});
    p6 = [full(p61fun(qAnim)), full(p62fun(qAnim))]

    p71fun = Function('f', {rbm.States.q.sym}, {rbm.BodyPositions{7,1}});
    p72fun = Function('f', {rbm.States.q.sym}, {rbm.BodyPositions{7,2}});
    p7 = [full(p71fun(qAnim)), full(p72fun(qAnim))']

    p81fun = Function('f', {rbm.States.q.sym}, {rbm.BodyPositions{8,1}});
    p82fun = Function('f', {rbm.States.q.sym}, {rbm.BodyPositions{8,2}});
    p8 = [full(p81fun(qAnim)), full(p82fun(qAnim))]

    p91fun = Function('f', {rbm.States.q.sym}, {rbm.BodyPositions{9,1}});
    p92fun = Function('f', {rbm.States.q.sym}, {rbm.BodyPositions{9,2}});
    p9 = [full(p91fun(qAnim)), full(p92fun(qAnim))']

    p101fun = Function('f', {rbm.States.q.sym}, {rbm.BodyPositions{10,1}});
    p102fun = Function('f', {rbm.States.q.sym}, {rbm.BodyPositions{10,2}});
    p10 = [full(p101fun(qAnim)), full(p102fun(qAnim))]

    p111fun = Function('f', {rbm.States.q.sym}, {rbm.BodyPositions{11,1}});
    p112fun = Function('f', {rbm.States.q.sym}, {rbm.BodyPositions{11,2}});
    p11 = [full(p111fun(qAnim)), full(p112fun(qAnim))]

    X = Function('f', {rbm.States.q.sym}, {rbm.Dynamics.p_com});
    pcom = full(X(qAnim))
    
end
