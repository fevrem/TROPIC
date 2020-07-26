%% LOAD DYNAMICAL SYSTEM

clear all; clc; close all; %#ok<*CLALL>

[rbm] = ld_model(...
    {'model', @Model.planar_7_dof_biped},...
    {'debug', false});


%% SPECIFY CONTACT

% Right leg end
rbm.Contacts{1} = Contact(rbm, 'Point',...
    {'Friction', true},...
    {'FrictionCoefficient', 0.6},...
    {'FrictionType', 'Pyramid'},...
    {'ContactFrame', rbm.BodyPositions{5,2}([1,3])});


%% CREATE NLP

nlp = NLP(rbm,...
    {'NFE', 25},...
    {'CollocationScheme', 'HermiteSimpson'},...
    {'LinearSolver', 'mumps'},...
    {'ConstraintTolerance', 1E-4});

% Create functions for dynamics equations 
nlp = ConfigFunctions(nlp, rbm);


%% Virtual constraint
nlp = AddVirtualConstraints(nlp, rbm,...
    {'PolyType', 'Bezier'},...
    {'PolyOrder', 5},...
    {'PolyPhase', 'state-based'});


% load user-defined constraints
[nlp, rbm] = LoadConstraints(nlp, rbm);


%% LOAD SEED

%[nlp, rbm] = LoadSeed(nlp, rbm);
[nlp, rbm] = LoadSeed(nlp, rbm, 'planar-7-dof-seed.mat');
 

%% FILL UP & SOLVE NLP

nlp = ParseNLP(nlp, rbm);

nlp = SolveNLP(nlp);


%% EXTRACT SOLUTION

% structure containing the solution data
data = ExtractData(nlp, rbm);


%% ANIMATE SOLUTION

%qAnim = data.pos;
%tAnim = data.t;
tAnim = seed.t;
qAnim = seed.q;

% options: true or false 
anim_options.bool = true;

anim_options.axis.x = [-0.1 0.6];
anim_options.axis.y = [-0.1 0.4];
anim_options.axis.z = [0 1.3];

% skips frame to animate faster 
anim_options.skip_frame = 1;

anim_options.views = {'3D','sagittal'};

% save options
anim_options.save_movie    = false;
anim_options.movie_name    = 'planar_7dof_biped.mp4';
anim_options.movie_quality = 100; % scalar between [0 100], default 75
anim_options.movie_fps     = 30;  % frame rate, default 30

% create a light object or not
anim_options.lights = true;

% can pass figure as 5th argument
Anim.animate(rbm, tAnim, qAnim, anim_options)
%Anim.animate(rbm, seed.t, seed.q, anim_options)
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
str2save = 'planar-7-dof-seed.mat';
save(str2save, 'seed')

