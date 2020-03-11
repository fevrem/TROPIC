
clear all; clc; close all;

rmpath(genpath(pwd))
TOPIC_add_path


cd('examples/spatial-12-dof-biped')

% LOAD DYNAMICAL SYSTEM

%clear all; clc; close all;

[rbm] = ld_model(...
    {'model',@Model.spatial_12_dof_biped},...
    {'debug',false});
 

% SPECIFY CONTACT

% Right leg end
rbm.Contacts{1} = Contact(rbm, 'Point',...
    {'Friction', true},...
    {'FrictionCoefficient', 0.6},...
    {'FrictionType', 'Pyramid'},...
    {'ContactFrame', rbm.BodyPositions{12,2}})


% Left leg end
% rbm.Contacts{2} = Contact(rbm, 'Point',...
%     {'Friction', true},...
%     {'FrictionCoefficient', 0.6},...
%     {'FrictionType', 'Cone'},...
%     {'ContactFrame', rbm.BodyPositions{9,2}});


%
% CREATE EMPTY NLP

%{
- nfe: 
    number of finite elements
- linear_solver:
    options: 'ma57', 'ma27', 'mumps'
- quadrature:
    rule of integration for collocation scheme
    options: 'trapezoidal'
- constraint_tol:
    options: double
%}

nlp = NLP(rbm,...
    {'NFE', 25},...
    {'CollocationScheme', 'HermiteSimpson'},... %HermiteSimpson,Trapezoidal
    {'LinearSolver', 'mumps'},... %'ma57'},..
    {'ConstraintTolerance', 1E-4});


% Create functions for dynamics equations 
nlp = ConfigFunctions(nlp, rbm);


% Virtual constraint
nlp = AddVirtualConstraints(nlp, rbm,...
    {'PolyType', 'Bezier'},...
    {'PolyOrder', 5},...
    {'PolyPhase', 'time-based'});



% load user-defined constraints
[nlp, rbm] = LoadConstraints(nlp, rbm);



% load seed
%[nlp, rbm] = LoadSeed(nlp, rbm);
[nlp, rbm] = LoadSeed(nlp, rbm, 'spatial-12-dof-seed.mat');
 


nlp = ParseNLP(nlp, rbm);



% SOLVE
nlp = SolveNLP(nlp);


%% EXTRACT SOLUTION

% structure containing the solution data
data = ExtractData(nlp, rbm)


%% ANIMATE SEED

qAnim = rbm.States.q.Seed;
tAnim = linspace(0, nlp.Problem.FinalTime.UpperBound-nlp.Problem.FinalTime.LowerBound, nlp.Settings.ncp);


%% REFINE BOUNDS

qAnim = [0.2, 0.2, 1.0, 0.0, 0.0, 0.0, -0.2, -0.8, 0.3, 0.2, -0.4, 0.3]';
tAnim = 0;


%% FLIP MAP

R = Model.RelabelingMatrix();
qAnim = R*qAnim;


%% ANIMATE SOLUTION

qAnim = data.pos;
tAnim = data.t;


%% ANIMATE

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
anim_options.movie_name    = 'five_link.mp4';
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
str2save = 'spatial-5link-seed.mat';
save(str2save, 'seed')





