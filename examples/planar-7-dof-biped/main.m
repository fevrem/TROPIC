%% LOAD DYNAMICAL SYSTEM

clear all; clc; close all;

[rbm] = ld_model(...
    {'model', @Model.planar_7_dof_biped},...
    {'debug', false});
 
 


% SPECIFY CONTACT

% Right leg end
rbm.Contacts{1} = Contact(rbm, 'Point',...
    {'Friction', true},...
    {'FrictionCoefficient', 0.6},...
    {'FrictionType', 'Pyramid'},...
    {'ContactFrame', rbm.BodyPositions{5,2}([1,3])});



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
    {'PolyPhase', 'state-based'});





% load user-defined constraints
[nlp, rbm] = LoadConstraints(nlp, rbm);


% load seed
%[nlp, rbm] = LoadSeed(nlp, rbm);
[nlp, rbm] = LoadSeed(nlp, rbm, 'planar-7-dof-seed.mat');
 


nlp = ParseNLP(nlp, rbm);



%% SOLVE

nlp = SolveNLP(nlp);


%% EXTRACT SOLUTION

% structure containing the solution data
data = ExtractData(nlp, rbm)


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









%% get impact map

q_guard = [qn(1:3),-qn(4),qn(5),-qn(6),-qn(10),qn(11:12),-qn(7),qn(8:9)];

% can pass figure as 5th argument
Anim.animate( rbm , 0 , q_guard , anim_options )




%%


R = [1,0,0,0,0,0,0,0,0,0,0,0;
     0,1,0,0,0,0,0,0,0,0,0,0;
     0,0,1,0,0,0,0,0,0,0,0,0;
     0,0,0,-1,0,0,0,0,0,0,0,0;
     0,0,0,0,1,0,0,0,0,0,0,0;
     0,0,0,0,0,-1,0,0,0,0,0,0;
     0,0,0,0,0,0,0,0,0,-1,0,0;
     0,0,0,0,0,0,0,0,0,0,1,0;
     0,0,0,0,0,0,0,0,0,0,0,1;
     0,0,0,0,0,0,-1,0,0,0,0,0;
     0,0,0,0,0,0,0,1,0,0,0,0;
     0,0,0,0,0,0,0,0,1,0,0,0];
     
q_guard = R*qn'

% can pass figure as 5th argument
Anim.animate( rbm , 0 , q_guard , anim_options )


%% Extract


nlp
size(nlp.VarsName,2)
sol
x_opt = full(sol.x);

sol.pos = [];
sol.vel = [];
sol.acc = [];
sol.input = [];
sol.cForce = [];
sol.tf = [];

for i = 1:size(nlp.VarsName,2)
    i
    %nlp.VarsName{i}
    %x_opt(nlp.VarsIdx{i})
        
    sol.(nlp.VarsName{i}) = [sol.(nlp.VarsName{i}), x_opt(nlp.VarsIdx{i})];


end

%%

t = linspace(0, sol.tf, size(sol.pos,2))

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
Anim.animate(rbm , t , sol.pos , anim_options )
%set(gcf,'menubar','figure')
%set(gcf,'menubar','none')






%% minimal coordinates

qm  = rbm.model.q(size(rbm.J_contact,1)+1:end);
qdm = rbm.model.qd(size(rbm.J_contact,1)+1:end);

[rb_min] = Model.minimal_coordinates(rbm.model,rbm.J_contact);

[H_min,C_min] = Control.HandC_minimal(rb_min,qm,qdm);


%% EXPORT DYNAMICS

Export2c.main( rbm ,...
    {rbm.H_matrix   , 'H_matrix'    , [rbm.model.q]},...
    {rbm.C_terms    , 'C_terms'     , [rbm.model.q;rbm.model.qd]},...
    {rbm.p_com      , 'p_com'       , [rbm.model.q]},...
    {rbm.v_com      , 'v_com'       , [rbm.model.q;rbm.model.qd]},...
    {rbm.KE         , 'KE'          , [rbm.model.q;rbm.model.qd]},...
    {rbm.PE         , 'PE'          , [rbm.model.q]},...
    {rbm.p_end_effector , 'p_end_effector'   , [rbm.model.q]},...
    {rbm.v_end_effector , 'v_end_effector'   , [rbm.model.q;rbm.model.qd]},...
    {rbm.J_contact      , 'Jc'      , [rbm.model.q]},...
    {rbm.Jdot_contact   , 'Jc_dot'  , [rbm.model.q;rbm.model.qd]}) 


%% OPTIMIZE GAIT
    
import casadi.* % CasADi 3.4.5

% plot options (type: bool), turn off for CRC 
plt_opt.bool = false;

% number of collocation points
nlp.settings.ncp = 25;

% options: 'ma57', 'ma27', 'mumps'
nlp.settings.linear_solver = 'ma57';

% options: 'implicit' or 'explicit'
nlp.settings.dynamics_ode = 'implicit';

% options: 'open-loop' or 'closed-loop'
nlp.settings.control_type = 'open-loop';

% options: 'free' or 'virtual-constraint' 
nlp.settings.desired_trajectory = 'virtual-constraint';

% order of Bezier polynomial if 'virtual-constraint' is selected
nlp.settings.bezier_order = 6;

% control gains (Kp = eps^2 , Kd = 2*eps)
nlp.con.control.eps = 10;

% configure dynamics and control functions
[casadi] = Optim.configure_functions( rbm , nlp );

% choose an existing seed or generate one manually
seed = Optim.load_seed( rbm , 'gen_seed', {'ncp',nlp.settings.ncp} , {'tf_guess',0.5} , casadi , nlp , plt_opt );
%seed = Optim.load_seed( rbm , 'spatial-5link-seed.mat' , {'ncp',nlp.settings.ncp} , [] , casadi , nlp , plt_opt );

% load cost function
nlp = Opt.objective_fun(nlp);

% load list of constraints
nlp = Opt.con_list( nlp , seed , plt_opt );

%%

% build & solve the nlp
[sol, nlp, problem, seed] = Optim.configure_nlp( nlp , rbm , casadi , seed , plt_opt );
  

%% CHECK NLP INITIALIZATION

% options: true (write report.txt) or false (display report)
write_file = true;

% print the report
init_report( problem , write_file )




%% PLOT SOLUTION  

% options: 'time' or 'gait-cycle'
Plt.optim_data( 'time' , data , rbm , nlp )
 

%% SAVE SOLUTION  

% can be used as seed
str2save = 'spatial-5link-seed.mat';
Optim.save_result( data , rbm , str2save )






%%


load('spatial-5link-seed.mat')

data = optimal_solution;
data.q = data.q';


nlp.Functions = casadi.functions;

%% ANIMATE

clear qToAnim tToAnim qsteprev q_minus qVec t_interp q_interp

% number of steps to animate
N_steps = 2;

if 1 % for smoother videos 
    NP = 50;
    t_interp = linspace(data.t(1),data.t(end),NP);
    for i = 1:size(data.q,2)
        q_interp(:,i) = interp1( data.t, data.q(:,i)', t_interp )';
    end
    data.q = q_interp;
    data.t = t_interp;
end


qToAnim = data.q;
tToAnim = data.t;
for k = 1:N_steps
   
    k
    qsteprev = [data.q(:,1) - mod(k,2)*data.step_width , data.q(:,2) - k*data.step_length , data.q(:,3:12)];

    if mod(k,2) == 1
        for i = 1:length(data.t)

            q_minus  = nlp.Functions.q_minus( [qsteprev(i,:)';zeros(12,1)] );
            [q_minus, ~] = Model.flip_map( rbm , q_minus , zeros(12,1) );
            qVec(:,i) = casadi2double( q_minus );

        end
        
    else
        qVec = qsteprev';
    end
    
    qToAnim = [ qToAnim ; qVec(:,2:end)' ] ;
    tToAnim = [tToAnim,data.t(2:end)+data.t(end)*k];
    
end



%%
% options: true or false 
anim_options.bool = true;

% set axes if provided (p_com.mex otherwise)
anim_options.axis.x = [-1 1];
anim_options.axis.y = [-1.0 1.5];
anim_options.axis.z = [ 0.0 1.5];

% skips frame to animate faster 
anim_options.skip_frame = 2;

% views to show. Options: {'3D','frontal','sagittal','top'}
%anim_options.views = {'3D','frontal','sagittal','top'};
anim_options.views = {'3D'};

% save options
anim_options.save_movie    = true;
anim_options.movie_name    = 'five_link.mp4';
anim_options.movie_quality = 100; % scalar between [0 100], default 75
anim_options.movie_fps     = 30;  % frame rate, default 30

% create a light object or not
anim_options.lights = true;

% can pass figure as 5th argument
Anim.animate( rbm , tToAnim , qToAnim' , anim_options )
%set(gcf,'menubar','figure')
%set(gcf,'menubar','none')














%%





% --------------------------- symmetric step ---------------------------- %

px_sw = zeros(ncp,1);
py_sw = zeros(ncp,1);
pz_sw = zeros(ncp,1);
xd_sw = zeros(ncp,1);
yd_sw = zeros(ncp,1);
zd_sw = zeros(ncp,1);
for i = 1:ncp
    
    [px_sw_temp,py_sw_temp,pz_sw_temp,vx_sw_temp,vy_sw_temp,vz_sw_temp] = CasADifun.fun_swfoot( [q_guess(:,i) ; qd_guess(:,i)]' );
    px_sw(i,1) = py_sw_temp.to_double();
    py_sw(i,1) = -px_sw_temp.to_double();
    pz_sw(i,1) = pz_sw_temp.to_double();
    xd_sw(i,1) = vy_sw_temp.to_double();
    yd_sw(i,1) = -vx_sw_temp.to_double();
    zd_sw(i,1) = vz_sw_temp.to_double();
    
end


q_flipped  = zeros(NB,ncp);
qd_flipped = zeros(NB,ncp);
for i = 1:ncp
    
    [ q_flipped_temp , qd_flipped_temp ] = CasADifun.fun_flip_q_qd( [q_guess(:,i) ; qd_guess(:,i)]' , px_sw(end,1) , py_sw(1,1) );

    q_flipped(:,i)  = CasADi_DM_2_double(q_flipped_temp);
    qd_flipped(:,i) = CasADi_DM_2_double(qd_flipped_temp);

end


% ------------------------- concatenate 2 steps ------------------------- %
t_2step  = [ t_guess, t_guess + t_guess(end) ];
q_2step  = [ q_flipped  , q_guess  ];
qd_2step = [ qd_flipped , qd_guess ];
if anim_params.bool
    showmotion_setup(robot,robot.NB,t_2step(1),q_2step(:,1),'scroll')
    w = waitforbuttonpress;
    for k = 1:2 % repeat step
        for i = 1:1:2*ncp
            pause(0.01)
            showmotion_update(robot,robot.NB,t_2step(i),q_2step(:,i),'scroll')
        end
    end
end




%%



Np = 100;
t_new = linspace(t(1),t(end),Np);
q_new = [];
for i = 1:12
    q_new = [q_new;interp1(t,x_opt(:,i),t_new)];
end
q_new(1,:) = 0.5*ones(1,Np);

t_new = [t_new,t_new+t_new(end)];
q_prev = q_new;
p_swing_foot = casadi.functions.p_end_effector( [q_prev(:,end);zeros(12,1)] );    
p_swing_foot = casadi2double( p_swing_foot );
q_prev(1,:) = -p_swing_foot(2)*ones(1,Np);
q_prev(2,:) = -p_swing_foot(1)*ones(1,Np);
q_prev(3,:) = p_swing_foot(3)*ones(1,Np);
for i = 1:Np
    q_minus = casadi.functions.q_minus( [q_prev(:,i);zeros(12,1)] );
    [q_minus, ~] = Model.flip_map( rbm , q_minus , zeros(12,1) );
    q_minus = casadi2double(q_minus);
    q_new = [q_new, q_minus];
end



for n = 1:5
    
    new_stance = q_new(1:3,end);
    q_new = [q_new, [ new_stance(1)*ones(1,Np);new_stance(2)*ones(1,Np);new_stance(3)*ones(1,Np);q_new(4:12,1:Np)] ];
    t_new = [t_new,t_new(1:2*Np)+t_new(end)];

    q_prev = q_new(:,end-Np+1:end);
    p_swing_foot = casadi.functions.p_end_effector( [q_prev(:,end);zeros(12,1)] );    
    p_swing_foot = casadi2double( p_swing_foot );
    q_prev(1,:) = -p_swing_foot(2)*ones(1,Np);
    q_prev(2,:) = -p_swing_foot(1)*ones(1,Np);
    q_prev(3,:) = p_swing_foot(3)*ones(1,Np);
    for i = 1:Np
        q_minus = casadi.functions.q_minus( [q_prev(:,i);zeros(12,1)] );
        [q_minus, ~] = Model.flip_map( rbm , q_minus , zeros(12,1) );
        q_minus = casadi2double(q_minus);
        q_new = [q_new, q_minus];
    end

end


    

anim_options.bool = true;
anim_options.axis.x = [-0.5 1.5];
anim_options.axis.y = [-0.5 1.5];
anim_options.axis.z = [0 1.1];

anim_options.skip_frame    = 5;
anim_options.save_movie    = false;
anim_options.movie_name    = 'five_link_open_loop.mp4';
anim_options.movie_quality = 100; % scalar between [0 100], default 75
anim_options.movie_fps     = 30;  % frame rate, default 30
%anim_options.vertical_axis_moving = true;
%anim_options.numStepsShow  = 4;
%anim_options.sampleRate    = 500; 
%animate_2link3(out,animOptions,model_params)


Anim.animate(rbm,t_new,q_new,anim_options)
%Anim.animate(rbm,t_new(1:10:end),q_new(:,1:10:end),anim_options)




%% ANIMATE

% options: true or false 
anim_options.bool = true;

% set axes if provided (p_com.mex otherwise)
anim_options.axis.x = [-1.0 0.5];
anim_options.axis.y = [-0.5 1.0];
anim_options.axis.z = [ 0.0 1.25];

% skips frame to animate faster 
anim_options.skip_frame = 1;

% views to show. Options: {'3D','frontal','sagittal','top'}
%anim_options.views = {'3D','frontal','sagittal','top'};
anim_options.views = {'3D'};

% save options
anim_options.save_movie    = false;
anim_options.movie_name    = 'five_link.mp4';
anim_options.movie_quality = 100; % scalar between [0 100], default 75
anim_options.movie_fps     = 30;  % frame rate, default 30

% create a light object or not
anim_options.lights = true;

% can pass figure as 5th argument
Anim.animate( rbm , data.t , data.x(:,1:rbm.model.NB)' , anim_options )
%set(gcf,'menubar','figure')
%set(gcf,'menubar','none')







