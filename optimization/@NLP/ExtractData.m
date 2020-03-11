function [out] = ExtractData(nlp, rbm)

arguments
    nlp (1,1) NLP
    rbm (1,1) DynamicalSystem
end


% number of DOF
NB = numel(rbm.States.q.sym);

% number of collocation points
NCP = nlp.Settings.ncp;

% number of control inputs
NU = numel(rbm.Inputs.u.sym);

% number of contacts
NC = numel(rbm.Contacts);

% number of holonomic constraints
%NHC = numel(rbm.J_contact,1)

% options of desired trajectory
desired_trajectory = nlp.Problem.Trajectory;

% useful kinematic functions
funs = nlp.Functions;

% user-defined constraints
con = nlp.Problem;

% extract the solution
w_opt = full(nlp.Sol.x);

%lb_opt = nlp.VarsLB;
%ub_opt = nlp.VarsUB;
%tol = nlp.Settings.constraint_tol;

% initialize data structure
out = struct();


for i = 1:numel(nlp.VarsName)    
    if ~isfield(out, nlp.VarsName{i})
        out.(nlp.VarsName{i}) = [];
    end
    out.(nlp.VarsName{i}) = [out.(nlp.VarsName{i}), w_opt(nlp.VarsIdx{i})];
end



if isfield(out, 'a')
   out.a = reshape(out.a, NU, desired_trajectory.PolyOrder+1); 
end


if isfield(out, 'tf')
   out.t = linspace(0, out.tf, NCP);
   out = rmfield(out, 'tf');
end


% Some of these do not mean much in some cases but still output them
%if desired_trajectory.Bool
%switch desired_trajectory.PolyPhase
%case 'time-based'
out.p_time = [0; out.t(end)];
out.tau_time = ((1:NCP)-1)/(NCP-1);

%case 'state-based'
if isfield(rbm.Model, 'c')
    out.p_phase = (rbm.Model.c*[out.pos(:,1), out.pos(:,end)])';
    out.tau_phase = (rbm.Model.c*out.pos - out.p_phase(1))/(out.p_phase(2) - out.p_phase(1));
end
%end
%end



% position and velocity of CoM
out.p_com = zeros(3,NCP);
out.v_com = zeros(3,NCP);
for i = 1:NCP
    out.p_com(:,i) = full(nlp.Functions.PositionCOM(out.pos(:,i)));
    out.v_com(:,i) = full(nlp.Functions.VelocityCOM(out.pos(:,i)', out.vel(:,i)'));
end


import casadi.*
for i = 1:NB
    
    bodyposfun{i,1} = Function('f', {rbm.States.q.sym}, {rbm.BodyPositions{i,1}});
    bodyposfun{i,2} = Function('f', {rbm.States.q.sym}, {rbm.BodyPositions{i,2}});
    
    bodyvelfun{i,1} = Function('f', {rbm.States.q.sym, rbm.States.dq.sym}, {rbm.BodyVelocities{i,1}});
    bodyvelfun{i,2} = Function('f', {rbm.States.q.sym, rbm.States.dq.sym}, {rbm.BodyVelocities{i,2}});
    
    for k = 1:NCP
        out.pbody{i,1}(:,k) = full(bodyposfun{i,1}(out.pos(:,k)));
        out.pbody{i,2}(:,k) = full(bodyposfun{i,2}(out.pos(:,k)));
        
        out.vbody{i,1}(:,k) = full(bodyvelfun{i,1}(out.pos(:,k), out.vel(:,k)));
        out.vbody{i,2}(:,k) = full(bodyvelfun{i,2}(out.pos(:,k), out.vel(:,k)));
    end
    
end


  


if desired_trajectory.Bool
    
    switch desired_trajectory.PolyPhase
        case 'state-based'

            t_plus = out.p_phase(1);
            t_minus = out.p_phase(2);
            tau = out.tau_phase;

        case 'time-based'

            t_plus = out.p_time(1);
            t_minus = out.p_time(2);        
            tau = out.tau_time;
    end


    control_gain = desired_trajectory.ControlGain;
    for i = 1:NCP
        out.y(:,i) = full(funs.Y_Controller(out.pos(:,i), out.a, tau(i), t_plus, t_minus));
        out.dy(:,i) = full(funs.DY_Controller(out.pos(:,i), out.vel(:,i), out.a, tau(i), t_plus, t_minus));
        out.ddy(:,i) = full(funs.DDY_Controller(out.pos(:,i), out.vel(:,i), out.acc(:,i), out.a, tau(i), t_plus, t_minus));
        out.zd_error(:,i) = out.ddy(:,i) + control_gain^2*out.y(:,i) + 2*control_gain*out.dy(:,i);
    end
 

end





% -------------------- display step characteristics --------------------- %

fprintf(['\n\nOptimal solution: ', num2str(full(nlp.Sol.f),'%.3f')])

if con.FinalTime.Bool
    out.final_time = out.t(end);
    fprintf(['\nfinal time:     ', num2str(out.final_time,'%.3f'), ' s    ==> [' num2str(con.FinalTime.LowerBound,'%.3f') ',' num2str(con.FinalTime.UpperBound,'%.3f') ']'])
end

if con.StepLength.Bool
    out.step_length = full(con.StepLength.Function(out.pos(:,end)));
    fprintf(['\nstep length:   ', num2str(out.step_length,'%.3f'), ' m     ==> [' num2str(con.StepLength.LowerBound,'%.3f') ',' num2str(con.StepLength.UpperBound,'%.3f') ']'])
end

if con.StepWidth.Bool
    out.step_width = full(con.StepWidth.Function(out.pos(:,end)));
    fprintf(['\nstep width:     ', num2str(out.step_width,'%.3f') , ' m    ==> [' num2str(con.StepWidth.LowerBound,'%.3f') ',' num2str(con.StepWidth.UpperBound,'%.3f') ']'])
end

if con.ForwardWalkingSpeed.Bool
    out.walking_speed = out.step_length/out.final_time;
    fprintf(['\nstep velocity: ', num2str(out.walking_speed,'%.3f') , ' m/s   ==> [' num2str(con.ForwardWalkingSpeed.LowerBound, '%.3f') ',' num2str(con.ForwardWalkingSpeed.UpperBound,'%.3f') ']'])
end

if con.StepHeight.Bool
    out.step_height = full(con.StepHeight.Function(out.pos(:,end)));
    fprintf(['\nstep height:   ', num2str(out.step_height,'%.3f'), ' m     ==> [' num2str(con.StepHeight.LowerBound,'%.3f') ',' num2str(con.StepHeight.UpperBound,'%.3f') ']'])
end
fprintf('\n\n')





       

end