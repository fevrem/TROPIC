% function [position2enforce,velocity2enforce] = PeriodicityConstraints(rbm, H, impulseContribution, x0, xEnd, xFlipped)
function [PosPeriodicity,VelPeriodicity] = PeriodicityConstraints(rbm, H, impulseContribution, x0, xEnd, xFlipped)


q0 = x0(1:11);
dq0 = x0(12:22);

qEnd = xEnd(1:11);
dqEnd = xEnd(12:22);

qFlipped = xFlipped(1:11);
dqFlipped = xFlipped(12:22);













%{
    do not solve IK (discontinuity)
    instead, enforce position of knee
%}

%bodpos_ex(rbm, qAnim)

%q_old = qAnim;

import casadi.*

%p111fun = Function('f', {rbm.States.q.sym}, {rbm.BodyPositions{11,1}});
%p_swing_knee = full(p111fun(qAnim));

p112fun = Function('f', {rbm.States.q.sym}, {rbm.BodyPositions{11,2}});
p_swing_foot = p112fun(qEnd);

v112fun = Function('f', {rbm.States.q.sym, rbm.States.dq.sym}, {rbm.BodyVelocities{11,2}});
v_swing_foot = v112fun(qEnd, dqEnd);


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

% solve IK
q4_new_var = atan2(r23, r22);
q5_new_var = atan2(r31, r11);

q4_new_fun = Function('f', {rbm.States.q.sym}, {q4_new_var});
q4_new = q4_new_fun(qEnd);

q5_new_fun = Function('f', {rbm.States.q.sym}, {q5_new_var});
q5_new = q5_new_fun(qEnd);


qFlipped(1) = p_swing_foot(1); % x
qFlipped(2) = -p_swing_foot(2); % y
qFlipped(3) = p_swing_foot(3); % z
qFlipped(4) = -q4_new; % roll (rx)
qFlipped(5) = q5_new; % pitch (ry)
qFlipped(6) = -qEnd(11); % stance knee (ry)
qFlipped(7) = -qEnd(10); % stance hip (ry)
qFlipped(8) = qEnd(9); % stance ab/ad (rx)
qFlipped(9) = qEnd(8); % stance knee
qFlipped(10) = -qEnd(7); % stance knee
qFlipped(11) = -qEnd(6); % stance knee



dqFlipped(1) = v_swing_foot(1); % x
dqFlipped(2) = -v_swing_foot(2); % y
dqFlipped(3) = v_swing_foot(3); % z

dqFlipVar1 = jacobian(-q4_new_var, rbm.States.q.sym)*rbm.States.dq.sym;
dqFlipVar2 = jacobian(q5_new_var, rbm.States.q.sym)*rbm.States.dq.sym;

dqFlipFun1 = Function('f', {rbm.States.q.sym, rbm.States.dq.sym}, {dqFlipVar1});
dqFlipFun2 = Function('f', {rbm.States.q.sym, rbm.States.dq.sym}, {dqFlipVar2});

dqFlipped(4) = dqFlipFun1(qEnd, dqEnd);
dqFlipped(5) = dqFlipFun2(qEnd, dqEnd);

dqFlipped(6) = -dqEnd(11); % stance knee (ry)
dqFlipped(7) = -dqEnd(10); % stance hip (ry)
dqFlipped(8) = dqEnd(9); % stance ab/ad (rx)
dqFlipped(9) = dqEnd(8); % stance knee
dqFlipped(10) = -dqEnd(7); % stance knee
dqFlipped(11) = -dqEnd(6); % stance knee    

        
       











% dxDiscreteMap
dxMap = H*(dq0 - dqFlipped) - (impulseContribution);

% xDiscreteMap
xMap = q0 - qFlipped;

% xhip
%xMap(1) = 1;

% yhip
xMap(2) = q0(2) - qEnd(2);


% x0 are the initial conditions
% xPlus are the values after impact

% position2enforce = 2:12;
% velocity2enforce = 1:12;
position2enforce = 3:11;
velocity2enforce = 1:11;

%R_map = Model.RelabelingMatrix();

%qm  = nlp.Functions.q_minus( x );
%dqm = nlp.Functions.qd_minus( x , dx );


%qm = R_map*x;
%dqm = R_map*dx;

PosPeriodicity = xMap(position2enforce);
VelPeriodicity = dxMap(velocity2enforce);


%[position2enforce,velocity2enforce] = Opt.PeriodicityConstraints(rbm, [xn; dxn], [x; dx], [qm; dqm]);


%nlp = AddConstraint(nlp, dxMap(velocity2enforce), -nlp.Settings.ConstraintTolerance*ones(numel(velocity2enforce),1), nlp.Settings.ConstraintTolerance*ones(numel(velocity2enforce),1), 'Periodicity (velocities)');


%nlp = AddConstraint(nlp, xMap(position2enforce), -nlp.Settings.ConstraintTolerance*ones(numel(position2enforce),1), nlp.Settings.ConstraintTolerance*ones(numel(position2enforce),1), 'Periodicity (positions)');     

    
    
end
