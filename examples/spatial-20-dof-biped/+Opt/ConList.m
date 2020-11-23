function [obj, rbm] = ConList(obj, rbm, ncp)
% this function loads the constraints

arguments
    obj (1,1) ConstraintList
    rbm (1,1) DynamicalSystem
    ncp (1,1) double
end


import casadi.*

% final time (s)
obj.FinalTime.Name = 'Step period';
obj.FinalTime.Bool = true;
obj.FinalTime.LowerBound = 0.15;
obj.FinalTime.UpperBound = 0.75;


% step width (m)
obj.StepWidth.Name = 'Step width';
obj.StepWidth.Bool = true;
obj.StepWidth.Function = Function('f', {rbm.States.q.sym}, {rbm.BodyPositions{9,2}(2)});
obj.StepWidth.LowerBound = 0.10;
obj.StepWidth.UpperBound = 0.20;


% step length (m)
obj.StepLength.Name = 'Step length';
obj.StepLength.Bool = true;
obj.StepLength.Function = Function('f', {rbm.States.q.sym}, {rbm.BodyPositions{9,2}(1)});
obj.StepLength.LowerBound = 0.1;
obj.StepLength.UpperBound = 0.2;


% step height (m)
obj.StepHeight.Name = 'Step height';      
obj.StepHeight.Bool = true;
obj.StepHeight.Function = Function('f', {rbm.States.q.sym}, {rbm.BodyPositions{9,2}(3)});
obj.StepHeight.LowerBound = 0;      
obj.StepHeight.UpperBound = 0;


% desired walking speed (m/s)
obj.ForwardWalkingSpeed.Name = 'Walking speed';      
obj.ForwardWalkingSpeed.Bool = true;
obj.ForwardWalkingSpeed.LowerBound = 0.20;  % 0.20    
obj.ForwardWalkingSpeed.UpperBound = 0.20;  % 0.20


% minimum velocity phase variable (rad/s or m/s)
obj.PhaseVariableDerivative.Name = 'Theta dot';
obj.PhaseVariableDerivative.Bool = true;
obj.PhaseVariableDerivative.LowerBound = 0.05;
obj.PhaseVariableDerivative.UpperBound = Inf;


% swing foot height (m)
obj.SwingFootHeight.Name = 'Swing foot height';
obj.SwingFootHeight.Bool = false;
obj.SwingFootHeight.Timing = ceil(ncp/2);
obj.SwingFootHeight.Function = Function('f', {rbm.States.q.sym}, {rbm.BodyPositions{9,2}(3)});
obj.SwingFootHeight.LowerBound = 0.05;
obj.SwingFootHeight.UpperBound = Inf;


% ground clearance (m)
obj.GroundClearance.Name = 'Ground clearance';
obj.GroundClearance.Bool = true;
obj.GroundClearance.Function = Function('f', {rbm.States.q.sym}, {rbm.BodyPositions{9,2}(3)});
obj.GroundClearance.LowerBound = 0;
obj.GroundClearance.UpperBound = Inf;


% stance foot initial position (m)
obj.StanceFootInitialPosition.Name = 'Stance position at 0';
obj.StanceFootInitialPosition.Bool = true;
obj.StanceFootInitialPosition.Function = Function('f', {rbm.States.q.sym}, {rbm.BodyPositions{12,2}});
obj.StanceFootInitialPosition.LowerBound = [0;0;0];
obj.StanceFootInitialPosition.UpperBound = [0;0;0];


% stance foot initial velocity (m/s)
obj.StanceFootInitialVelocity.Name = 'Stance velocity at 0';
obj.StanceFootInitialVelocity.Bool = true;
obj.StanceFootInitialVelocity.Function = Function('f', {rbm.States.q.sym, rbm.States.dq.sym}, {rbm.BodyVelocities{12,2}});
obj.StanceFootInitialVelocity.LowerBound = [0;0;0];
obj.StanceFootInitialVelocity.UpperBound = [0;0;0];


% swing foot vertical impact velocity (m/s)
obj.SwingFootVerticalImpactVelocity.Name = 'Swing foot vertical impact velocity';
obj.SwingFootVerticalImpactVelocity.Bool = true;
obj.SwingFootVerticalImpactVelocity.Function = Function('f', {rbm.States.q.sym, rbm.States.dq.sym}, {rbm.BodyVelocities{9,2}(3)});
obj.SwingFootVerticalImpactVelocity.LowerBound = -Inf;
obj.SwingFootVerticalImpactVelocity.UpperBound = -0.1;


% swing foot lateral impact velocity (m/s)
obj.SwingFootLateralImpactVelocity.Name = 'Swing foot lateral impact velocity';
obj.SwingFootLateralImpactVelocity.Bool = true;
obj.SwingFootLateralImpactVelocity.Function = Function('f', {rbm.States.q.sym, rbm.States.dq.sym}, {rbm.BodyVelocities{9,2}(2)});
obj.SwingFootLateralImpactVelocity.LowerBound = -1;
obj.SwingFootLateralImpactVelocity.UpperBound = 1;


% swing foot forward impact velocity (m/s)
obj.SwingFootForwardImpactVelocity.Name = 'Swing foot forward impact velocity';
obj.SwingFootForwardImpactVelocity.Bool = true;
obj.SwingFootForwardImpactVelocity.Function = Function('f', {rbm.States.q.sym, rbm.States.dq.sym}, {rbm.BodyVelocities{9,2}(1)});
obj.SwingFootForwardImpactVelocity.LowerBound = -1;
obj.SwingFootForwardImpactVelocity.UpperBound = 1;


% foot interference
obj.FootInterference.Name = 'Foot interference';
obj.FootInterference.Bool = false;
obj.FootInterference.Function = Function('f', {rbm.States.q.sym}, {rbm.BodyPositions{9,2}(2) - rbm.BodyPositions{12,2}(2)});
obj.FootInterference.LowerBound = 0.05;
obj.FootInterference.UpperBound = Inf;


        
% slew rate (N.m/s)
obj.SlewRate.Name = 'Actuator slew rate';
obj.SlewRate.Bool = false;
obj.SlewRate.LowerBound = -1/0.0001;
obj.SlewRate.UpperBound = 1/0.0001;


% Bezier coefficients
if obj.Trajectory.Bool
    obj.Trajectory.PolyCoeff.LowerBound = -5*ones(14,6);
    obj.Trajectory.PolyCoeff.UpperBound = 5*ones(14,6);
end


% periodicity constraints
obj.OneStepPeriodic.Name = 'Periodicity constraints';
obj.OneStepPeriodic.Bool = true;




% velocity bounds
rbm.States.dq.LowerBound = -20*ones(20,ncp);
rbm.States.dq.UpperBound = 20*ones(20,ncp);

% acceleration bounds
rbm.States.ddq.LowerBound = -100*ones(20,ncp);
rbm.States.ddq.UpperBound = 100*ones(20,ncp);

% torque limits (N.m)
torque_limits = 300; 
rbm.Inputs.u.LowerBound = -torque_limits*ones(14,ncp);
rbm.Inputs.u.UpperBound = torque_limits*ones(14,ncp);


% contact forces (N)
rbm.Contacts{1}.Fc.LowerBound = -1000*ones(3,ncp);
rbm.Contacts{1}.Fc.UpperBound = 1000*ones(3,ncp);



% position bounds
rbm.States.q.LowerBound = -pi*ones(20,ncp);
rbm.States.q.UpperBound = pi*ones(20,ncp);

% x_hip
rbm.States.q.LowerBound(1,:) = -0.2*ones(1,ncp);
rbm.States.q.UpperBound(1,:) = 0.5*ones(1,ncp);

% y_hip
rbm.States.q.LowerBound(2,:) = -0.05*ones(1,ncp);
rbm.States.q.UpperBound(2,:) = 0.5*ones(1,ncp);

% z_hip
rbm.States.q.LowerBound(3,:) = 0.5*ones(1,ncp);
rbm.States.q.UpperBound(3,:) = 1.2*ones(1,ncp);

% phi_x
rbm.States.q.LowerBound(4,:) = -pi/100*ones(1,ncp);
rbm.States.q.UpperBound(4,:) = pi/100*ones(1,ncp);

% phi_y
% rbm.States.q.LowerBound(5,:) = 0*ones(1,ncp);
% rbm.States.q.UpperBound(5,:) = pi/30*ones(1,ncp);
rbm.States.q.LowerBound(5,:) = -pi/100*ones(1,ncp);
rbm.States.q.UpperBound(5,:) = pi/100*ones(1,ncp);

% phi_z
rbm.States.q.LowerBound(6,:) = -pi/100*ones(1,ncp);
rbm.States.q.UpperBound(6,:) = pi/100*ones(1,ncp);

% stance_ab/ad
rbm.States.q.LowerBound(7,:) = -pi/30*ones(1,ncp);
rbm.States.q.UpperBound(7,:) = 0*ones(1,ncp);

% stance hip
rbm.States.q.LowerBound(8,:) = -pi/8*ones(1,ncp);
rbm.States.q.UpperBound(8,:) = pi/8*ones(1,ncp);

% stance_knee
rbm.States.q.LowerBound(9,:) = deg2rad(5)*ones(1,ncp);
rbm.States.q.UpperBound(9,:) = pi/10*ones(1,ncp);

% swing_ab/ad
rbm.States.q.LowerBound(10,:) = 0*ones(1,ncp);
rbm.States.q.UpperBound(10,:) = pi/30*ones(1,ncp);

% swing hip
rbm.States.q.LowerBound(11,:) = -pi/8*ones(1,ncp);
rbm.States.q.UpperBound(11,:) = pi/8*ones(1,ncp);

% swing_knee
rbm.States.q.LowerBound(12,:) = deg2rad(5)*ones(1,ncp);
rbm.States.q.UpperBound(12,:) = pi/6*ones(1,ncp);

% shoulder yaw
rbm.States.q.LowerBound(13,:) = -pi/100*ones(1,ncp);
rbm.States.q.UpperBound(13,:) = pi/100*ones(1,ncp);

% head
rbm.States.q.LowerBound(14,:) = -pi/100*ones(1,ncp);
rbm.States.q.UpperBound(14,:) = pi/100*ones(1,ncp);

% shoulder 1 in/out
rbm.States.q.LowerBound(15,:) = -pi/100*ones(1,ncp);
rbm.States.q.UpperBound(15,:) = pi/100*ones(1,ncp);

% shoulder 1 back/forth
rbm.States.q.LowerBound(16,:) = -pi/10*ones(1,ncp);
rbm.States.q.UpperBound(16,:) = pi/10*ones(1,ncp);

% elbow 1
rbm.States.q.LowerBound(17,:) = -pi/10*ones(1,ncp);
rbm.States.q.UpperBound(17,:) = -pi/30*ones(1,ncp);

% shoulder 2 in/out
rbm.States.q.LowerBound(18,:) = -pi/100*ones(1,ncp);
rbm.States.q.UpperBound(18,:) = pi/100*ones(1,ncp);

% shoulder 2 back/forth
rbm.States.q.LowerBound(19,:) = -pi/10*ones(1,ncp);
rbm.States.q.UpperBound(19,:) = pi/10*ones(1,ncp);

% elbow 2
rbm.States.q.LowerBound(20,:) = -pi/10*ones(1,ncp);
rbm.States.q.UpperBound(20,:) = -pi/30*ones(1,ncp);




end