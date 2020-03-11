function [obj, rbm] = ConList(obj, rbm, ncp)
% this function loads the constraints

arguments
    obj (1,1) ConstraintList
    %traj (1,1) %double
    rbm (1,1) DynamicalSystem
    ncp (1,1) double
end

%fknfr
%validateattributes('ConstraintList')


import casadi.*

% final time (s)
obj.FinalTime.Name = 'Step period';
obj.FinalTime.Bool = true;
obj.FinalTime.LowerBound = 0.25;
obj.FinalTime.UpperBound = 0.75;


% step length (m)
obj.StepLength.Name = 'Step length';
obj.StepLength.Bool = true;
obj.StepLength.Function = Function('f', {rbm.States.q.sym}, {rbm.BodyPositions{7,2}(1)});
obj.StepLength.LowerBound = 0.1;
obj.StepLength.UpperBound = 0.5;


% step height (m)
obj.StepHeight.Name = 'Step height';      
obj.StepHeight.Bool = true;
obj.StepHeight.Function = Function('f', {rbm.States.q.sym}, {rbm.BodyPositions{7,2}(3)});
obj.StepHeight.LowerBound = 0;      
obj.StepHeight.UpperBound = 0;


% desired walking speed (m/s)
obj.ForwardWalkingSpeed.Name = 'Walking speed';      
obj.ForwardWalkingSpeed.Bool = true;
obj.ForwardWalkingSpeed.LowerBound = 0.40;      
obj.ForwardWalkingSpeed.UpperBound = 0.40;


% minimum velocity phase variable (rad/s or m/s)
obj.PhaseVariableDerivative.Name = 'Theta dot';
obj.PhaseVariableDerivative.Bool = true;
obj.PhaseVariableDerivative.LowerBound = 0.05;
obj.PhaseVariableDerivative.UpperBound = Inf;


% swing foot height (m)
obj.SwingFootHeight.Name = 'Swing foot height';
obj.SwingFootHeight.Bool = true;
obj.SwingFootHeight.Timing = ceil(ncp/2);
obj.SwingFootHeight.Function = Function('f', {rbm.States.q.sym}, {rbm.BodyPositions{7,2}(3)});
obj.SwingFootHeight.LowerBound = 0.1;
obj.SwingFootHeight.UpperBound = Inf;


% ground clearance (m)
obj.GroundClearance.Name = 'Ground clearance';
obj.GroundClearance.Bool = true;
obj.GroundClearance.Function = Function('f', {rbm.States.q.sym}, {rbm.BodyPositions{7,2}(3)});
obj.GroundClearance.LowerBound = 0;
obj.GroundClearance.UpperBound = Inf;


% stance foot initial position (m)
obj.StanceFootInitialPosition.Name = 'Stance position at 0';
obj.StanceFootInitialPosition.Bool = true;
obj.StanceFootInitialPosition.Function = Function('f', {rbm.States.q.sym}, {[rbm.BodyPositions{5,2}(1); rbm.BodyPositions{5,2}(3)]});
obj.StanceFootInitialPosition.LowerBound = [0;0];
obj.StanceFootInitialPosition.UpperBound = [0;0];


% stance foot initial velocity (m/s)
obj.StanceFootInitialVelocity.Name = 'Stance velocity at 0';
obj.StanceFootInitialVelocity.Bool = true;
obj.StanceFootInitialVelocity.Function = Function('f', {rbm.States.q.sym, rbm.States.dq.sym}, {[rbm.BodyVelocities{5,2}(1); rbm.BodyVelocities{5,2}(3)]});
obj.StanceFootInitialVelocity.LowerBound = [0;0];
obj.StanceFootInitialVelocity.UpperBound = [0;0];


% swing foot vertical impact velocity (m/s)
obj.SwingFootVerticalImpactVelocity.Name = 'Swing foot vertical impact velocity';
obj.SwingFootVerticalImpactVelocity.Bool = true;
obj.SwingFootVerticalImpactVelocity.Function = Function('f', {rbm.States.q.sym, rbm.States.dq.sym}, {rbm.BodyVelocities{7,2}(3)});
obj.SwingFootVerticalImpactVelocity.LowerBound = -Inf;
obj.SwingFootVerticalImpactVelocity.UpperBound = -0.25;



% swing foot forward impact velocity (m/s)
obj.SwingFootForwardImpactVelocity.Name = 'Swing foot forward impact velocity';
obj.SwingFootForwardImpactVelocity.Bool = true;
obj.SwingFootForwardImpactVelocity.Function = Function('f', {rbm.States.q.sym, rbm.States.dq.sym}, {rbm.BodyVelocities{7,2}(1)});
obj.SwingFootForwardImpactVelocity.LowerBound = -0.01;
obj.SwingFootForwardImpactVelocity.UpperBound = 0.01;


% slew rate (N.m/s)
obj.SlewRate.Name = 'Actuator slew rate';
obj.SlewRate.Bool = false;
obj.SlewRate.LowerBound = -1/0.001;
obj.SlewRate.UpperBound = 1/0.001;


% Bezier coefficients
if obj.Trajectory.Bool
    obj.Trajectory.PolyCoeff.LowerBound = -5*ones(4,6);
    obj.Trajectory.PolyCoeff.UpperBound = 5*ones(4,6);
end


% periodicity constraints
obj.OneStepPeriodic.Name = 'Periodicity constraints';
obj.OneStepPeriodic.Bool = true;



% velocity bounds
rbm.States.dq.LowerBound = -20*ones(7,ncp);
rbm.States.dq.UpperBound = 20*ones(7,ncp);

% acceleration bounds
rbm.States.ddq.LowerBound = -100*ones(7,ncp);
rbm.States.ddq.UpperBound = 100*ones(7,ncp);

% torque limits (N.m)
torque_limits = 100; 
rbm.Inputs.u.LowerBound = -torque_limits*ones(4,ncp);
rbm.Inputs.u.UpperBound = torque_limits*ones(4,ncp);


% contact forces (N)
warning('add new kind of contact for planar')

rbm.Contacts{1}.Fc.LowerBound = -1000*ones(2,ncp);
rbm.Contacts{1}.Fc.UpperBound = 1000*ones(2,ncp);

warning('add flexible constraints')

warning('add normal Force as flexible constraint')

% position bounds
rbm.States.q.LowerBound = -pi*ones(7,ncp);
rbm.States.q.UpperBound = pi*ones(7,ncp);

% x_hip
rbm.States.q.LowerBound(1,:) = -0.5*ones(1,ncp);
rbm.States.q.UpperBound(1,:) = 0.5*ones(1,ncp);

% z_hip
rbm.States.q.LowerBound(2,:) = 0.5*ones(1,ncp);
rbm.States.q.UpperBound(2,:) = 1.25*ones(1,ncp);

% torso
rbm.States.q.LowerBound(3,:) = -pi/6*ones(1,ncp);
rbm.States.q.UpperBound(3,:) = pi/6*ones(1,ncp);


% knees don't hyperextend
rbm.States.q.LowerBound(5,:) = deg2rad(5)*ones(1,ncp);
rbm.States.q.LowerBound(7,:) = deg2rad(5)*ones(1,ncp);



end