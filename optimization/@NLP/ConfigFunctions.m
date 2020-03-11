% function [casadi] = configure_functions( rbm , nlp )
function [obj] = ConfigFunctions(obj, rbm)

arguments
    obj (1,1) NLP
    rbm (1,1) DynamicalSystem
    %contactD (1,1) ContactDynamics
end


%fprintf('Dynamic options:')


% 2nd-order dynamics
obj.Functions.DynamicsODE = ContinuousSecondOrder(obj, rbm);%, contactDyna)%varargin)

% Constrained 2nd-order dynamics
nContact = size(rbm.Contacts, 2);
if nContact ~= 0
    obj.Functions.ConstrainedDynamicsODE = ConstrainedDynamics(obj, rbm);%, contactD)%JcArg, dJcArg)
end



% inertia matrix needed for impact
obj.Functions.InertiaMatrix = Function('f', {rbm.States.q.sym} , {rbm.Dynamics.H_matrix} );


% contact jacobian for impact

if nContact ~= 0
    for i = 1:nContact

%         if i == 1
%             combinedJacobian = rbm.Contacts{i}.Jac_contact;
%         else 
%             combinedJacobian = [combinedJacobian; rbm.Contacts{i}.Jac_contact];
%         end
        
        obj.Functions.ContactJacobian{i} = Function('f', {rbm.States.q.sym}, {rbm.Contacts{i}.Jac_contact});

    end
end
% obj.Functions.ContactJacobian = Function('f', {rbm.States.q.sym}, {combinedJacobian});


% position and velocity of center of mass        
obj.Functions.PositionCOM = Function('f', {rbm.States.q.sym} , {rbm.Dynamics.p_com});
obj.Functions.VelocityCOM = Function('f', {rbm.States.q.sym, rbm.States.dq.sym} , {rbm.Dynamics.v_com});



return                

frrfrf

import casadi.*

% state variables
q   = rbm.model.q
qd  = rbm.model.qd
qdd = rbm.model.qdd
%x = [ q ; qd ];

% number of DOF
%NB = rbm.model.NB;

% define control as decision variables
%u = SX.sym('u',size(rbm.model.B,2));
u = rbm.model.u;


% inertia matrix
H = rbm.H_matrix;

% % inertia matrix needed for impact
% if ~isempty(rbm.J_contact)
%     casadi.functions.H = Function('f', { x } , { H } );
% end

% vector of coriolis and gravitational terms
C = rbm.C_terms;

% input matrix
B = rbm.model.B;




%{
    define the dynamics equations (2nd-order ode + holonomic 
    constraint) that will be enforced implicitly
%}

fprintf('\n\t- equations of motion: enforced implicitly')

switch rbm.dynamics
    case 'continuous'

        % 2nd-order dynamics
    case 'hybrid'

       
end



% position and velocity of center of mass        
obj.Functions.P_com = Function('f', { q } , { rbm.p_com } );
obj.Functions.V_com = Function('f', { q , qd } , { rbm.v_com } );

% position and velocity of end effector
obj.Functions.P_end_effector = Function('f', { q } , { rbm.p_end_effector } );
obj.Functions.V_end_effector = Function('f', { q , qd } , { rbm.v_end_effector } );



% load model-specific functions
%obj = extra_functions(obj, rbm);
extraFun = extra_functions(obj, rbm);
fdnames = fieldnames(extraFun);
for i = 1:numel(fdnames)
    obj.Functions.(fdnames{i}) = extraFun.(fdnames{i});
end



% load model-specific functions
%[casadi] = Opt.extra_functions( rbm , casadi , x );

% get x_minus equivalent
% if ~isempty(rbm.J_contact)
%     [q_minus, qd_minus] = Model.get_x_minus( rbm );
%     casadi.functions.q_minus  = Function('f', { x } , { q_minus } );     
%     casadi.functions.qd_minus = Function('f', { x } , { qd_minus } );  
% end


% switch rbm.dynamics
% 
%     case 'hybrid'
% 
%         [q_minus, qd_minus] = Model.get_x_minus( rbm );
%         obj.Functions.q_minus  = Function('f', { q } , { q_minus } );     
%         obj.Functions.qd_minus = Function('f', { q , qd } , { qd_minus } );  
%         
% end






switch obj.Problem.desired_trajectory.option

    case 'virtual-constraint'

        fprintf('\n\t- desired trajectories: virtual constraints')
        
        % phase variable at t = 0
        t_plus = SX.sym('phase_var_0');

        % phase variable at t = T
        t_minus = SX.sym('phase_var_T');

        % normalized phase variable  
        s_var = SX.sym('norm_phase_var');

        switch obj.Problem.desired_trajectory.type
            
            case 'bezier'

                % number of actuated DOF
                number_actuated_DOF = size(rbm.model.B,2); 

                % order of bezier polynomial
                bezier_order = obj.Problem.desired_trajectory.order;
                
                % define the matrix of Bezier coefficients
                alpha = Control.bezier_coefficients( bezier_order , number_actuated_DOF );

                % compute symbolic forms of desired trajectories and derivatives
                [ phi , dphi_dtheta , d2phi_dtheta2 ] = Control.bezier_trajectory( alpha , s_var , t_minus , t_plus );

        
            otherwise
                error('Only Bezier polynomials are supported for now.')
                
        end
        
        
        parameterization_type = obj.Problem.desired_trajectory.param;
        
        
        
        % controller outputs and time derivatives
        [Y,DY,DDY] = Control.controller_output( rbm , q , qd , qdd , phi , dphi_dtheta , d2phi_dtheta2 , parameterization_type );

        

        obj.Functions.Control.y   = Function('f',{ q , alpha , s_var , t_plus , t_minus } , { Y } );
        obj.Functions.Control.dy  = Function('f',{ q , qd , alpha , s_var , t_plus , t_minus } , { DY } );
        obj.Functions.Control.ddy = Function('f',{ q , qd , qdd , alpha , s_var , t_plus , t_minus } , { DDY } );

        
        
    case 'free'
        
        %error('need to add this')
        fprintf('\n\t- desired trajectories: free')

    otherwise
        error('Options for desired_trajectory are ''free'', ''virtual-constraint''.')

end



fprintf('\n')
dline(1,'-')
fprintf('\n')





end