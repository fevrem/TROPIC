% function [casadi] = configure_functions( rbm , nlp )
function [obj] = Functions(obj, rbm)


fprintf('\n')
dline(1,'-')
fprintf('\n')


fprintf('Dynamic options:')

import casadi.*

% state variables
q   = rbm.model.q;
qd  = rbm.model.qd;
qdd = rbm.model.qdd;
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



% final time is variable 
%tf = SX.sym('tf');


%{
    define the dynamics equations (2nd-order ode + holonomic 
    constraint) that will be enforced implicitly
%}

fprintf('\n\t- equations of motion: enforced implicitly')

switch rbm.dynamics
    case 'continuous'

        % 2nd-order dynamics
        
        second_order_ode = H*qdd + C - B*u ;
        obj.Functions.Dynamics.Equations = Function('f',{ [ q ; qd ; qdd ] , u } , { second_order_ode } );

    case 'hybrid'

        
        if 0
            

                % contact jacobian and time derivative
                %if ~isempty(rbm.J_contact)
                    Jc  = rbm.J_contact;
                    Jcd = rbm.Jdot_contact;
                %    casadi.functions.J_contact = Function('f', { x } , { Jc } );
                %    casadi.functions.Jdot_contact = Function('f', { x } , { Jcd} );
                %else
                %    Jc  = [];
                %    Jcd = [];
                %end

                % contact wrench


                % 2nd-order dynamics
                second_order_ode = H*qdd + C - B*u - Jc'*Fc;
                obj.Functions.Dynamics.Equations = Function('f', { [ q ; qd ; qdd ] , u , Fc } , { second_order_ode } );

                % holonomic constraints

                if 0
                    holonomic_constraints = Jc*qdd + Jcd*qd;
                    obj.Functions.Dynamics.Constraints = Function('f', { [ q ; qd ; qdd ] } , { holonomic_constraints } );

                else


                    %{
                        change to match FROST formulation
                    %}

                    pstvar = SX.sym('pstvar',2);

                    %q = casadi2double(sol.x(3:9))';
                    %qd = casadi2double(sol.x(10:16))';
                    %qdd = casadi2double(sol.x(17:23))';

                    holonomic_constraints = [(2*qdd(5))/5 + (2*qd(5))/5 - pstvar(1) + q(1) + (2*(1 - cos(q(5)))*(cos(q(4))*sin((q(3)+pi/2)) + cos((q(3)+pi/2))*sin(q(4))))/5 - (2*(cos((q(3)+pi/2))*cos(q(4)) - sin((q(3)+pi/2))*sin(q(4)))*sin(q(5)))/5 + (4*(cos(q(5))*(cos(q(4))*sin((q(3)+pi/2)) + cos((q(3)+pi/2))*sin(q(4))) + (cos((q(3)+pi/2))*cos(q(4)) - sin((q(3)+pi/2))*sin(q(4)))*sin(q(5))))/5 + qdd(4)*(((2*(1 - cos(q(5))))/5 + (4*cos(q(5)))/5)*cos(q(5)) + (2*sin(q(5))^2)/5) + qd(4)*(((2*(1 - cos(q(5))))/5 + (4*cos(q(5)))/5)*cos(q(5)) + (2*sin(q(5))^2)/5) + qdd(1)*(sin((q(3)+pi/2))*(-(cos(q(5))*sin(q(4))) - cos(q(4))*sin(q(5))) + cos((q(3)+pi/2))*(cos(q(4))*cos(q(5)) - sin(q(4))*sin(q(5)))) + qd(1)*(sin((q(3)+pi/2))*(-(cos(q(5))*sin(q(4))) - cos(q(4))*sin(q(5))) + cos((q(3)+pi/2))*(cos(q(4))*cos(q(5)) - sin(q(4))*sin(q(5)))) + qdd(2)*(cos((q(3)+pi/2))*(-(cos(q(5))*sin(q(4))) - cos(q(4))*sin(q(5))) - sin((q(3)+pi/2))*(cos(q(4))*cos(q(5)) - sin(q(4))*sin(q(5)))) + qd(2)*(cos((q(3)+pi/2))*(-(cos(q(5))*sin(q(4))) - cos(q(4))*sin(q(5))) - sin((q(3)+pi/2))*(cos(q(4))*cos(q(5)) - sin(q(4))*sin(q(5)))) + qdd(3)*(-((-(cos(q(5))*sin(q(4))) - cos(q(4))*sin(q(5)))*(((2*(1 - cos(q(5))))/5 + (4*cos(q(5)))/5)*sin(q(4)) + (2*cos(q(4))*sin(q(5)))/5)) - (cos(q(4))*((2*(1 - cos(q(5))))/5 + (4*cos(q(5)))/5) - (2*sin(q(4))*sin(q(5)))/5)*(-(cos(q(4))*cos(q(5))) + sin(q(4))*sin(q(5)))) + qd(3)*(-((-(cos(q(5))*sin(q(4))) - cos(q(4))*sin(q(5)))*(((2*(1 - cos(q(5))))/5 + (4*cos(q(5)))/5)*sin(q(4)) + (2*cos(q(4))*sin(q(5)))/5)) - (cos(q(4))*((2*(1 - cos(q(5))))/5 + (4*cos(q(5)))/5) - (2*sin(q(4))*sin(q(5)))/5)*(-(cos(q(4))*cos(q(5))) + sin(q(4))*sin(q(5)))) + qd(3)*(qd(2)*(-(sin((q(3)+pi/2))*(-(cos(q(5))*sin(q(4))) - cos(q(4))*sin(q(5)))) - cos((q(3)+pi/2))*(cos(q(4))*cos(q(5)) - sin(q(4))*sin(q(5)))) + qd(1)*(cos((q(3)+pi/2))*(-(cos(q(5))*sin(q(4))) - cos(q(4))*sin(q(5))) - sin((q(3)+pi/2))*(cos(q(4))*cos(q(5)) - sin(q(4))*sin(q(5))))) + qd(4)*(qd(2)*(-(sin((q(3)+pi/2))*(-(cos(q(5))*sin(q(4))) - cos(q(4))*sin(q(5)))) + cos((q(3)+pi/2))*(-(cos(q(4))*cos(q(5))) + sin(q(4))*sin(q(5)))) + qd(1)*(cos((q(3)+pi/2))*(-(cos(q(5))*sin(q(4))) - cos(q(4))*sin(q(5))) + sin((q(3)+pi/2))*(-(cos(q(4))*cos(q(5))) + sin(q(4))*sin(q(5)))) + qd(3)*(-((-(cos(q(5))*sin(q(4))) - cos(q(4))*sin(q(5)))*(cos(q(4))*((2*(1 - cos(q(5))))/5 + (4*cos(q(5)))/5) - (2*sin(q(4))*sin(q(5)))/5)) - (cos(q(5))*sin(q(4)) + cos(q(4))*sin(q(5)))*(cos(q(4))*((2*(1 - cos(q(5))))/5 + (4*cos(q(5)))/5) - (2*sin(q(4))*sin(q(5)))/5) - (-(((2*(1 - cos(q(5))))/5 + (4*cos(q(5)))/5)*sin(q(4))) - (2*cos(q(4))*sin(q(5)))/5)*(-(cos(q(4))*cos(q(5))) + sin(q(4))*sin(q(5))) - (((2*(1 - cos(q(5))))/5 + (4*cos(q(5)))/5)*sin(q(4)) + (2*cos(q(4))*sin(q(5)))/5)*(-(cos(q(4))*cos(q(5))) + sin(q(4))*sin(q(5))))) + qd(5)*(qd(4)*(-(((2*(1 - cos(q(5))))/5 + (4*cos(q(5)))/5)*sin(q(5))) + (2*cos(q(5))*sin(q(5)))/5) + qd(2)*(-(sin((q(3)+pi/2))*(-(cos(q(5))*sin(q(4))) - cos(q(4))*sin(q(5)))) + cos((q(3)+pi/2))*(-(cos(q(4))*cos(q(5))) + sin(q(4))*sin(q(5)))) + qd(1)*(cos((q(3)+pi/2))*(-(cos(q(5))*sin(q(4))) - cos(q(4))*sin(q(5))) + sin((q(3)+pi/2))*(-(cos(q(4))*cos(q(5))) + sin(q(4))*sin(q(5)))) + qd(3)*(-((cos(q(5))*sin(q(4)) + cos(q(4))*sin(q(5)))*(cos(q(4))*((2*(1 - cos(q(5))))/5 + (4*cos(q(5)))/5) - (2*sin(q(4))*sin(q(5)))/5)) - (-(cos(q(5))*sin(q(4))) - cos(q(4))*sin(q(5)))*((2*cos(q(4))*cos(q(5)))/5 - (2*sin(q(4))*sin(q(5)))/5) - ((-2*cos(q(5))*sin(q(4)))/5 - (2*cos(q(4))*sin(q(5)))/5)*(-(cos(q(4))*cos(q(5))) + sin(q(4))*sin(q(5))) - (((2*(1 - cos(q(5))))/5 + (4*cos(q(5)))/5)*sin(q(4)) + (2*cos(q(4))*sin(q(5)))/5)*(-(cos(q(4))*cos(q(5))) + sin(q(4))*sin(q(5)))));
                                            -pstvar(2) + q(2) + (2*(1 - cos(q(5)))*(cos((q(3)+pi/2))*cos(q(4)) - sin((q(3)+pi/2))*sin(q(4))))/5 - (2*(-(cos(q(4))*sin((q(3)+pi/2))) - cos((q(3)+pi/2))*sin(q(4)))*sin(q(5)))/5 + qdd(4)*(-(((2*(1 - cos(q(5))))/5 + (4*cos(q(5)))/5)*sin(q(5))) + (2*cos(q(5))*sin(q(5)))/5) + qd(4)*(-(((2*(1 - cos(q(5))))/5 + (4*cos(q(5)))/5)*sin(q(5))) + (2*cos(q(5))*sin(q(5)))/5) + (4*(cos(q(5))*(cos((q(3)+pi/2))*cos(q(4)) - sin((q(3)+pi/2))*sin(q(4))) + (-(cos(q(4))*sin((q(3)+pi/2))) - cos((q(3)+pi/2))*sin(q(4)))*sin(q(5))))/5 + qdd(3)*((((2*(1 - cos(q(5))))/5 + (4*cos(q(5)))/5)*sin(q(4)) + (2*cos(q(4))*sin(q(5)))/5)*(cos(q(4))*cos(q(5)) - sin(q(4))*sin(q(5))) + (-(cos(q(5))*sin(q(4))) - cos(q(4))*sin(q(5)))*(cos(q(4))*((2*(1 - cos(q(5))))/5 + (4*cos(q(5)))/5) - (2*sin(q(4))*sin(q(5)))/5)) + qd(3)*((((2*(1 - cos(q(5))))/5 + (4*cos(q(5)))/5)*sin(q(4)) + (2*cos(q(4))*sin(q(5)))/5)*(cos(q(4))*cos(q(5)) - sin(q(4))*sin(q(5))) + (-(cos(q(5))*sin(q(4))) - cos(q(4))*sin(q(5)))*(cos(q(4))*((2*(1 - cos(q(5))))/5 + (4*cos(q(5)))/5) - (2*sin(q(4))*sin(q(5)))/5)) + qdd(2)*(-(sin((q(3)+pi/2))*(-(cos(q(5))*sin(q(4))) - cos(q(4))*sin(q(5)))) + cos((q(3)+pi/2))*(-(cos(q(4))*cos(q(5))) + sin(q(4))*sin(q(5)))) + qd(2)*(-(sin((q(3)+pi/2))*(-(cos(q(5))*sin(q(4))) - cos(q(4))*sin(q(5)))) + cos((q(3)+pi/2))*(-(cos(q(4))*cos(q(5))) + sin(q(4))*sin(q(5)))) + qdd(1)*(cos((q(3)+pi/2))*(-(cos(q(5))*sin(q(4))) - cos(q(4))*sin(q(5))) + sin((q(3)+pi/2))*(-(cos(q(4))*cos(q(5))) + sin(q(4))*sin(q(5)))) + qd(1)*(cos((q(3)+pi/2))*(-(cos(q(5))*sin(q(4))) - cos(q(4))*sin(q(5))) + sin((q(3)+pi/2))*(-(cos(q(4))*cos(q(5))) + sin(q(4))*sin(q(5)))) + qd(3)*(qd(1)*(-(sin((q(3)+pi/2))*(-(cos(q(5))*sin(q(4))) - cos(q(4))*sin(q(5)))) + cos((q(3)+pi/2))*(-(cos(q(4))*cos(q(5))) + sin(q(4))*sin(q(5)))) + qd(2)*(-(cos((q(3)+pi/2))*(-(cos(q(5))*sin(q(4))) - cos(q(4))*sin(q(5)))) - sin((q(3)+pi/2))*(-(cos(q(4))*cos(q(5))) + sin(q(4))*sin(q(5))))) + qd(4)*(qd(1)*(sin((q(3)+pi/2))*(cos(q(5))*sin(q(4)) + cos(q(4))*sin(q(5))) + cos((q(3)+pi/2))*(-(cos(q(4))*cos(q(5))) + sin(q(4))*sin(q(5)))) + qd(2)*(cos((q(3)+pi/2))*(cos(q(5))*sin(q(4)) + cos(q(4))*sin(q(5))) - sin((q(3)+pi/2))*(-(cos(q(4))*cos(q(5))) + sin(q(4))*sin(q(5)))) + qd(3)*((-(cos(q(5))*sin(q(4))) - cos(q(4))*sin(q(5)))*(-(((2*(1 - cos(q(5))))/5 + (4*cos(q(5)))/5)*sin(q(4))) - (2*cos(q(4))*sin(q(5)))/5) + (-(cos(q(5))*sin(q(4))) - cos(q(4))*sin(q(5)))*(((2*(1 - cos(q(5))))/5 + (4*cos(q(5)))/5)*sin(q(4)) + (2*cos(q(4))*sin(q(5)))/5) + (cos(q(4))*cos(q(5)) - sin(q(4))*sin(q(5)))*(cos(q(4))*((2*(1 - cos(q(5))))/5 + (4*cos(q(5)))/5) - (2*sin(q(4))*sin(q(5)))/5) + (cos(q(4))*((2*(1 - cos(q(5))))/5 + (4*cos(q(5)))/5) - (2*sin(q(4))*sin(q(5)))/5)*(-(cos(q(4))*cos(q(5))) + sin(q(4))*sin(q(5))))) + qd(5)*(qd(4)*(-(((2*(1 - cos(q(5))))/5 + (4*cos(q(5)))/5)*cos(q(5))) + (2*cos(q(5))^2)/5) + qd(1)*(sin((q(3)+pi/2))*(cos(q(5))*sin(q(4)) + cos(q(4))*sin(q(5))) + cos((q(3)+pi/2))*(-(cos(q(4))*cos(q(5))) + sin(q(4))*sin(q(5)))) + qd(2)*(cos((q(3)+pi/2))*(cos(q(5))*sin(q(4)) + cos(q(4))*sin(q(5))) - sin((q(3)+pi/2))*(-(cos(q(4))*cos(q(5))) + sin(q(4))*sin(q(5)))) + qd(3)*((-(cos(q(5))*sin(q(4))) - cos(q(4))*sin(q(5)))*((-2*cos(q(5))*sin(q(4)))/5 - (2*cos(q(4))*sin(q(5)))/5) + (-(cos(q(5))*sin(q(4))) - cos(q(4))*sin(q(5)))*(((2*(1 - cos(q(5))))/5 + (4*cos(q(5)))/5)*sin(q(4)) + (2*cos(q(4))*sin(q(5)))/5) + (cos(q(4))*cos(q(5)) - sin(q(4))*sin(q(5)))*((2*cos(q(4))*cos(q(5)))/5 - (2*sin(q(4))*sin(q(5)))/5) + (cos(q(4))*((2*(1 - cos(q(5))))/5 + (4*cos(q(5)))/5) - (2*sin(q(4))*sin(q(5)))/5)*(-(cos(q(4))*cos(q(5))) + sin(q(4))*sin(q(5)))))];

                    obj.Functions.Dynamics.Constraints = Function('f', { [ q ; qd ; qdd ], pstvar } , { holonomic_constraints } );

                end



                % inertia matrix needed for impact
                obj.Functions.H_matrix = Function('f', { q } , { H } );


                obj.Functions.C_terms = Function('f', { [q ; qd ]} , { C } );

                % contact jacobian for impact
                obj.Functions.J_contact = Function('f', { q } , { Jc } );

                obj.Functions.Jdot_contact = Function('f', { [ q ; qd ] } , { Jcd } );

                obj.Functions.eta = Function('f', { q } , { rbm.eta } );
                obj.Functions.etadot = Function('f', { [q ; qd]} , { rbm.etadot } );


        
        end
        

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



switch rbm.dynamics

    case 'hybrid'

        [q_minus, qd_minus] = Model.get_x_minus( rbm );
        obj.Functions.q_minus  = Function('f', { q } , { q_minus } );     
        obj.Functions.qd_minus = Function('f', { q , qd } , { qd_minus } );  
        
end






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