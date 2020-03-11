function [obj] = enforce_dynamics(obj, rbm, states, control, idxArg )
% idx is the number of the collocation point
%   e.g. idx = 0 means t = 0

idx = idxArg{2};

% if nargin ~= 11
%     error('wrong number of input arguments')
% end
% 
NB = rbm.model.NB;


% evaluate dynamics at t = 0
% % f_k = tf*dx;

% enforce continuity 
%warning('may neeed this')
% Need this??
%
%[problem] = Optim.add_constraint(problem, dx(1:NB) - x(NB+1:2*NB) , -1E-9*ones(NB,1) , 1E-9*ones(NB,1) );


switch rbm.dynamics

    case 'continuous'

        %error('fill this')
        
%         % enforce equations of motion implicitly
%         [problem] = Optim.add_constraint(problem, fun.implicit_dynamics( [ x ; dx(NB+1:2*NB) ] , u ) , -1E-9*ones(NB,1) , 1E-9*ones(NB,1) ); 
% 
%         obj.Functions.Dynamics.Equations = Function('f',{ [ q ; qd ; qdd ] , u } , { second_order_ode } );
% 
% 
%         Fc_inds = [];
%         Fc_k = [];




        % enforce equations of motion implicitly
        obj = add_constraint(obj, obj.Functions.Dynamics.Equations([states.pos; states.vel; states.acc], control.u) , -obj.Settings.constraint_tol*ones(NB,1) , obj.Settings.constraint_tol*ones(NB,1) , 'dynamics (EOM)' );     



    case 'hybrid'        

%         % contact wrench as decision variable
%         if idx == 0  
%             Fc_inds = [];
%         end

if 0 %FROST MATCH
        [obj, Fc] = add_contact_force(obj, rbm, idxArg);

        % enforce friction constraints  
        [obj] = enforce_friction(obj, rbm, Fc );        
        
        % enforce equations of motion implicitly
        obj = add_constraint(obj, obj.Functions.Dynamics.Equations([states.pos; states.vel; states.acc], control.u, Fc) , -obj.Settings.constraint_tol*ones(NB,1) , obj.Settings.constraint_tol*ones(NB,1) , 'dynamics (EOM)' );     

        % number of holonomic constraints
        NHC = size(rbm.J_contact,1);
        
        % enforce constraint equations
        obj = add_constraint(obj, obj.Functions.Dynamics.Constraints([states.pos; states.vel; states.acc]) , -obj.Settings.constraint_tol*ones(NHC,1) , obj.Settings.constraint_tol*ones(NHC,1) , 'dynamics (HC)' );     
end

        
end



        



end


