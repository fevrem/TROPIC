function [obj,q_minus,qd_plus] = apply_impact(obj, rbm, q_minus, qd_minus)
%{
    The argument passed as q- and qd- are based on what is running.
    Optimization: 
        q_minus  = q_minus_flipped
        qd_minus = qd_minus_flipped 
    Simulation: 
        q_minus  = q_minus
        qd_minus = qd_minus
%}

% q_minus
% qd_minus
% 
% 
%     if nargin < 4
%         error('Not enough input arguments')
%     end
% 
%     NB = RoBoDyn.model.NB;
% 
%     if strcmpi(sym_type,'numeric') || strcmpi(sym_type,'matlab')
% 
%         [arg2imp(1:NB).val] = numToCell( q_minus );
%         H_impact = H_matrix(arg2imp.val);
%         J_impact = Jc(arg2imp.val);
% 
%         
%     elseif strcmpi(sym_type,'casadi')

        
%         casadi = varargin{1};

        NB = rbm.model.NB;

        % inertia matrix needed for impact
        H = obj.Functions.H_matrix( q_minus );

        % contact jacobian for impact
        J = obj.Functions.J_contact( q_minus );


%     end
    
        if 0
            
            delta_dq = eye(NB) - (H\J')/((J/H)*J')*J;
            
            qd_plus = delta_dq * qd_minus;
            
        else % implicitly

            LHS = [ H , -J' ; J , zeros(size(J,1)) ];
            RHS = [ H*qd_minus ; zeros(size(J,1),1) ];
            
            
            
            
            
            
            %[ ~ , ~ , qd_lb , qd_ub , ~ , ~ ] = Opt.bound_vars( rbm.model, obj.Seed.q(:,1) );

     

            
            
            
if isfield(obj.Seed,'qd')
    qd_guess = obj.Seed.qd(:,1);
else
    qd_guess = zeros(NB,1);
end



% if isfield(obj.Seed,'qd_lb') && isfield(obj.Seed,'qd_ub')
%     qd_lb = obj.Seed.qd_lb(:,1);
%     qd_ub = obj.Seed.qd_ub(:,1);
% else
%     [ ~ , ~ , qd_lb , qd_ub , ~ , ~ ] = Opt.bound_vars( rbm.model, obj.Seed.q(:,1) );
% end


qd_lb = obj.Problem.states.qd_lb;
qd_ub = obj.Problem.states.qd_ub;


% xPlusCont 
% -> xPlus = x0

% xMinusCont
% -> xMinus swapped 

            
%
%
%FROST MATCH
[obj, q_plus] = add_var( obj , 'QPlus' , NB , zeros(NB,1) , -Inf*ones(NB,1), Inf*ones(NB,1) , 'q_plus' );
%[obj, qd_plus] = add_var( obj , 'dQPlus' , NB , qd_guess , qd_lb , qd_ub , 'qd_plus' );
[obj, qd_plus] = add_var( obj , 'dQPlus' , NB , zeros(NB,1) , -Inf*ones(NB,1), Inf*ones(NB,1) , 'qd_plus' );
%[obj, f_imp] = add_var( obj , 'fc_imp' , size(J,1) , 100*ones(size(J,1),1) , -1000*ones(size(J,1),1) , 1000*ones(size(J,1),1) , 'fc_imp' );
[obj, f_imp] = add_var( obj , 'fc_imp' , size(J,1) , zeros(size(J,1),1) , -Inf*ones(size(J,1),1) , Inf*ones(size(J,1),1) , 'fc_imp' );


obj = add_constraint(obj, q_plus - q_minus , -obj.Settings.constraint_tol*ones(NB+size(J,1),1) , obj.Settings.constraint_tol*ones(NB+size(J,1),1) , 'impact' );     
% obj = add_constraint(obj, LHS*[qd_plus;f_imp] - RHS , -obj.Settings.constraint_tol*ones(NB+size(J,1),1) , obj.Settings.constraint_tol*ones(NB+size(J,1),1) , 'impact' );     
obj = add_constraint(obj, LHS*[qd_plus;f_imp] - RHS , -obj.Settings.constraint_tol*ones(NB+size(J,1),1) , obj.Settings.constraint_tol*ones(NB+size(J,1),1) , 'impact' );     
%
%

            
            
%[obj, dq_k] = add_var( obj , ['dQ_' num2str(idx)] , rbm.model.NB , obj.Seed.qd(:,idx+1) , qd_lb , qd_ub , 'qd' );



            %[obj, f_imp] = add_var( obj , 'fc_imp' , size(J,1) , zeros(size(J,1),1) , -Inf*ones(size(J,1),1) , Inf*ones(size(J,1),1) , 'fc_imp' );

            
            
 
            
            
            
        end

    
    % post impact states
    %q_plus  = q_minus;
    
    

end

