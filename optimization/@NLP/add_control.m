function [nlp , control] = add_control(nlp, rbm, idxArg)

% idx is the number of the collocation point
%   e.g. idx = 0 means t = 0

    idx = idxArg{2};

    %% CHECK THE ARGUMENTS FIELDS FO STRUCTURES


% % 
% %     if nlp.Problem.torque.bool
% %         u_lb = -nlp.Problem.torque.max*ones(size(rbm.model.B,2),1);
% %         u_ub = nlp.Problem.torque.max*ones(size(rbm.model.B,2),1);
% %     else
% %         u_lb = -Inf*ones(size(rbm.model.B,2),1);
% %         u_ub = Inf*ones(size(rbm.model.B,2),1);
% %     end
% % 
% %     u_k_guess = nlp.Seed.u(:,idx+1);
% %     
% %     
    
  
    
    
    
    



% if isfield(nlp.Seed,'u_lb') && isfield(nlp.Seed,'u_ub')
%     u_lb = nlp.Seed.u_lb(:,idx+1);
%     u_ub = nlp.Seed.u_ub(:,idx+1);
% else
%     u_lb = -1000*ones(size(rbm.model.B,2),1);
%     u_ub =  1000*ones(size(rbm.model.B,2),1);
% end


% number of inputs
NU = size(rbm.model.B,2);

if nlp.Problem.torque.bool
    u_lb = nlp.Problem.torque.u_lb;
    u_ub = nlp.Problem.torque.u_ub;
else
    u_lb = -Inf*ones(NU,1);
    u_ub = Inf*ones(NU,1);
end
    
if isfield(nlp.Seed,'u')
    u_guess = nlp.Seed.u(:,idx+1);
else
    u_guess = zeros(NU,1);
end

 
    
 
%     if isfield(seed,'u')
%         u_k_guess = seed.u(:,idx+1);
%     else
%         u_k_guess = zeros(size(rbm.model.B,2),1);
%     end
    
[nlp, u_k] = add_var( nlp , ['U_' num2str(idx)] , NU , u_guess , u_lb , u_ub , 'u' );

%     [problem, u_k, u_inds(end+1,:)] = Optim.add_var(problem, ['U_' num2str(idx)], size(rbm.model.B,2) , u_k_guess , u_lb , u_ub , ['u@CP' num2str(idx)] );
% 
% 
%    [problem, u_k, u_inds] = Optim.add_open_loop_control( nlp , rbm , problem , seed , idx , u_inds );
% 


 
% add slew rate of actuators at t = 0
[nlp, du_k] = add_slew_rate(nlp, rbm, {'idx',idx} );


control.u = u_k;
control.du = du_k;


end