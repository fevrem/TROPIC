% function [ q_k , dq_k , ddq_k , q_inds , dq_inds , ddq_inds , problem ] = add_states( nlp , problem , rbm , seed , q_inds , dq_inds , ddq_inds , idx )
function [nlp, states] = add_states(nlp, rbm, idxArg )


% arguments
%     nlp (:,:) struct {}
%     problem (:,:) struct {}
%     rbm
%     seed (:,:) {mustBeNumeric}
%     q_inds
%     dq_inds
%     ddq_inds
%     idx
% end
    


% idx is the number of the collocation point
%   e.g. idx = 0 means t = 0


% if nargin ~= 8
%     error('wrong number of input arguments')
% end



idx = idxArg{2};

NB = rbm.model.NB;

    
% state variables (position and velocity) + acceleration


if isfield(nlp.Seed,'q')
    q_guess = nlp.Seed.q(:,idx+1);
else
    q_guess = zeros(NB,1);
    error('catch')
end

if isfield(nlp.Seed,'qd')
    qd_guess = nlp.Seed.qd(:,idx+1);
else
    qd_guess = zeros(NB,1);
    error('catch')
end

if isfield(nlp.Seed,'qdd')
    qdd_guess = nlp.Seed.qdd(:,idx+1);
else
    qdd_guess = zeros(NB,1);
    error('catch')
end


% if isfield(nlp.Seed,'q_lb') && isfield(nlp.Seed,'q_ub')
%     q_lb = nlp.Seed.q_lb(:,idx+1);
%     q_ub = nlp.Seed.q_ub(:,idx+1);
% else
%     [ q_lb , q_ub , ~ , ~ , ~ , ~ ] = Opt.bound_vars( rbm.model, nlp.Seed.q(:,1) );
% end
% 
% if isfield(nlp.Seed,'qd_lb') && isfield(nlp.Seed,'qd_ub')
%     qd_lb = nlp.Seed.qd_lb(:,idx+1);
%     qd_ub = nlp.Seed.qd_ub(:,idx+1);
% else
%     [ ~ , ~ , qd_lb , qd_ub , ~ , ~ ] = Opt.bound_vars( rbm.model, nlp.Seed.q(:,1) );
% end
% 
% if isfield(nlp.Seed,'qdd_lb') && isfield(nlp.Seed,'qdd_ub')
%     qdd_lb = nlp.Seed.qdd_lb(:,idx+1);
%     qdd_ub = nlp.Seed.qdd_ub(:,idx+1);
% else
%     [ ~ , ~ , ~ , ~ , qdd_lb , qdd_ub ] = Opt.bound_vars( rbm.model, nlp.Seed.q(:,1) );
% end





q_lb = nlp.Problem.states.q_lb;
q_ub = nlp.Problem.states.q_ub;

qd_lb = nlp.Problem.states.qd_lb;
qd_ub = nlp.Problem.states.qd_ub;

qdd_lb = nlp.Problem.states.qdd_lb;
qdd_ub = nlp.Problem.states.qdd_ub;



%[ q_lb , q_ub , qd_lb , qd_ub , qdd_lb , qdd_ub ] = Opt.bound_vars( rbm.model, nlp.Seed.q(:,1) );

    

[nlp, q_k] = add_var( nlp , ['Q_' num2str(idx)] , NB , q_guess , q_lb , q_ub , 'q' );
[nlp, dq_k] = add_var( nlp , ['dQ_' num2str(idx)] , NB , qd_guess , qd_lb , qd_ub , 'qd' );
[nlp, ddq_k] = add_var( nlp , ['ddQ_' num2str(idx)] , NB , qdd_guess , qdd_lb , qdd_ub , 'qdd' );



%[problem, q_k, q_inds(end+1,:)] = add_var(problem, ['Q_' num2str(idx)] , rbm.model.NB , seed.q(:,idx+1) , q_lb , q_ub , ['q@CP' num2str(idx) ] );
%[problem, dq_k, dq_inds(end+1,:)] = Optim.add_var(problem, ['DQ_' num2str(idx)] , rbm.model.NB , seed.qd(:,idx+1) , qd_lb , qd_ub , ['qd@CP' num2str(idx) ] );
%[problem, ddq_k, ddq_inds(end+1,:)] = Optim.add_var(problem, ['DDQ_' num2str(idx)] , rbm.model.NB , seed.qdd(:,idx+1) , qdd_lb , qdd_ub , ['qdd@CP' num2str(idx) ] );

states = struct();
states.pos = q_k;
states.vel = dq_k;
states.acc = ddq_k;


end