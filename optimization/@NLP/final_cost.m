%function [obj] = final_cost(obj, rbm, q, qd, qdd, u)
% function running_cost = final_cost(obj, rbm, running_cost, q, qd, qdd, u, ud, tf)
function running_cost = final_cost(obj, rbm, running_cost, states, control, tf)%q, qd, qdd, u, ud, tf)

% x0 are the initial conditions
% xPlus are the values after impact

% this can be used to divide by step length for the cost of transport 

% or to multiply by tf for scaling 

running_cost = Opt.final_cost(obj, rbm, running_cost, states, control, tf);%q, qd, qdd, u, ud, tf);

%obj.Cost = Opt.final_cost( obj, rbm, q, qd, qdd, u );




end