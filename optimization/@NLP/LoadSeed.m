% function [casadi] = configure_functions( rbm , nlp )
function [nlp, rbm] = LoadSeed(nlp, rbm, varargin)

arguments
    nlp (1,1) NLP
    rbm (1,1) DynamicalSystem
end

arguments (Repeating)
   varargin (1,:) char 
end

import casadi.*

ncp = nlp.Settings.ncp;

if nargin == 2
    
    
    [q_seed, qd_seed, qdd_seed, t_seed] = GenManualSeed(nlp, rbm);

    seed_data.q = q_seed;
    seed_data.qd = qd_seed;
    seed_data.qdd = qdd_seed;
    seed_data.t = t_seed;
    
else
    
    seed_name = varargin{1};
    
    if ~strcmp(seed_name(end-3:end), '.mat')
        error('The seed must be provided as a .mat file')
    end
    
    seed_data = load(seed_name);
    fdnames = fieldnames(seed_data);
    
    if numel(fdnames) >= 2
       warning('Only looking for first field in seed') 
    end
    
    seed_data = seed_data.(fdnames{1});
    
    
    %% FIRST VERIFY THE FIELDS EXIST
    if isfield(seed_data, 'q')
        validateattributes(seed_data.q, {'double'}, {'size',[numel(rbm.States.q.sym),NaN]})
    else
        error('''q'' is a required seed argument.');
    end
    
    if isfield(seed_data, 't')
        validateattributes(seed_data.t, {'double'}, {'size',[1,NaN]})
    else
        error('''t'' is a required seed argument.');
    end
    
    if isfield(seed_data, 'qd')
        validateattributes(seed_data.qd, {'double'}, {'size',[numel(rbm.States.q.sym),NaN]})
    else
        for i = 1:numel(rbm.States.q.sym)
            seed_data.qd(i,:) = gradient(seed_data.q(i,:),seed_data.t);
        end
    end
    
    if isfield(seed_data, 'qdd')
        validateattributes(seed_data.qdd, {'double'}, {'size',[numel(rbm.States.q.sym),NaN]})
    else
        for i = 1:numel(rbm.States.q.sym)
            seed_data.qdd(i,:) = gradient(seed_data.qd(i,:),seed_data.t);
        end
    end
    
end



%% COMPLETE MISSING FIELDS
Fc_flag = 0;
for i = 1:numel(rbm.Contacts)
    if isfield(seed_data, ['Fc_', num2str(i)])
        validateattributes(seed_data.(['Fc_', num2str(i)]), {'double'}, {'size',[numel(rbm.Contacts{i}.Fc.sym),NaN]})
        Fc_flag = Fc_flag+1;
    else
        if Fc_flag ~= 0
            warning('Only certain contact wrenches are initialized. Verify that Seed.Fc_i corresponds to the correct contact i.')
        end
        seed_data.(['Fc_', num2str(i)]) = zeros(numel(rbm.Contacts{i}.Fc.sym), ncp);
    end
end


if isfield(seed_data, 'u')
    validateattributes(seed_data.u, {'double'}, {'size',[numel(rbm.Inputs.u.sym),NaN]})
else

    % open-loop (no virtual constraints defined)
    sumContactForces = rbm.States.q.sym(1)*zeros(numel(rbm.States.q.sym,1));
    symarg2usym = {rbm.States.q.sym, rbm.States.dq.sym, rbm.States.ddq.sym};
    for i = 1:numel(rbm.Contacts)
        sumContactForces = sumContactForces + rbm.Contacts{i}.Jac_contact'*rbm.Contacts{i}.Fc.sym;
        symarg2usym{end+1} = rbm.Contacts{i}.Fc.sym;
    end
    uSym = rbm.InputMap'*(rbm.Dynamics.H_matrix*rbm.States.ddq.sym + rbm.Dynamics.C_terms - sumContactForces);

    uFun = Function('f', symarg2usym, {uSym});

    for k = 1:size(seed_data.q,2)
        numarg2ufun = {seed_data.q(:,k), seed_data.qd(:,k), seed_data.qdd(:,k)};

        for j = 1:numel(rbm.Contacts)
            tempArg = seed_data.(['Fc_', num2str(j)]);
            numarg2ufun{end+1} = tempArg(:,j);
        end
        seed_data.u(:,k) = full(uFun(numarg2ufun{:}));
    end

end


if isfield(seed_data, 'du')
    validateattributes(seed_data.du, {'double'}, {'size',[numel(rbm.Inputs.u.sym),NaN]})
else
    for k = 1:numel(rbm.Inputs.u.sym)
        seed_data.du(k,:) = gradient(seed_data.u(k,:), seed_data.t);
    end
end



if nlp.Problem.Trajectory.Bool

    if isfield(seed_data, 'a')
        validateattributes(seed_data.a, {'double'}, {'size',[numel(rbm.Inputs.u.sym),NaN]})

        if size(seed_data.a,2) ~= nlp.Problem.Trajectory.PolyOrder+1
            seed_data.a = InitializeBezier(nlp, rbm, [seed_data.q; seed_data.qd], seed_data.t);
        end

    else
        seed_data.a = InitializeBezier(nlp, rbm, [seed_data.q; seed_data.qd], seed_data.t);

    end


end




    



%% SIZE OF THE FIELDS
if size(seed_data.t,2) ~= ncp
    tTemp = seed_data.t;
    seed_data.t = linspace(seed_data.t(1), seed_data.t(end), ncp);
    
    
    
    

    qTemp = zeros(numel(rbm.States.q.sym),ncp);
    qdTemp = zeros(numel(rbm.States.q.sym),ncp);
    qddTemp = zeros(numel(rbm.States.q.sym),ncp);
    
    for i = 1:numel(rbm.States.q.sym)
        qTemp(i,:) = interp1(tTemp, seed_data.q(i,:), seed_data.t);
        qdTemp(i,:) = interp1(tTemp, seed_data.qd(i,:), seed_data.t);
        qddTemp(i,:) = interp1(tTemp, seed_data.qdd(i,:), seed_data.t);
    end
    
    uTemp = zeros(numel(rbm.Inputs.u.sym),ncp);
    duTemp = zeros(numel(rbm.Inputs.u.sym),ncp);
    for i = 1:numel(rbm.Inputs.u.sym)
        uTemp(i,:) = interp1(tTemp, seed_data.u(i,:), seed_data.t);
        duTemp(i,:) = interp1(tTemp, seed_data.du(i,:), seed_data.t);
    end
    
    for i = 1:numel(rbm.Contacts)
        tempArg = seed_data.(['Fc_', num2str(i)]);
        FTemp = zeros(numel(rbm.Contacts{i}.Fc.sym),ncp);
        for k = 1:numel(rbm.Contacts{i}.Fc.sym) 
            FTemp(k,:) = interp1(tTemp, tempArg(k,:), seed_data.t);
        end
        
        seed_data.(['Fc_', num2str(i)]) = FTemp;
    end
    

    seed_data.q = qTemp;
    seed_data.qd = qdTemp;
    seed_data.qdd = qddTemp;
    seed_data.u = uTemp;
    seed_data.du = duTemp;
end



%{
% By now they should all have the right size

% and I can check whether the initial guess is going to satisfy the primal
% feasibility

    check the initial guess (states only)
    returns warning if initial guess is outside feasible region
%}
%seed_data.du(2,2) = 1E6;
[nlp, rbm] = CheckFeasibility(nlp, rbm, seed_data);


end
