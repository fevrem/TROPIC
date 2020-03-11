function [nlp] = EnforceDynamics(nlp, rbm, grid_var)


arguments
    nlp (1,1) NLP
    rbm (1,1) DynamicalSystem
    grid_var (1,1) struct
end
    

NB = numel(rbm.States.q.sym);


nContact = size(rbm.Contacts, 2);

% number of holonomic constraints
% number of holonomic constraints
%NHC = size(rbm.J_contact,1);
        
NH = 0;
for i = 1:nContact
    NH = NH + size(rbm.Contacts{i}.Jac_contact,1);
end

% idx is the number of the collocation point
%   e.g. idx = 0 means t = 0
for idx = 1:nlp.Settings.ncp
    
    vars_cp = {grid_var.(['pos_', num2str(idx)]),...
        grid_var.(['vel_', num2str(idx)]),...
        grid_var.(['acc_', num2str(idx)])};
    
    
    if nContact ~= 0
        nlp = add_constraint(nlp, nlp.Functions.ConstrainedDynamicsODE(vars_cp{:}), -nlp.Settings.ConstraintTolerance*ones(NH,1), nlp.Settings.ConstraintTolerance*ones(NH,1), 'dynamics (HC)');     
    end    
    
    
    
    vars_cp{end+1} = grid_var.(['input_', num2str(idx)]);
    
    
    for i = 1:nContact
        
        vars_cp{end+1} = grid_var.(['Fc', num2str(i), '_', num2str(idx)]);
        
    end
    

    % enforce equations of motion implicitly
    nlp = add_constraint(nlp, nlp.Functions.DynamicsODE(vars_cp{:}), -nlp.Settings.ConstraintTolerance*ones(NB,1), nlp.Settings.ConstraintTolerance*ones(NB,1), 'dynamics (EOM)');     

    



end




% 
% [obj, Fc] = add_contact_force(obj, rbm, idxArg);
% 
% % enforce friction constraints  
% [obj] = enforce_friction(obj, rbm, Fc );        







        



end


