function [nlp , grid_var] = AddContactForces(nlp, rbm, grid_var)


arguments
    nlp (1,1) NLP
    rbm (1,1) DynamicalSystem
    grid_var (1,1) struct
end

NC = numel(rbm.Contacts);

for i = 1:NC
    % idx is the number of the collocation point
    %   e.g. idx = 0 means t = 0
    NFc = numel(rbm.Contacts{i}.Fc.sym);

    for idx = 1:nlp.Settings.ncp

        Fc_ub = rbm.Contacts{i}.Fc.UpperBound(:, idx);
        Fc_lb = rbm.Contacts{i}.Fc.LowerBound(:, idx);
        Fc_guess = rbm.Contacts{i}.Fc.Seed(:, idx);

        [nlp, Fc_idx] = AddVar(nlp, ['Fc', num2str(i), '_', num2str(idx)], NFc, Fc_guess, Fc_lb, Fc_ub, ['Fc', num2str(i)]);
        grid_var.(['Fc', num2str(i),'_', num2str(idx)]) = Fc_idx;

        
        
        if rbm.Contacts{i}.Friction.bool
            
            [nlp] = EnforceFriction(nlp, rbm, rbm.Contacts{i}, grid_var.(['Fc', num2str(i),'_', num2str(idx)]));
            
        end
        
        
        
    end

    
    
    
    
end




 

end




