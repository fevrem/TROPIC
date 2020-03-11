function [nlp,Fc_k] = add_contact_force(nlp,rbm,idxArg)


    idx = idxArg{2};

%     % bound the contact 'states' automatically 
%     [Fc_lb,Fc_ub] = Opt.bounds_Fc( nlp );
% 
%     Fc_k_guess = nlp.Seed.Fc(:,idx+1);
%     
%     
    




% if isfield(nlp.Seed,'Fc')
%     Fc_guess = nlp.Seed.Fc(:,idx+1);
% else
%     Fc_guess = zeros(size(rbm.J_contact,1),1);
% end
% 
% if isfield(nlp.Seed,'Fc_lb') && isfield(nlp.Seed,'Fc_ub')
%     Fc_lb = nlp.Seed.Fc_lb(:,idx+1);
%     Fc_ub = nlp.Seed.Fc_ub(:,idx+1);
% else
%     Fc_lb = -Inf * ones(size(rbm.J_contact,1),1);
%     Fc_ub = Inf * ones(size(rbm.J_contact,1),1);
% end
% 
% if isfield(nlp.Problem,'normal_force')
%     if nlp.Problem.normal_force.bool
%         switch rbm.dimensions
%             case '2D'
%                 %
%                 %Fc_lb(2) = 10;%nlp.Problem.normal_force.min;
%                 %
%             case '3D'
%                 error('need to fix this')
%                 
%         end
%         
%     end
% end





% number of contact forces
NF = size(rbm.J_contact,1)

Fc_lb = nlp.Problem.contact.forces.Fc_lb;
Fc_ub = nlp.Problem.contact.forces.Fc_ub;

if isfield(nlp.Seed,'Fc')
    Fc_guess = nlp.Seed.Fc(:,idx+1);
else
    Fc_guess = zeros(NF,1);
end




% if nlp.Problem.normal_force.bool
%     Fc_lb(2) = nlp.Problem.normal_force.min;
% end      
%         
        

    
%     if isfield(seed,'Fc')
%         Fc_k_guess = seed.Fc(:,idx+1);
%     else
%         Fc_k_guess = zeros(size(rbm.J_contact,1),1);
%     end
    
    [nlp, Fc_k] = add_var(nlp, ['Fc_' num2str(idx)] , NF , Fc_guess , Fc_lb , Fc_ub , 'Fc' );

%     [problem, Fc_k, Fc_inds(end+1,:)] = Optim.add_var(problem, ['Fc_' num2str(idx)] , size(rbm.J_contact,1) , Fc_k_guess , Fc_lb , Fc_ub , ['Fc@CP' num2str(idx)] );


end