function [obj, du_k] = add_slew_rate(obj, rbm, idxArg )

idx = idxArg{2};



if obj.Problem.slew_rate.bool
     
    % number of inputs
    NU = size(rbm.model.B,2);
   
    du_lb = obj.Problem.slew_rate.du_lb;
    du_ub = obj.Problem.slew_rate.du_ub;
    
    % du_guess = obj.Seed.du(:,idx+1);
    
%     if isfield(seed,'du')
%         du_k_guess = seed.du(:,idx+1);
%     else
%         du_k_guess = zeros(size(rbm.model.B,2),1);
%     end

%     du_k_guess = seed.du(:,idx+1);

    if isfield(obj.Seed,'du')
        du_guess = obj.Seed.du(:,idx+1);
    else
        du_guess = zeros(NU,1);
    end


    [obj, du_k] = add_var(obj, ['dU_' num2str(idx)] , NU , du_guess , du_lb , du_ub , 'du' );

    
else
    
    du_k = [];
        
end


end

