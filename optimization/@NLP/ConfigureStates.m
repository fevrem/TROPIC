function vars = ConfigureStates(obj, rbm, vars)
    arguments
        obj (1,1) NLP
        rbm (1,1) DynamicalSystem
        vars
    end

    
    
    
    
    vars.pos = rbm.States.q;
    vars.vel = rbm.States.dq;
    vars.acc = rbm.States.ddq;
    vars.input = rbm.Inputs.u;
    
    
    nc = size(rbm.Contacts, 2);

    if nc ~= 0
        vars.cForce = rbm.Contacts{1}.Fc;
        for i = 2:nc


            vars.cForce.sym = [vars.cForce.sym; rbm.Contacts{i}.Fc.sym];
            vars.cForce.UpperBound = [vars.cForce.UpperBound; rbm.Contacts{i}.Fc.UpperBound];
            vars.cForce.LowerBound = [vars.cForce.LowerBound; rbm.Contacts{i}.Fc.LowerBound];
            vars.cForce.Seed = [vars.cForce.Seed; rbm.Contacts{i}.Fc.Seed];
            
            if ~strcmp(vars.cForce.ID, rbm.Contacts{i}.Fc.ID)
               error('Same ID reference for all contact forces in each phase')
            end
            
            
            
        end
    end

    
    
end