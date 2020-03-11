function [obj] = enforce_periodicity(obj, rbm, x0, xPlus )
% x0 are the initial conditions
% xPlus are the values after impact


        

        [obj,pos2per,vel2per] = Opt.periodicity_constraints(obj,rbm, x0, xPlus);%(rbm, x_0, x_T );


        NB = rbm.model.NB;

        q_0  = x0(1:NB);
        qd_0 = x0(NB+1:2*NB);

        q_plus  = xPlus(1:NB);
        qd_plus = xPlus(NB+1:2*NB);

        %tol_per = 1E-12;


        obj = add_constraint(obj, qd_0(vel2per) - qd_plus(vel2per) , -obj.Settings.constraint_tol*ones(length(vel2per),1) , obj.Settings.constraint_tol*ones(length(vel2per),1) , 'velocity periodicity' );     

        obj = add_constraint(obj, q_0(pos2per) - q_plus(pos2per) , -obj.Settings.constraint_tol*ones(length(pos2per),1) , obj.Settings.constraint_tol*ones(length(pos2per),1) , 'position periodicity' );     

        % qd_0(pos2per) - qd_plus(pos2per)
        % 
        % % periodicity constraints
        % [problem] = Optim.add_constraint(problem, q_plus(3:7) - q_0(3:7) , -tol_per*ones(5,1), tol_per*ones(5,1) );
        % [problem] = Optim.add_constraint(problem, qd_plus - qd_0 , -tol_per*ones(NB,1), tol_per*ones(NB,1) );



  

end