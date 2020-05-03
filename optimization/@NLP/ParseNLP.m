function [obj] = ParseNLP(obj, rbm)


%% Argument Validation
arguments
    obj (1,1) NLP
    rbm (1,1) DynamicalSystem
end


%% Begin NLP Formulation
import casadi.* 


%% Declare grid of NLP variables for states and control

var_grid = struct();

[obj, var_grid] = AddStates(obj, rbm, var_grid);

[obj, var_grid] = AddControl(obj, rbm, var_grid);

[obj, var_grid] = AddContactForces(obj, rbm, var_grid);

[obj, var_grid] = AddVCParameters(obj, rbm, var_grid);

[obj, var_grid] = AddFinalTime(obj, rbm, var_grid);

[obj, var_grid] = AddPhaseVariable(obj, rbm, var_grid);

[obj] = EnforceDynamics(obj, rbm, var_grid);

[obj] = EnforceZDInvariance(obj, rbm, var_grid);

[obj] = StepCharacteristics(obj, rbm, var_grid);

[obj] = FootHeight(obj, rbm, var_grid);

[obj] = AbsTorsoAngles(obj, rbm, var_grid);

[obj] = InitialConditions(obj, rbm, var_grid);

[obj] = FinalConditions(obj, rbm, var_grid);

[obj] = StanceFoot(obj, rbm, var_grid);

[obj] = OneStepPeriodic(obj, rbm, var_grid);

[obj] = SwingFootImpactVelocities(obj, rbm, var_grid);



%% Parse Trajectory

% unscaled time horizon
T = 1;

% length of finite elements
h = T/obj.Settings.nfe;

% initialize the running cost
running_cost = 0;

% integrate over each finite element
for idx_fe = 1:obj.Settings.nfe
    
    
    
    
    %idx_fe
    
    switch obj.Settings.CollocationScheme
        
        case 'Trapezoidal'
         
            %[idx_fe, idx_fe+1]
            
            idx1 = idx_fe;
            idx2 = idx_fe+1;
            
            x1 = [var_grid.(['pos_', num2str(idx1)]); var_grid.(['vel_', num2str(idx1)])];
            x2 = [var_grid.(['pos_', num2str(idx2)]); var_grid.(['vel_', num2str(idx2)])];
            
            f1 = var_grid.tf*[var_grid.(['vel_', num2str(idx1)]); var_grid.(['acc_', num2str(idx1)])];
            f2 = var_grid.tf*[var_grid.(['vel_', num2str(idx2)]); var_grid.(['acc_', num2str(idx2)])];
            
            [obj] = AddConstraint(obj, x2 - (x1 + 1/2*h*(f2 + f1)), -obj.Settings.ConstraintTolerance*ones(2*numel(rbm.States.q.sym),1), obj.Settings.ConstraintTolerance*ones(2*numel(rbm.States.q.sym),1), 'Trap');


            % enforce slew rate
            if obj.Problem.SlewRate.Bool
                
                u1 = var_grid.(['input_', num2str(idx1)]);
                u2 = var_grid.(['input_', num2str(idx2)]);
                
                du1 = var_grid.tf*var_grid.(['der_input_', num2str(idx1)]);
                du2 = var_grid.tf*var_grid.(['der_input_', num2str(idx2)]);
                
                [obj] = AddConstraint(obj, u2 - (u1 + 1/2*h*(du2 + du1)), -obj.Settings.ConstraintTolerance*ones(numel(rbm.Inputs.u.sym),1), obj.Settings.ConstraintTolerance*ones(numel(rbm.Inputs.u.sym),1), 'Slew rate (Int)');
            
            end
            
            
            
            
            
            % integrate cost
            cost1 = Opt.RunningCost(rbm, var_grid.(['pos_', num2str(idx1)]), var_grid.(['vel_', num2str(idx1)]), var_grid.(['acc_', num2str(idx1)]), var_grid.(['input_', num2str(idx1)]));
            cost2 = Opt.RunningCost(rbm, var_grid.(['pos_', num2str(idx2)]), var_grid.(['vel_', num2str(idx2)]), var_grid.(['acc_', num2str(idx2)]), var_grid.(['input_', num2str(idx2)]));
            running_cost = running_cost + 1/2*h*(cost1 + cost2);  

            
            
            
            
        case 'HermiteSimpson'
            
            %[2*idx_fe-1, 2*idx_fe, 2*idx_fe+1]

            idx1 = 2*idx_fe-1;
            idx2 = 2*idx_fe;
            idx3 = 2*idx_fe+1;
            
            x1 = [var_grid.(['pos_', num2str(idx1)]); var_grid.(['vel_', num2str(idx1)])];
            x2 = [var_grid.(['pos_', num2str(idx2)]); var_grid.(['vel_', num2str(idx2)])];
            x3 = [var_grid.(['pos_', num2str(idx3)]); var_grid.(['vel_', num2str(idx3)])];
            
            f1 = var_grid.tf*[var_grid.(['vel_', num2str(idx1)]); var_grid.(['acc_', num2str(idx1)])];
            f2 = var_grid.tf*[var_grid.(['vel_', num2str(idx2)]); var_grid.(['acc_', num2str(idx2)])];
            f3 = var_grid.tf*[var_grid.(['vel_', num2str(idx3)]); var_grid.(['acc_', num2str(idx3)])];
            
            
            % (4.3)                       
            [obj] = AddConstraint(obj, x3 - x1 - h/6*(f1 + 4*f2 + f3), -obj.Settings.ConstraintTolerance*ones(2*numel(rbm.States.q.sym),1), obj.Settings.ConstraintTolerance*ones(2*numel(rbm.States.q.sym),1), 'HermiteS (1)');
            
            % (4.4)
            [obj] = AddConstraint(obj, x2 - 1/2*(x1 + x3) - h/8*(f1 - f3), -obj.Settings.ConstraintTolerance*ones(2*numel(rbm.States.q.sym),1), obj.Settings.ConstraintTolerance*ones(2*numel(rbm.States.q.sym),1), 'HermiteS (1)');

            
            % integrate cost
            cost1 = Opt.RunningCost(rbm, var_grid.(['pos_', num2str(idx1)]), var_grid.(['vel_', num2str(idx1)]), var_grid.(['acc_', num2str(idx1)]), var_grid.(['input_', num2str(idx1)]));
            cost2 = Opt.RunningCost(rbm, var_grid.(['pos_', num2str(idx2)]), var_grid.(['vel_', num2str(idx2)]), var_grid.(['acc_', num2str(idx2)]), var_grid.(['input_', num2str(idx2)]));
            cost3 = Opt.RunningCost(rbm, var_grid.(['pos_', num2str(idx3)]), var_grid.(['vel_', num2str(idx3)]), var_grid.(['acc_', num2str(idx3)]), var_grid.(['input_', num2str(idx3)]));
            running_cost = running_cost + h/6*(cost1 + 4*cost2 + cost3);


            % enforce slew rate
            if obj.Problem.SlewRate.Bool
                
                u1 = var_grid.(['input_', num2str(idx1)]);
                u2 = var_grid.(['input_', num2str(idx2)]);
                u3 = var_grid.(['input_', num2str(idx3)]);
                
                du1 = var_grid.tf*var_grid.(['der_input_', num2str(idx1)]);
                du2 = var_grid.tf*var_grid.(['der_input_', num2str(idx2)]);
                du3 = var_grid.tf*var_grid.(['der_input_', num2str(idx3)]);
                
            
                % (4.3)                       
                [obj] = AddConstraint(obj, u3 - u1 - h/6*(du1 + 4*du2 + du3), -obj.Settings.ConstraintTolerance*ones(numel(rbm.Inputs.u.sym),1), obj.Settings.ConstraintTolerance*ones(numel(rbm.Inputs.u.sym),1), 'Slew rate (Int1)');
            
                % (4.4)
                [obj] = AddConstraint(obj, u2 - 1/2*(u1 + u3) - h/8*(du1 - du3), -obj.Settings.ConstraintTolerance*ones(numel(rbm.Inputs.u.sym),1), obj.Settings.ConstraintTolerance*ones(numel(rbm.Inputs.u.sym),1), 'Slew rate (Int2)');

            
            end
            
            
        otherwise
            error('Not supported yet')
            
    end
    
    
    
end



%% Wrap Up NLP

% add final cost
total_cost = Opt.FinalCost(rbm, running_cost, var_grid.tf, var_grid.(['pos_', num2str(idx1)]), var_grid.(['vel_', num2str(idx1)]), var_grid.(['acc_', num2str(idx1)]), var_grid.(['input_', num2str(idx1)]));
            
% set NLP objective 
obj.Cost = total_cost;



end
