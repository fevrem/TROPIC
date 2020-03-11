function [obj, grid_var] = AddPhaseVariable(obj, rbm, grid_var)%, T)

%% Argument Validation
arguments
    obj (1,1) NLP
    rbm (1,1) DynamicalSystem
    grid_var (1,1) struct
    %T (1,:) char {mustBeMember(T, {'t_minus','t_plus'})}
end


% if obj.Problem.Trajectory.Bool
% 
%     switch obj.Problem.Trajectory.PolyPhase
%         case 'time-based'    
%             p_var_0 = 0;
%             
%             p_var_T = grid_var.tf;
%             
%             for i = 1:obj.Settings.ncp
%                
%                 tau_i = (i-1)/(obj.Settings.ncp-1)
% 
%             end
%             
%             
%         case 'state-based'
%             
%             if 0
%                 p0_guess = rbm.Model.c*rbm.States.q.Seed(:,1);
%                 p0_lb = rbm.Model.c*rbm.States.q.LowerBound(:,1);
%                 p0_ub = rbm.Model.c*rbm.States.q.UpperBound(:,1);
%                 [obj, p_var_0] = add_var(obj, 'p0', 1, p0_guess, p0_lb, p0_ub, 'phase variable (1)');
%                 obj = add_constraint(obj, p_var_0 - rbm.Model.c*grid_var.pos_1, 0, 0, 'phase variable (1)');     
% 
%                 pT_guess = rbm.Model.c*rbm.States.q.Seed(:,end);
%                 pT_lb = rbm.Model.c*rbm.States.q.LowerBound(:,end);
%                 pT_ub = rbm.Model.c*rbm.States.q.UpperBound(:,end);
%                 [obj, p_var_T] = add_var(obj, 'pT', 1, pT_guess, pT_lb, pT_ub, 'phase variable (2)');
%                 obj = add_constraint(obj, p_var_T - rbm.Model.c*grid_var.(['pos_', num2str(obj.Settings.ncp)]), 0, 0, 'phase variable (2)');     
%             end
%             
%             p_var_0 = rbm.Model.c*grid_var.pos_1
% 
%             p_var_T = rbm.Model.c*grid_var.(['pos_', num2str(obj.Settings.ncp)])
%           
%             
%             for i = 1:obj.Settings.ncp
%                
%                 i
%                 
%                 p_var_i = rbm.Model.c*grid_var.(['pos_', num2str(i)])
%                 
%                 tau_i = (p_var_i - p_var_0)/(p_var_T - p_var_0)
%                 
% 
%             end
%             
%     end
% 
% end
% 
% 
% grid_var.phase_var_1 = p_var_0;
% grid_var.(['phase_var_', num2str(obj.Settings.ncp)]) = p_var_T;












%{
    Define 2 sets of phase variables (time and phase)
    Not as decision variables, but calculated variables
    Defining as decision variables should increase linearity but 
    p_var = c*q is very trivial
%}

%p_var_0 = 0;
%p_var_T = grid_var.tf;
for i = 1:obj.Settings.ncp
    tau_time_i = (i-1)/(obj.Settings.ncp-1);
    grid_var.(['tau_time_', num2str(i)]) = tau_time_i;
end
grid_var.p_time_plus = 0; % beginning of phase
grid_var.p_time_minus = grid_var.tf; % end of phase



p_var_0 = rbm.Model.c*grid_var.pos_1;
p_var_T = rbm.Model.c*grid_var.(['pos_', num2str(obj.Settings.ncp)]);
for i = 1:obj.Settings.ncp
    p_var_i = rbm.Model.c*grid_var.(['pos_', num2str(i)]);
    tau_phase_i = (p_var_i - p_var_0)/(p_var_T - p_var_0);
    grid_var.(['tau_phase_', num2str(i)]) = tau_phase_i;
end
grid_var.p_phase_plus = p_var_0; % beginning of phase
grid_var.p_phase_minus = p_var_T; % end of phase



% possible to define monotonically increasing quantity even if 
% time-based or open-loop are choosen 
if obj.Problem.PhaseVariableDerivative.Bool
    for i = 1:obj.Settings.ncp
        obj = add_constraint(obj, rbm.Model.c*grid_var.(['vel_', num2str(i)]), obj.Problem.PhaseVariableDerivative.LowerBound, obj.Problem.PhaseVariableDerivative.UpperBound, 'Monotonic phase variable');     
    end
end




end
