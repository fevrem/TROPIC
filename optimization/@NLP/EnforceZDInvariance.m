function [obj] = EnforceZDInvariance(obj, rbm, grid_var)


%% Argument Validation
arguments
    obj (1,1) NLP
    rbm (1,1) DynamicalSystem
    grid_var (1,1) struct
end


if obj.Problem.Trajectory.Bool

    switch obj.Problem.Trajectory.PolyType
        case 'Bezier'


            a = reshape(grid_var.a, numel(rbm.Inputs.u.sym), obj.Problem.Trajectory.PolyOrder+1);

            % Enforce hybrid invariance
            [obj] = EnforceHZDInvariance(obj, rbm, grid_var);

            
            % can change that but ideally gaits are robut to gains
            %ctrl_gain = 10;
            ctrl_gain = obj.Problem.Trajectory.ControlGain;
            
            for i = 1:obj.Settings.ncp

                switch obj.Problem.Trajectory.PolyPhase
                    case 'state-based'
                        tau_i = grid_var.(['tau_phase_', num2str(i)]);
                        t_plus = grid_var.p_phase_plus;
                        t_minus = grid_var.p_phase_minus;

                    case 'time-based'
                        tau_i = grid_var.(['tau_time_', num2str(i)]);
                        t_plus = grid_var.p_time_plus;
                        t_minus = grid_var.p_time_minus;
                        
                end
                  
                y_i = obj.Functions.Y_Controller(grid_var.(['pos_', num2str(i)]), a, tau_i, t_plus, t_minus);
                dy_i = obj.Functions.DY_Controller(grid_var.(['pos_', num2str(i)]), grid_var.(['vel_', num2str(i)]), a, tau_i, t_plus, t_minus);
                ddy_i = obj.Functions.DDY_Controller(grid_var.(['pos_', num2str(i)]), grid_var.(['vel_', num2str(i)]), grid_var.(['acc_', num2str(i)]), a, tau_i, t_plus, t_minus);

                % enforce ZD invariance
                [obj] = AddConstraint(obj, ddy_i + ctrl_gain^2*y_i + 2*ctrl_gain*dy_i, -obj.Settings.ConstraintTolerance*ones(numel(rbm.Inputs.u.sym),1), obj.Settings.ConstraintTolerance*ones(numel(rbm.Inputs.u.sym),1), 'ZD invariance');


            end
            
            

        otherwise
            error('not currently supported')

    end

end



end
