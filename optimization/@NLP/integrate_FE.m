% function [obj, running_cost] = integrate_FE(obj, rbm, h, running_cost, cost_k, cost_previous, q_k, dq_k, x_previous, f_k, f_previous, u_k, u_previous, du_k, du_previous, idxArg )
function [obj, running_cost] = integrate_FE(obj, rbm, h, running_cost, cost_k, cost_previous, states_k, states_previous, control_k, control_previous, f_k, f_previous, idxArg )

q_k = states_k.pos;
dq_k = states_k.vel;

x_previous = [states_previous.pos; states_previous.vel];%[q_k; dq_k];

u_k = control_k.u;
u_previous = control_previous.u;

du_k = control_k.du;
du_previous = control_previous.du;


    idx = idxArg{2};

    NB = rbm.model.NB;


    switch obj.Settings.quadrature
        case 'trapezoidal'

            % integrate cost
            running_cost = running_cost + 1/2*h*(cost_k + cost_previous);  

            % integrate dynamics
           [obj] = add_constraint(obj, [q_k; dq_k] - (x_previous + 1/2*h*( f_k + f_previous )), -obj.Settings.constraint_tol*ones(2*NB,1), obj.Settings.constraint_tol*ones(2*NB,1), 'integrate' );

            % enforce slew rate
            if obj.Problem.slew_rate.bool
                [obj] = add_constraint(obj, u_k - (u_previous + 1/2*h*( du_k + du_previous )) , -obj.Settings.constraint_tol*ones(size(rbm.model.B,2),1), obj.Settings.constraint_tol*ones(size(rbm.model.B,2),1), 'slew rate');
            end

        otherwise
            error('not supported yet')

    end


end
