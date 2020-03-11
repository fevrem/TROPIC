function [Y,DY,DDY] = ControllerOutput(VirtualCon, rbm, phi, dphi, d2phi, parameterization_type)


arguments
    VirtualCon (1,1) VirtualConstraint
    rbm (1,1) DynamicalSystem
    phi (:,1) casadi.SX
    dphi (:,1) casadi.SX
    d2phi (:,1) casadi.SX
    parameterization_type (1,:) {mustBeMember(parameterization_type, {'time-based','state-based'})}
end



q = rbm.States.q.sym;
qd = rbm.States.dq.sym;
qdd = rbm.States.ddq.sym;



switch parameterization_type
    
    case 'state-based'
        
        % gradient of desired trajectories
        dphi_dq   = dphi * rbm.Model.c;
        d2phi_dq2 = d2phi * rbm.Model.c.^2;

        

        % controller outputs and time derivatives
        Y   = rbm.Model.H0*q - phi;
        DY  = (rbm.Model.H0 - dphi_dq)*qd;
        DDY = (rbm.Model.H0 - dphi_dq)*qdd - d2phi_dq2*qd.^2;

    case 'time-based'
        
        % in this case, theta (the phase variable) was simply time
        
        % controller outputs and time derivatives
        Y   = rbm.Model.H0*q - phi;
        DY  = rbm.Model.H0*qd - dphi;
        DDY = rbm.Model.H0*qdd - d2phi;
        
    otherwise
        
        error('not supported yet')
        
end




end