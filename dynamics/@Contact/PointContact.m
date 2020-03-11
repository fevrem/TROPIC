function [obj] = PointContact(obj, rbm, frame)

arguments
    obj (1,1) Contact
    rbm (1,1) DynamicalSystem
    frame (:,1) casadi.SX
end



switch rbm.Model.dimensions
    case 'planar'
        if size(frame,1) ~= 2
            error('Contact frame must be 2x1 for 2D point contacts')
        end
    case 'spatial'
        if size(frame,1) ~= 3
            error('Contact frame must be 3x1 for 3D point contacts')
        end
end


%validatePointContact(obj, contacts)

q = rbm.States.q.sym;
dq = rbm.States.dq.sym;


obj.ContactFrame = frame;



Jc = jacobian(frame, q);
obj.Jac_contact = Jc;

dJc = jacobian( Jc * dq, q);
obj.dJac_contact = dJc;


end

    



