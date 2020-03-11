function [obj] = getContact(obj, rbm, contacts)
    arguments
        obj (1,1) ContactDynamics
        rbm (1,1) DynamicalSystem
        contacts (1,:) struct
    end

    
    validateContact(obj, contacts)

    q = rbm.States.q.sym;
    dq = rbm.States.dq.sym;

    
    for i = 1:numel(contacts)              
                                
        switch contacts(i).type
            case 'point'
                % for point contacts, the contact frame ("position 
                % of contact") is given by the position of the end
                % effector(s) given by contacts.points                        
                
                obj.ContactTypes{i} = contacts(i).type;

                obj.ContactFrames{i} = rbm.BodyPositions{contacts(i).points, 2};
                
                Jc_i = jacobian(rbm.BodyPositions{contacts(i).points, 2}, q);
                obj.Jac_contact{i} = Jc_i;
                
                dJc_i = jacobian( Jc_i * dq, q);
                obj.dJac_contact{i} = dJc_i;


                obj.Friction{i} = contacts(i).friction;

                
            case 'line'
                error('Need to implement ''line'' contact type.')

                
            case 'plane'
                error('Need to implement ''line'' contact type.')

                
            otherwise                        
                error('MyComponent:incorrectType',...
                    'Error. \nContact type must be ''point'', ''line'', or ''plane'', not %s.', contacts(i).type)
        
        end

    end
          

    %obj.Model = rmfield(obj.Model, 'contact');


end