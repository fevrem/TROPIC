function [obj] = getPhase(obj, contacts)
    arguments
        obj (1,:) HybridDynamicalSystem
        contacts (1,:) struct
    end

    
    validateContact(obj, contacts)

   
    
    
    
    for i = 1:numel(contacts)              
                                
        switch contacts(i).type
            case 'point'
                % for point contacts, the contact frame ("position 
                % of contact") is given by the position of the end
                % effector(s) given by contacts.points                        
                
                obj.ContactTypes{i} = contacts(i).type;

                obj.ContactFrames{i} = obj.BodyPositions{contacts(i).points, 2};
                
                Jc_i = jacobian(obj.BodyPositions{contacts(i).points, 2}, obj.States.q);
                obj.J_contact{i} = Jc_i;
                
                dJc_i = jacobian( Jc_i * obj.States.dq, obj.States.q);
                obj.dJ_contact{i} = dJc_i;



                
                
 
            case 'line'
                error('Need to implement ''line'' contact type.')

                
            case 'plane'
                error('Need to implement ''line'' contact type.')

                
            otherwise                        
                error('MyComponent:incorrectType',...
                    'Error. \nContact type must be ''point'', ''line'', or ''plane'', not %s.', contacts(i).type)
        
        end

    end
          

    obj.Model = rmfield(obj.Model, 'contact');


end