function validatePhase(obj, contacts) 
             
    for i = 1:numel(contacts)              

        switch contacts(i).type
            case 'point'
                % for point contacts, the contact frame ("position 
                % of contact") is given by the position of the end
                % effector(s) given by contacts.points                        
                mustBeMember('points', fieldnames(contacts))

            case 'line'
                error('Need to implement ''line'' contact type.')

            case 'plane'
                error('Need to implement ''line'' contact type.')

            otherwise                        
                error('MyComponent:incorrectType',...
                    'Error. \nContact type must be ''point'', ''line'', or ''plane'', not %s.', contacts(i).type)
        end

    end

end