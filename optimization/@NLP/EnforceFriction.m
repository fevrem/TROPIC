function nlp = EnforceFriction(nlp, rbm, Con, Fc)

arguments
    nlp (1,1) NLP
    rbm (1,1) DynamicalSystem
    Con (1,1) Contact
    Fc (:,1) casadi.MX
end



mustBeMember(Con.Friction.Type, {'Pyramid', 'Cone'})

validateForceNum(rbm, Con.ContactType, Fc)

mu = Con.Friction.mu;
mustBeNonnegative(mu)



switch Con.Friction.Type
    case 'Cone'
        switch Con.ContactType
            case 'Point'
                
                switch rbm.Model.dimensions
                    case 'planar'                                     
                        nlp = AddConstraint(nlp, Fc(1) - mu*Fc(2), -Inf, 0 ,'friction');
                        nlp = AddConstraint(nlp, -Fc(1) - mu*Fc(2), -Inf, 0 ,'friction');

                        nlp = AddConstraint(nlp, Fc(2), 0, Inf, 'Normal force');
                 
                    case 'spatial'
                        nlp = AddConstraint(nlp, norm_2([Fc(1); Fc(2)]) - mu*Fc(3), -Inf, 0, 'Friction cone');
                        
                        nlp = AddConstraint(nlp, Fc(3), 0, Inf, 'Normal force');
                end
                
                
            case 'Line'
                error('not done yet')

            case 'Plane'
                error('not done yet')

        end




    case 'Pyramid'
        switch Con.ContactType
            case 'Point'

                switch rbm.Model.dimensions
                    case 'planar'                                     
                        nlp = AddConstraint(nlp, Fc(1) - mu*Fc(2), -Inf, 0 ,'friction');
                        nlp = AddConstraint(nlp, -Fc(1) - mu*Fc(2), -Inf, 0 ,'friction');

                        nlp = AddConstraint(nlp, Fc(2), 0, Inf, 'Normal force');

                    case 'spatial'
                        
                        
                    sym_con = [Fc(1) - 1/sqrt(2)*mu*Fc(3); 
                            -Fc(1) - 1/sqrt(2)*mu*Fc(3);
                            Fc(2) - 1/sqrt(2)*mu*Fc(3);
                            -Fc(2) - 1/sqrt(2)*mu*Fc(3)];

                        nlp = AddConstraint(nlp, sym_con, -Inf*ones(4,1), zeros(4,1), 'Friction pyramid');
                
                    nlp = AddConstraint(nlp, Fc(3), 0, Inf, 'Normal force');

                end
                
            case 'Line'
                error('not done yet')

            case 'Plane'
                error('not done yet')

        end
        
end




end


function validateForceNum(rbm, contact_type, contact_forces)

nf = numel(contact_forces);

switch contact_type
    case 'Point'
        
        switch rbm.Model.dimensions
            case 'spatial'
                if nf ~= 3
                    error('Error. \nContact wrench must be 3x1.\nThere must be 3 nontrivial contact forces for a 3D point contact, not %d.', nf);
                end
            case 'planar'
                if nf ~= 2
                    error('Error. \nContact wrench must be 2x1.\nThere must be 2 nontrivial contact forces for a 2D point contact, not %d.', nf);
                end
        end
        
        
        
    case 'Line'
        error('not done yet')

    case 'Plane'
        error('not done yet')

end

end
