%function [symExpr, depVariables, symFun, conLB, conUB] = EnforceFriction(rbm, contactDyn)
function nlp = EnforceFriction(nlp, rbm, Con, Fc)%, contactDyn)

arguments
    nlp (1,1) NLP
    rbm (1,1) DynamicalSystem
    Con (1,1) Contact
    Fc (:,1) casadi.MX
end


%import casadi.*

mustBeMember(Con.Friction.Type, {'Pyramid', 'Cone'})

validateForceNum(rbm, Con.ContactType, Fc)

mu = Con.Friction.mu;
mustBeNonnegative(mu)

%Fc_i = rbm.Contacts{i}.Fc.sym;

switch Con.Friction.Type
    case 'Cone'
        switch Con.ContactType
            case 'Point'
                
                switch rbm.Model.dimensions
                    case 'planar'                                     
                        nlp = add_constraint(nlp, Fc(1) - mu*Fc(2), -Inf, 0 ,'friction');
                        nlp = add_constraint(nlp, -Fc(1) - mu*Fc(2), -Inf, 0 ,'friction');

                        nlp = add_constraint(nlp, Fc(2), 0, Inf, 'Normal force');
                 
                    case 'spatial'
                        nlp = add_constraint(nlp, norm_2([Fc(1); Fc(2)]) - mu*Fc(3), -Inf, 0, 'Friction cone');  
                        
                        nlp = add_constraint(nlp, Fc(3), 0, Inf, 'Normal force');
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
                        nlp = add_constraint(nlp, Fc(1) - mu*Fc(2), -Inf, 0 ,'friction');
                        nlp = add_constraint(nlp, -Fc(1) - mu*Fc(2), -Inf, 0 ,'friction');

                        nlp = add_constraint(nlp, Fc(2), 0, Inf, 'Normal force');

                    case 'spatial'
                        
                        
                    sym_con = [Fc(1) - 1/sqrt(2)*mu*Fc(3); 
                            -Fc(1) - 1/sqrt(2)*mu*Fc(3);
                            Fc(2) - 1/sqrt(2)*mu*Fc(3);
                            -Fc(2) - 1/sqrt(2)*mu*Fc(3)];

                        nlp = add_constraint(nlp, sym_con, -Inf*ones(4,1), zeros(4,1), 'Friction pyramid');     
                
                    nlp = add_constraint(nlp, Fc(3), 0, Inf, 'Normal force');

                end
                
            case 'Line'
                error('not done yet')

            case 'Plane'
                error('not done yet')

        end
        
end






% switch nlp.Problem.contact.type
%     case 'FxFz' %'planarWithFriction'
% 
%         
% 
%         nlp = add_constraint( nlp , Fc(2) , nlp.Problem.contact.forces.min_normal_force , Inf , 'normal force');
% 
%         if nlp.Problem.contact.friction.bool
%             
%             mu = nlp.Problem.contact.friction.mu;
%             
%             nlp = add_constraint( nlp , Fc(1) - mu*Fc(2) , -Inf , 0 ,'friction');
%             nlp = add_constraint( nlp , -Fc(1) - mu*Fc(2) , -Inf , 0 ,'friction');
%         end
% 
%     case 'FxFyFzMz' %'spatialWithFriction'
%         
%         
%         
%         nlp = add_constraint( nlp , Fc(3) , nlp.Problem.contact.forces.min_normal_force , Inf , 'normal force');
% 
%         if nlp.Problem.contact.friction.bool
%             
%             mu = nlp.Problem.contact.friction.mu;
% 
%             switch nlp.Problem.contact.friction.type
%                 case 'pyramid'
% 
%                     nlp = add_constraint( nlp , Fc(1) - 1/sqrt(2)*mu*Fc(3) , -Inf , 0 ,'friction');
%                     nlp = add_constraint( nlp , -Fc(1) - 1/sqrt(2)*mu*Fc(3) , -Inf , 0 ,'friction');
%                     nlp = add_constraint( nlp , Fc(2) - 1/sqrt(2)*mu*Fc(3) , -Inf , 0 ,'friction');
%                     nlp = add_constraint( nlp , -Fc(2) - 1/sqrt(2)*mu*Fc(3) , -Inf , 0 ,'friction');
% 
%                 case 'cone'
%                     nlp = add_constraint( nlp , norm_2( [ Fc(1) ; Fc(2) ] ) - mu*Fc(3) , -Inf, 0); % casadi.norm_2 is norm() for class(var) casadi.MX 
%             end
%             
%         end
% 
%         
%         %nlp = add_constraint( nlp , Fc(4) , nlp.Problem.contact.forces.min_normal_force , Inf , 'normal force');
% 
%         nlp = add_constraint( nlp , Fc(4) , -nlp.Problem.contact.moments.max_normal_moment, nlp.Problem.contact.moments.max_normal_moment, 'normal moment');
% 
%         %error('how about F3')
%  
% 
% 
%     otherwise
%         error('need to introduce these other cases')
% 
% end
% 
%     
%     %{ 
%     switch rbm.dimensions
% 
%         case '2D'
% 
%             nlp = add_constraint( nlp , Fc(2) , 10 , 1000 , 'friction');
%             %warning('remove this??')
%             %warning('remove this??')
%             
%             nlp = add_constraint( nlp , Fc(1) - mu*Fc(2) , -Inf , 0 ,'friction');
%             nlp = add_constraint( nlp , -Fc(1) - mu*Fc(2) , -Inf , 0 ,'friction');
% 
% 
%         case '3D'
% 
%             error('how about F3')
%             switch nlp.Problem.friction.type
%                 case 'pyramid'
%                     nlp = add_constraint( nlp , Fc(1) - 1/sqrt(2)*mu*Fc(3) , -Inf , 0 );
%                     nlp = add_constraint( nlp , -Fc(1) - 1/sqrt(2)*mu*Fc(3) , -Inf , 0 );
%                     nlp = add_constraint( nlp , Fc(2) - 1/sqrt(2)*mu*Fc(3) , -Inf , 0 );
%                     nlp = add_constraint( nlp , -Fc(2) - 1/sqrt(2)*mu*Fc(3) , -Inf , 0 );
%                 case 'cone'
%                     nlp = add_constraint( nlp , norm_2( [ Fc(1) ; Fc(2) ] ) - mu*Fc(3) , -Inf, 0); % casadi.norm_2 is norm() for class(var) casadi.MX 
%             end
% 
%     end
%     
%     
%     %}
%     
%     
%     
% %end
% 
% 






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
