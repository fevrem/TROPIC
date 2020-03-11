function [ nlp ] = enforce_friction(nlp, rbm, Fc)

% if nargin ~= 4
%     error('wrong number of input arguments')
% end



% friction constraints 
%if nlp.Problem.contact.friction.bool
    
%     if isfield(nlp.Seed,'mu_friction')
%         mu = nlp.Seed.mu_friction;
%     else
%         error('shouldnt happen yet because Im copying FROST value')
%         mu = nlp.Problem.friction.mu;
%     end








switch nlp.Problem.contact.type
    case 'FxFz' %'planarWithFriction'

        

        nlp = add_constraint( nlp , Fc(2) , nlp.Problem.contact.forces.min_normal_force , Inf , 'normal force');

        if nlp.Problem.contact.friction.bool
            
            mu = nlp.Problem.contact.friction.mu;
            
            nlp = add_constraint( nlp , Fc(1) - mu*Fc(2) , -Inf , 0 ,'friction');
            nlp = add_constraint( nlp , -Fc(1) - mu*Fc(2) , -Inf , 0 ,'friction');
        end

    case 'FxFyFzMz' %'spatialWithFriction'
        
        
        
        nlp = add_constraint( nlp , Fc(3) , nlp.Problem.contact.forces.min_normal_force , Inf , 'normal force');

        if nlp.Problem.contact.friction.bool
            
            mu = nlp.Problem.contact.friction.mu;

            switch nlp.Problem.contact.friction.type
                case 'pyramid'

                    nlp = add_constraint( nlp , Fc(1) - 1/sqrt(2)*mu*Fc(3) , -Inf , 0 ,'friction');
                    nlp = add_constraint( nlp , -Fc(1) - 1/sqrt(2)*mu*Fc(3) , -Inf , 0 ,'friction');
                    nlp = add_constraint( nlp , Fc(2) - 1/sqrt(2)*mu*Fc(3) , -Inf , 0 ,'friction');
                    nlp = add_constraint( nlp , -Fc(2) - 1/sqrt(2)*mu*Fc(3) , -Inf , 0 ,'friction');

                case 'cone'
                    nlp = add_constraint( nlp , norm_2( [ Fc(1) ; Fc(2) ] ) - mu*Fc(3) , -Inf, 0); % casadi.norm_2 is norm() for class(var) casadi.MX 
            end
            
        end

        
        %nlp = add_constraint( nlp , Fc(4) , nlp.Problem.contact.forces.min_normal_force , Inf , 'normal force');

        nlp = add_constraint( nlp , Fc(4) , -nlp.Problem.contact.moments.max_normal_moment, nlp.Problem.contact.moments.max_normal_moment, 'normal moment');

        %error('how about F3')
 


    otherwise
        error('need to introduce these other cases')

end

    
    %{ 
    switch rbm.dimensions

        case '2D'

            nlp = add_constraint( nlp , Fc(2) , 10 , 1000 , 'friction');
            %warning('remove this??')
            %warning('remove this??')
            
            nlp = add_constraint( nlp , Fc(1) - mu*Fc(2) , -Inf , 0 ,'friction');
            nlp = add_constraint( nlp , -Fc(1) - mu*Fc(2) , -Inf , 0 ,'friction');


        case '3D'

            error('how about F3')
            switch nlp.Problem.friction.type
                case 'pyramid'
                    nlp = add_constraint( nlp , Fc(1) - 1/sqrt(2)*mu*Fc(3) , -Inf , 0 );
                    nlp = add_constraint( nlp , -Fc(1) - 1/sqrt(2)*mu*Fc(3) , -Inf , 0 );
                    nlp = add_constraint( nlp , Fc(2) - 1/sqrt(2)*mu*Fc(3) , -Inf , 0 );
                    nlp = add_constraint( nlp , -Fc(2) - 1/sqrt(2)*mu*Fc(3) , -Inf , 0 );
                case 'cone'
                    nlp = add_constraint( nlp , norm_2( [ Fc(1) ; Fc(2) ] ) - mu*Fc(3) , -Inf, 0); % casadi.norm_2 is norm() for class(var) casadi.MX 
            end

    end
    
    
    %}
    
    
    
%end








end
