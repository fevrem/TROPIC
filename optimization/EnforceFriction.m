%function [symExpr, depVariables, symFun, conLB, conUB] = EnforceFriction(rbm, contactDyn)
function con = EnforceFriction(con, rbm)%, contactDyn)

arguments
    con (1,:) cell
    rbm (1,1) DynamicalSystem
    %contactDyn (1,1) ContactDynamics
end

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


import casadi.*

% number of contacts
nc = size(rbm.Contacts, 2);

sym_con = [];
LB_con = [];
UB_con = [];

Fc_vars_sym = [];
Fc_vars_ID = {};

for i = 1:nc
        
    if rbm.Contacts{i}.Friction.bool
    
        mustBeMember(rbm.Contacts{i}.Friction.Type, {'Pyramid', 'Cone'})
        
        validateForceNum(rbm.Contacts{i}.ContactType, rbm.Contacts{i}.Fc.sym)

        mu_i = rbm.Contacts{i}.Friction.mu;
        Fc_i = rbm.Contacts{i}.Fc.sym;
                        
        switch rbm.Contacts{i}.Friction.Type
            case 'Cone'


                switch rbm.Contacts{i}.ContactType
                    case 'Point'

                        

                        sym_con = [sym_con; 
                            norm_2( [ Fc_i(1) ; Fc_i(2) ] ) - mu_i*Fc_i(3)]; % casadi.norm_2 is norm() for class(var) casadi.MX 

                        LB_con = [LB_con; -Inf];
                        UB_con = [UB_con; 0];


                    case 'Line'
                        error('not done yet')

                    case 'Plane'
                        error('not done yet')

                end




            case 'Pyramid'

                switch rbm.Contacts{i}.ContactType
                    case 'Point'

  

                        sym_con = [sym_con; 
                            Fc_i(1) - 1/sqrt(2)*mu_i*Fc_i(3); 
                            -Fc_i(1) - 1/sqrt(2)*mu_i*Fc_i(3);
                            Fc_i(2) - 1/sqrt(2)*mu_i*Fc_i(3);
                            -Fc_i(2) - 1/sqrt(2)*mu_i*Fc_i(3)];

                        LB_con = [LB_con; -Inf*ones(4,1)];
                        UB_con = [UB_con; zeros(4,1)];


                    case 'Line'
                        error('not done yet')

                    case 'Plane'
                        error('not done yet')

                end



        end



        
        
        
        
    end
    
    Fc_vars_sym = [Fc_vars_sym; rbm.Contacts{i}.Fc.sym];
    
    Fc_vars_ID{end+1} = rbm.Contacts{i}.Fc.ID;
    
end


% nc = size(rbm.Contacts, 2)
% nCon = size(sym_con,2)
% 
% symExpr = [];
% 
% for i = 1:nCon
%    symExpr = [symExpr; ];
%    UBvec = [];
%    LBvec = [];
% end



for i = 2:nc
    if ~strcmp(Fc_vars_ID{i}, Fc_vars_ID{1})
        error('Treating all the contact forces together but they have different ID.')
    end
end
%Fc_vars_ID = Fc_vars_ID{1};

%second_order_ode = second_order_ode - sumContactForces;

variables = {Fc_vars_sym};
variablesID = {Fc_vars_ID{1}};


if isempty(LB_con)
    error('There is no friction constraint to enforce.')
end



symFun = Function('f', variables, {sym_con});
%conLB = vertcat(LB_con{:});
%conUB = vertcat(UB_con{:});


con{end}.SymbolicExpression = sym_con;
con{end}.DependentVariables = variables;
con{end}.DependentVariablesID = variablesID;
con{end}.Function = symFun;
con{end}.UpperBound = UB_con;
con{end}.LowerBound = LB_con;
%con(end).Occurrence = 1:ncp;



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


function validateForceNum(contact_type, contact_forces)

nf = numel(contact_forces);

switch contact_type
    case 'Point'
        if nf ~= 3
            error('Error. \nContact wrench must be 3x1.\nThere must be 3 nontrivial contact forces for a point contact, not %d.', nf);
        end

    case 'Line'
        error('not done yet')

    case 'Plane'
        error('not done yet')

end

end
