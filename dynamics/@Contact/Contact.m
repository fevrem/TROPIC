classdef Contact 


    properties (GetAccess = public, SetAccess = protected)
        
        
        ContactType
        
        ContactFrame
        
        Friction
        
        Jac_contact
        
        dJac_contact
        
        
        
        
    end
    
    properties (Access = public)
    
        Fc 
    
    end
    
    
    %% Public methods
    methods
        
        
        function obj = Contact(rbm, contact_type, varargin)
            
            arguments
                rbm (1,1) DynamicalSystem
                contact_type (1,:) char {mustBeMember(contact_type,{'Point','Line','Plane'})}
            end
            arguments (Repeating)
                varargin (1,2) cell
            end
            
            for i = 1:size(varargin,2)
            
                argVar = varargin{i};
                mustBeMember(argVar{1}, {'ContactFrame','Friction','FrictionType','FrictionCoefficient'})
                
                contactArg.(argVar{1}) = argVar{2};
                
                
            end
            
            
            if isfield(contactArg, 'Friction')
  
                if ~islogical(contactArg.Friction)
                    error('Error. \nFriction must be of type logical, not %s.' , class(contactArg.Friction));
                end
                
                obj.Friction.bool = contactArg.Friction;
                
                if contactArg.Friction
                    if isfield(contactArg, 'FrictionType')
                        mustBeMember(contactArg.FrictionType,{'Pyramid','Cone'})
                        obj.Friction.Type = contactArg.FrictionType;
                    else
                        % assign pyramid automatically
                        obj.Friction.Type = 'Pyramid';
                    end
                    
                    if isfield(contactArg, 'FrictionCoefficient')
                        mustBeGreaterThan(contactArg.FrictionCoefficient, 0)
                        
                        obj.Friction.mu = contactArg.FrictionCoefficient;
                        
                    else
                        error('FrictionCoefficient is a required argument for contacts with friction.')
                    end
                end
                
                
            else
                error('Friction (logical) is a required argument of Contact');
            end
            


            switch contact_type
                case 'Point'
                    
                    switch rbm.Model.dimensions
                        case 'planar'
                            nF = 2;
                        case 'spatial'
                            nF = 3;
                    end

                    obj.Fc = Var('Fc_', nF);                    
                    
                    obj.Fc.ID = 'cForce'; % same ID for all contact forces because fixed number for each phase
                    obj.ContactType = 'Point';
                    
                    obj.Fc
                                
                    if ~isfield(contactArg, 'ContactFrame')
                        error('ContactFrame is a required argument for point contacts.')
                    end
            
                    obj = PointContact(obj, rbm, contactArg.ContactFrame);
                    
                case 'Line'
                    obj.ContactType = 'Line';
                    error('Not supported yet')
                    
                case 'Plane'
                    obj.ContactType = 'Plane';
                    error('Not supported yet')
                
                    
            end
            

        end
        
        
       
        
        
    end
    
    

    


  
    
end

