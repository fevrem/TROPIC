classdef Contact 
    % A class with basic model descriptions and functionalities
    % of the multi-link rigid-body robot platform.
    %
    % - Roy Featherstones. "Rigid Body Dynamics Algorithm". Springer, 2008.
    % http://royfeatherstone.org/spatial/
    %
    % @author Ayonga Hereid @date 2016-09-26
    % @new{1,0, ayonga, 2016-09-26} Migrated from old modelConfig class
    %
    % Copyright (c) 2016, AMBER Lab
    % All right reserved.
    %
    % Redistribution and use in source and binary forms, with or without
    % modification, are permitted only in compliance with the BSD 3-Clause
    % license, see
    % http://www.opensource.org/licenses/bsd-license.php
    
    
    properties (Constant)
        
        
    end
    
    
    properties (GetAccess = public, SetAccess = protected)
        
        % The robot model
        %   type: struct
        
        
        ContactType
        
        ContactFrame
        
        Friction
        
        Jac_contact
        
        dJac_contact
        
        %ContactForces
        
        %HybridDynamics
        %ForceVector
        
        
        
    end
    
    properties (Access = public)%(GetAccess = public, SetAccess = public)
    
        Fc 
    
    end
    
    
    %% Public methods
    methods
        
        
        function obj = Contact(rbm, contact_type, varargin)%, dynamics_type)%varargin )
                        
            
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
%                     nc = size(rbm.Contacts,2)
% 
%                     obj.Fc = Var(['Fc', num2str(nc+1)], 3);


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
            

            
            %[obj.ContactForces, obj.ForceVector] = computeContactForces(obj, rbm);
            
            
            


        end
        
        
       
        
        
    end
    
    
    
    methods %(Access = private)
        
        %[obj] = getContact(obj)
          
        %validateContact(obj, contacts) 
        
    end
    


  
    
end

