classdef ContactDynamics 
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
        ContactTypes
        
        ContactFrames
        
        Friction
        
        Jac_contact
        
        dJac_contact
        
        ContactForces
        
        %HybridDynamics
        ForceVector
        
    end
    
    
    
    
    
    %% Public methods
    methods
        
        
        function obj = ContactDynamics(rbm, contacts)%, dynamics_type)%varargin )
                        
            
            arguments
                rbm (1,1) DynamicalSystem
                contacts
            end

            % call the superclass constructor
            %obj@DynamicalSystem(model)
            

            %if ~isfield(obj.Model, 'contact')
            %    error('Contact is a required argument.')
            %else
            %    contacts = obj.Model.contact;
            %end
                        
            [obj] = getContact(obj, rbm, contacts);

            
            
            [obj.ContactForces, obj.ForceVector] = computeContactForces(obj, rbm);
            
            
            


        end
        
        
       
        
        
    end
    
    
    
    methods %(Access = private)
        
        [obj] = getContact(obj)
          
        validateContact(obj, contacts) 
        
    end
    

    methods
        
        %validateContact(obj, contacts) 
        
%         function validateContact(obj,contacts) 
%              
%             for i = 1:numel(contacts)              
%                                 
%                 switch contacts(i).type
%                     case 'point'
%                         % for point contacts, the contact frame ("position 
%                         % of contact") is given by the position of the end
%                         % effector(s) given by contacts.points                        
%                         mustBeMember('points', fieldnames(contacts))
%                         
%                     case 'line'
%                         error('Need to implement ''line'' contact type.')
% 
%                     case 'plane'
%                         error('Need to implement ''line'' contact type.')
% 
%                     otherwise                        
%                         error('MyComponent:incorrectType',...
%                             'Error. \nContact type must be ''point'', ''line'', or ''plane'', not %s.', contacts(i).type)
%                 end
% 
%             end
%            
%         end
        
    end
    
    
    
    
  
    
end

