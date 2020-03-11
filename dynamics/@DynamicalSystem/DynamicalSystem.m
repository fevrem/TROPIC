classdef DynamicalSystem < handle 
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
        
        Dynamics
        
        HTransforms
            
        BodyPositions
        
        BodyVelocities
        
        InputMap
        
        Gravity
        
%         Inputs
%         
%         States
   
        Model
        
        Name
        
    end
    
    properties (Access = public)
       
        Inputs
        
        States
        
        Contacts
        
    end
    
    
    
    %% Public methods
    methods
        
        
        function obj = DynamicalSystem(model)%, dynamics_type)%varargin )
                        
            
            arguments
                model (1,1) function_handle
            end

            % call the superclass constructor
            robot_structure = model();
            
            
            obj.Model = robot_structure;
            
            obj.Name = robot_structure.name;
            obj.Model = rmfield(obj.Model, 'name');
            
            obj = AddStates(obj);
            
            obj = AddInputs(obj);
            
            obj.Gravity = robot_structure.gravity;
            obj.Model = rmfield(obj.Model, 'gravity');
            
            obj.InputMap = robot_structure.B;
            obj.Model = rmfield(obj.Model, 'B');

            obj.HTransforms = HomogeneousTransforms(obj);
           
            obj.BodyPositions = GetBodyPositions(obj);
            
            obj.BodyVelocities = GetBodyVelocities(obj);
            
            obj.Dynamics = ContinuousDynamics(obj);
            
            

        end
        
        
       
        
        
    end
    

    
  
    
end

