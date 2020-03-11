classdef HybridDynamics %< ContinuousDynamics
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
        %J_contact
        
        % The model name
        %   type: char
        %dJ_contact
        
        
        
    end
    
    
    
    
    
    %% Public methods
    methods
        
        
%         function obj = RobotDynamics( model , varargin)
        function obj = HybridDynamics(hds)%, dynamics_type)%varargin )
                        
            
            arguments
                hds (1,1) HybridDynamicalSystem
            end

            
            
        end
        
        
       
        
        
    end
    

    
    %% methods defined in seperate files
    methods

        %'1'
        
        %obj = HandC( obj )
        
        %obj = EnergyandMomentum( obj );

        %'2'
        
        %H_matrix = InertiaMatrix(obj, q, s, xup)
        
        %C_terms = CoriolisAndGravityTerms(obj, q, s, xup, fvp)

        
        %obj = TreeTransforms( obj )
        
        %obj = HomogeneousTransforms( obj )
        %
        %T = HomogeneousTransforms( obj )
        %
        
        
        
%         obj = addContact(obj, contact, fric_coef, geometry, load_path);
%         
%         obj = addFixedJoint(obj, fixed_joints, load_path);
%         
%         obj = removeContact(obj, contact);
%         
%         
%         
%         base_link = findBaseLink(obj, joints);
%         
%         terminals = findEndEffector(obj, joints);
%                 
%         indices = getLinkIndices(obj, link_names);
%         
%         indices = getJointIndices(obj, joint_names);
%         
%         
%         obj = cconfigure(obj, config, base, load_path);
%         
%         obj = configureKinematics(obj, dofs, links);
%         
%         obj = configureDynamics(obj, varargin);
%         
%         obj = configureActuator(obj, dofs, actuators);
%         
%         ang = getRelativeEulerAngles(obj, frame, R, p);
%         
%         [varargout] = getCartesianPosition(obj, frame, p);
%         
%         [varargout] = getEulerAngles(obj, frame, p);
%         
%         [varargout] = getBodyJacobian(obj, frame, p);
%         
%         [varargout] = getSpatialJacobian(obj, frame, p);
%         
%         mass = getTotalMass(obj);        
%         
%         pos = getComPosition(obj);
%         
%         bounds = getLimits(obj);
%         
%         obj = loadDynamics(obj,file_path,skip_load_vf,extra_fvecs);

    end
    
    
    
end

