classdef ContinuousDynamics < handle 
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
%         J_contact
%         
%         % The model name
%         %   type: char
%         dJ_contact
        
        H_matrix
        C_terms
%         B_matrix
%         Position_states
%         Velocity_states
        KE
        PE
        
        p_com
        v_com
        
        
    end
    
    
    
    
    
    %% Public methods
    methods
        
        
%         function obj = RobotDynamics( model , varargin)
        function obj = ContinuousDynamics(sys)%, dynamics_type)%varargin )
                        
            
%             arguments
%                 obj (1,1) {mustBeMember(obj,{DynamicalSystem,HybridDynamicalSystem})}
%             end
        
            

            
            
            %obj@ContinuousDynamics(model)
            

            [H,C] = HandC(obj, sys);
            
            obj.H_matrix = H;
            obj.C_terms  = C;
            
            
            
            [KE, PE, p_com, v_com] = EnergyAndMomentum(obj, sys);
            %[] = EnergyAndMomentum(obj, model.q, model.qd);
            obj.KE = KE;
            obj.PE = PE;
            obj.p_com = p_com;
            obj.v_com = v_com;
            
            
            
            
            
            
            
            return
            frmkkfm
            
            % call the superclass constructor
            %
            
            

            obj.J_contact = 1
            
            
            obj.dJ_contact = 1
            
%             [eta,etadot,Jc, Jcd] = Model.get_contact_jac( obj );
%             obj.eta = eta;
%             obj.etadot = etadot;
%             obj.J_contact = Jc;
%             obj.Jdot_contact = Jcd;


            
            
        end
        
        
       
        
        
    end
    

    
    %% methods defined in seperate files
    methods 

        %ContinuousSecondOrder(rbm)
        
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

