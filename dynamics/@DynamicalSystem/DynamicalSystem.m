classdef DynamicalSystem < handle 
    
    properties (GetAccess = public, SetAccess = protected)
        
        Dynamics
        
        HTransforms
            
        BodyPositions
        
        BodyVelocities
        
        InputMap
        
        Gravity
   
        Model
        
        Name
        
    end
    
    properties (Access = public)
       
        Inputs
        
        States
        
        Contacts
        
    end
    
    
 
    methods
        
        function obj = DynamicalSystem(model)

            arguments
                model (1,1) function_handle
            end

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