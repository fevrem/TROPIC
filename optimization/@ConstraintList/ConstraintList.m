classdef ConstraintList %< handle
    % NonlinearProgram defines an abstract class for general nonlinear
    % programing problems
    % 
    %
    % @author Ayonga Hereid @date 2016-10-26
    % 
    % Copyright (c) 2016, AMBER Lab
    % All right reserved.
    %
    % Redistribution and use in source and binary forms, with or without
    % modification, are permitted only in compliance with the BSD 3-Clause 
    % license, see
    % http://www.opensource.org/licenses/bsd-license.php
    
    
    %% Protected properties

    % build onto this list
    % if a constraint is not in this list, ParseNLP does not check for it
    % by default, they are all turned off
    properties (SetAccess=public , GetAccess=public)
    
        FinalTime
        
        StepLength
        
        StepWidth
        
        StepHeight
        
        ForwardWalkingSpeed
      
        %LateralWalkingSpeed

        PhaseVariableDerivative
        
        SlewRate
        
        Trajectory
        
        COMPosition
        
        COMVelocity
        
        InitialPositions % convenient for swing up pendulum
        
        FinalPositions % convenient for swing up pendulum

        InitialVelocities % convenient for swing up pendulum
        
        FinalVelocities % convenient for swing up pendulum
        
        SwingFootHeight
        
        GroundClearance
        
        StanceFootInitialPosition
        
        StanceFootInitialVelocity
        
        OneStepPeriodic
        
        SwingFootVerticalImpactVelocity
        
        SwingFootForwardImpactVelocity
        
        SwingFootLateralImpactVelocity
        
        FootInterference
        
    end
    
   
    
    
    properties 
        
        % The solution of the NLP problem
        %Sol
        
    end
    
    
    %% Public methods
    methods
        
%         function obj = NonlinearProgram(name)
        %function obj = NLP(name)
        function obj = ConstraintList()%rbm, phase, nfe)
        % The default class constructor function
        %
        % Parameters: 
        %  name: the name of the problem
            
        fdnames = fieldnames(obj);
        %numel(fdnames)
        for i = 1:numel(fdnames)
            
%             if strcmp(fdnames{i}, 'FinalTime') || strcmp(fdnames{i}, 'SwingFootHeight')
%                
%                 obj.(fdnames{i}) = FixedConstraint();
%                 obj.(fdnames{i}).Bool = false;
%                 continue;
% 
%             end

            
%             obj.(fdnames{i}) = VariableConstraint();
            obj.(fdnames{i}) = Constraint();
            obj.(fdnames{i}).Bool = false;
            

        end
        

            
        end
        
    end
    

    
end

