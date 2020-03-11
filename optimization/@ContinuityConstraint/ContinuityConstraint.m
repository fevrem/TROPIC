classdef ContinuityConstraint < Constraint
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

    
    
    properties (SetAccess=public , GetAccess=public)
    
        %Name
        
        TargetVars
        
        TargetPhase
        
        TargetTiming
        
        SourceVars
        
        SourcePhase
        
        SourceTiming
        
        %PreVars
        
        %PostVars
        
        %SymbolicExpression
        
        %UpperBound
        
        %LowerBound
        
        %Function
        
        %Occurrence
        
    end
    
   
    
    
    properties 
        
        % The solution of the NLP problem
        %Sol
        
    end
    
    
    %% Public methods
    methods
        
%         function obj = NonlinearProgram(name)
        %function obj = NLP(name)
        function obj = ContinuityConstraint()%rbm, phase, nfe)
        % The default class constructor function
        %
        % Parameters: 
        %  name: the name of the problem
   
            obj@Constraint
            %con
            %idx = addConstraint(obj, con)
            
            %obj(idx) = []
            
            
        end
        
    end
    
    %% Function definitions
    methods(Static)
        function con = addConstraint(con)
            
            %con
            %size(con)
            idx = size(con,2)+1;

%             if idx == 1
% 
%                 con = Constraint()
% 
%             else
%                 con(idx) = Constraint()
%             end
            
            con{idx} = ContinuityConstraint();

        end
      
        
    end
        
    
end

