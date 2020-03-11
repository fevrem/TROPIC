classdef NLP
    

    properties (SetAccess=immutable, GetAccess=public)
        % The name of the problem
        %
        % @type char
        %Type
        
        Settings
        
        
        
    end
    
    
    properties (SetAccess=protected , GetAccess=public)
    
        
        Functions
                
        Problem
        
        Vars
        
        VarsName
        
        VarsInit
        
        VarsLB
        
        VarsUB
        
        VarsIdx
        
        Constr
        
        ConstrName
        
        ConstrLB
        
        ConstrUB
        
        Cost
        
        
        
    end
    
    
    
    
    
    properties 
        
        % The solution of the NLP problem
        Sol
        
    end
    
    
    methods
        
        function obj = NLP(rbm, varargin)

            arguments
               rbm (1,1) DynamicalSystem 
            end
            
            arguments (Repeating)
                varargin (1,2) cell
            end
            
            obj.Vars = {};
            obj.VarsName = {};
            obj.VarsIdx = {};
            obj.Constr = {};
            obj.ConstrName = {};
            
            % supported functions (set by ConfigFunctions)
            %obj.Functions = ParseFunctions();
            obj.Functions = UserFunctions();    
            
            if nargin <= 1
                return;
            else    
                for i = 1:nargin-1
                    tempArg = varargin{i};
                    argIn.(tempArg{1}) = tempArg{2};
                end
            end


            if isfield(argIn, 'NFE')
                validateattributes(argIn.NFE, {'double'}, {'nonnegative'})
                obj.Settings.nfe = argIn.NFE;
            end
            
            if isfield(argIn, 'CollocationScheme')                
                validateattributes(argIn.CollocationScheme, {'char'}, {'size',[1,NaN]})
                mustBeMember(argIn.CollocationScheme, {'HermiteSimpson','Trapezoidal'})
                obj.Settings.CollocationScheme = argIn.CollocationScheme;
            end
            
            if isfield(argIn, 'LinearSolver')                
                validateattributes(argIn.LinearSolver, {'char'}, {'size',[1,NaN]})
                mustBeMember(argIn.LinearSolver, {'ma27','ma57','mumps'})
                obj.Settings.LinearSolver = argIn.LinearSolver;
            end
                        
            
            if isfield(argIn, 'ConstraintTolerance')                
                validateattributes(argIn.ConstraintTolerance, {'double'}, {'nonnegative'})
                obj.Settings.ConstraintTolerance = argIn.ConstraintTolerance;
            end
            
            
 

            switch obj.Settings.CollocationScheme
                case 'Trapezoidal'
                    obj.Settings.ncp = obj.Settings.nfe + 1;
                case 'HermiteSimpson'
                    obj.Settings.ncp = 2*obj.Settings.nfe + 1;
                otherwise
                    error('Error.\n%s is not a currently supported collocation method.\nOptions are ''Trapezoidal'' and ''HermiteSimpson''.', obj.Settings.CollocationScheme)
            end
                
                
 
            obj.Problem = ConstraintList();
            
           
            
            
        end
        
    end
  
        
    
end

