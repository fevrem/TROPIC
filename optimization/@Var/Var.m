classdef Var 
    
    properties (SetAccess=public , GetAccess=public)
    
        sym
        
        ID
        
        UpperBound
        
        LowerBound
       
        Seed
        
    end
    
   
    
    
    methods

        function obj = Var(varID, varDim)

            arguments
                varID (1,:) char
                varDim (1,1) double
            end
        
            mustBeInteger(varDim)
            mustBePositive(varDim)
            
            obj.ID = varID;
            
            import casadi.*
            
            obj.sym = SX.sym(varID, varDim);
                        
            
        end
        
    end
    
        
    
end

