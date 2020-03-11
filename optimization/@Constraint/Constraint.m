classdef Constraint 
    
    properties (SetAccess=public , GetAccess=public)
    
        Name
        
        Bool
        
        Function 
        
        Timing
        
        Value
         
        UpperBound
        
        LowerBound
        
    end
    
 
    
    methods
        
        function obj = Constraint()
    

            
            
        end
        
    end
    
    
    
    methods(Static)
        function con = addConstraint(con)
            
 
            idx = size(con,2)+1;

%             if idx == 1
% 
%                 con = Constraint()
% 
%             else
%                 con(idx) = Constraint()
%             end
            
            con{idx} = Constraint();

        end
      
        
    end
        
    
end

