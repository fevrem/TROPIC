classdef ContinuousDynamics < handle 
  
    
    
    properties (GetAccess = public, SetAccess = protected)
        

        H_matrix
        
        C_terms

        KE
        
        PE
        
        p_com
        
        v_com
        
        
    end
    
    
    methods

        function obj = ContinuousDynamics(sys)
                        
            [H,C] = HandC(obj, sys);
            
            obj.H_matrix = H;
            obj.C_terms  = C;
            
            
            
            [KE, PE, p_com, v_com] = EnergyAndMomentum(obj, sys);

            obj.KE = KE;
            obj.PE = PE;
            obj.p_com = p_com;
            obj.v_com = v_com;

        end

    end
    
    
    
    
end

