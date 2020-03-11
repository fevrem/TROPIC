%function [casadi] = extra_functions( rbm , casadi , x )
function funs = extra_functions(obj , rbm )

    

funs = Opt.extra_functions(obj, rbm );

    
import casadi.* % CasADi 3.4.5

%{
    No extra functions as of now
%}    



end