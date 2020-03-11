% function [obj, var, var_inds] = AddVar(obj, name, len, init, lb, ub, varargin)
function [obj,var] = AddVar(obj, name, len, init, lb, ub, varargin)


if nargin == 7
    var_name = varargin{1};
else
    var_name = '';
end


if size(lb,2) ~= 1 || size(lb,1) ~= len
    error('Error.\n[%s].LB should be %dx%d but is %dx%d instead.', name, len, 1, size(lb,1), size(lb,2))
end

if size(ub,2) ~= 1 || size(ub,1) ~= len
    error('Error.\n[%s].UB should be %dx%d but is %dx%d instead.', name, len, 1, size(ub,1), size(ub,2))
end


import casadi.*
var = MX.sym(name, len);    
obj.Vars = {obj.Vars{:}, var}; %#ok<*CCAT>
obj.VarsName = {obj.VarsName{:}, var_name}; %#ok<*CCAT>
obj.VarsLB = [obj.VarsLB; lb];
obj.VarsUB = [obj.VarsUB; ub ];
obj.VarsInit = [obj.VarsInit; init ];

ind_end = length( obj.VarsLB );
ind_begin = ind_end-len+1;
var_inds = ind_begin: ind_end;

obj.VarsIdx = {obj.VarsIdx{:}, var_inds};


end
