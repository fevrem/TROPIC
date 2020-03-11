function [obj] = AddConstraint(obj, constr, lb, ub, varargin)

if nargin == 5
    cstr_name = varargin{1};
else
    cstr_name = '';
end


if size(constr,2) ~= 1
    error('Error.\nConstraint %s should be a column vector but is %dx%d instead.', cstr_name, size(constr,1), size(constr,2))
end

if size(lb,2) ~= 1 || size(lb,1) ~= size(constr,1)
    error('Error.\n[%s].LB should be %dx%d but is %dx%d instead.', cstr_name, size(constr,1), size(constr,2), size(lb,1), size(lb,2))
end

if size(ub,2) ~= 1 || size(ub,1) ~= size(constr,1)
    error('Error.\n[%s].UB should be %dx%d but is %dx%d instead.', cstr_name, size(constr,1), size(constr,2), size(ub,1), size(ub,2))
end


obj.Constr = {obj.Constr{:}, constr};
obj.ConstrLB = [obj.ConstrLB; lb];
obj.ConstrUB = [obj.ConstrUB; ub];
obj.ConstrName = {obj.ConstrName{:}, cstr_name}; %#ok<*CCAT>

    
end
