% function obj = HomogeneousTransforms(obj)
function T = HomogeneousTransforms(obj)

arguments
    obj (1,1) DynamicalSystem
end

% References:
% The descriptions of rigid body model is based on but not limited to
% the following resrouces

% - Roy Featherstones. "Rigid Body Dynamics Algorithm". Springer, 2008.
% http://royfeatherstone.org/spatial/


% modified - 09/12/2019, Martin Fevre


model = obj.Model;

q = obj.States.q.sym;

Xtree = obj.Model.Xtree;


for i = 1:model.nd

    XJ = jcalc(model.jtype{i}, q(i));

    Xa{i} = XJ * Xtree{i};
    if model.parent(i) ~= 0
        Xa{i} = Xa{i} * Xa{model.parent(i)};
        Xa{i} = Xa{i};
    end
    
    if size(Xa{i},1) == 3		% Xa{i} is a planar coordinate xform
        [theta,r] = plnr(Xa{i});
        X = rotz(theta) * xlt([r;0]);
        T{i} = pluho(X);
    else
        T{i} = pluho(Xa{i});
    end

end

%obj.T = T;

  
end