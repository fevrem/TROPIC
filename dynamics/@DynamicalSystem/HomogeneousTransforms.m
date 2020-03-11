function T = HomogeneousTransforms(obj)

arguments
    obj (1,1) DynamicalSystem
end

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
  
end