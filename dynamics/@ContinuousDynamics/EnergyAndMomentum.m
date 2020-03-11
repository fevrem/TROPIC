function [KE, PE, cm, vcm] = EnergyAndMomentum(obj, sys)

% EnerMo  calculate energy, momentum and related quantities
% EnerMo(robot,q,qd)  returns a structure containing the fields KE, PE,
% htot, Itot, mass, cm and vcm.  These fields contain the kinetic and
% potential energies of the whole system, the total spatial momentum, the
% total spatial inertia, total mass, position of centre of mass, and the
% linear velocity of centre of mass, respectively.  Vector quantities are
% expressed in base coordinates.  PE is defined to be zero when cm is
% zero.

model = sys.Model;
Xtree = model.Xtree;

q = sys.States.q.sym;
qd = sys.States.dq.sym;

nd = model.nd;
jtype = model.jtype;
parent = model.parent;



KE = q(1)*0+zeros(nd,1);

for i = 1:nd
    [ XJ, S ] = jcalc( jtype{i}, q(i) );
    vJ = S*qd(i);
    Xup{i} = XJ * Xtree{i};
    if parent(i) == 0
        v{i} = vJ;
    else
        v{i} = Xup{i}*v{parent(i)} + vJ;
    end
    Ic{i} = model.I{i};
    hc{i} = Ic{i} * v{i};
    KE(i) = 0.5 * v{i}' * hc{i};
end

ret.Itot = zeros(size(Ic{1}));
ret.htot = zeros(size(hc{1}));




for i = nd:-1:1
    if parent(i) ~= 0
        Ic{parent(i)} = Ic{parent(i)} + Xup{i}'*Ic{i}*Xup{i};
        hc{parent(i)} = hc{parent(i)} + Xup{i}'*hc{i};
    else
        ret.Itot = ret.Itot + Xup{i}'*Ic{i}*Xup{i};
        ret.htot = ret.htot + Xup{i}'*hc{i};
    end
end

a_grav = get_gravity(model);

if length(a_grav) == 6
    g = a_grav(4:6);			% 3D linear gravitational accn
    h = ret.htot(4:6);			% 3D linear momentum
else
    g = a_grav(2:3);			% 2D gravity
    h = ret.htot(2:3);			% 2D linear momentum
end


[mass, cm] = mcI(ret.Itot);


KE = sum(KE);
PE = - mass * dot(cm,g);
vcm = h / mass;



end
