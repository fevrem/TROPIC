function H = InertiaMatrix(obj, sys, q, S, Xup)

model = sys.Model;

IC = model.I; % composite inertia calculation

nd = model.nd;
parent = model.parent;

for i = nd:-1:1
    if parent(i) ~= 0
        IC{parent(i)} = IC{parent(i)} + Xup{i}'*IC{i}*Xup{i};
    end
end

H = q(1)*0 + zeros(nd);
for i = 1:nd
    fh = IC{i} * S{i};  
    H(i,i) = S{i}' * fh;
    j = i;
    while parent(j) > 0
        fh = Xup{j}' * fh;
        j = parent(j);
        H(i,j) = S{j}' * fh;
        H(j,i) = H(i,j);
    end
end


end