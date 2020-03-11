function pos_body = GetBodyPositions(obj)

arguments
    obj (1,1) DynamicalSystem
end
  
    
model = obj.Model;
nd = model.nd;

T = obj.HTransforms; 

for i = 1:nd
    
    T_i = T{i};
    R_i = T_i(1:3,1:3)';
    
    p_i = -R_i*T_i(1:3,4);

    T_next = [R_i, p_i; [0 0 0, 1]];
    p_next = T_next*[model.body_length{i}*model.body_axis{i}, 1]';
    
    pos_body{i,1} = p_i;
    pos_body{i,2} = p_next(1:3);
    
end


end