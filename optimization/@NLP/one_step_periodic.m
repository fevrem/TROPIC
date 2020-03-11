function [obj] = one_step_periodic(obj, rbm, states_0, states, idxArg)

    switch rbm.dynamics

        case 'hybrid'

            
            q_minus  = obj.Functions.q_minus( states.pos );
            qd_minus = obj.Functions.qd_minus( states.pos , states.vel );

%
% FROST MATCH
%
%
NB = rbm.model.NB;

q_guess = obj.Seed.q(:,end);
qd_guess = obj.Seed.qd(:,end);

q_lb = obj.Problem.states.q_lb;
q_ub = obj.Problem.states.q_ub;

qd_lb = obj.Problem.states.qd_lb;
qd_ub = obj.Problem.states.qd_ub;

% THIS IS GOOD
[obj, qminus]  = add_var( obj , 'qminus'  , NB , q_guess , q_lb , q_ub , 'qMinus' );
[obj, qdminus] = add_var( obj , 'qdminus' , NB , qd_guess , qd_lb , qd_ub , 'qdMinus' );

obj = add_constraint(obj, qminus - q_minus , zeros(NB,1) , zeros(NB,1) , 'qMinusCont' );     
obj = add_constraint(obj, qdminus - qd_minus , zeros(NB,1) , zeros(NB,1) , 'qdMinusCont' );     
%
%
%
      

% flip x_minus (eye(2n) if planar biped)
switch rbm.dimensions
    case '3D'
        [qminus, qdminus] = Model.flip_map( rbm , qminus , qdminus );
end


%%

% see which ones are Cont and which ones are impact

clc
[gait.states{1}.x(:,1),gait.states{1}.dx(:,1)]
[gait.states{1}.x(:,end),gait.states{1}.dx(:,end)]
[gait.states{2}.x,gait.states{2}.dx]
[gait.states{2}.xn,gait.states{2}.dxn]


gait.states{1}.x(:,1) - gait.states{2}.xn
gait.states{1}.x(:,end) - gait.states{2}.x

gait.states{1}.dx(:,1) - gait.states{2}.dxn
gait.states{1}.dx(:,end) - gait.states{2}.dx

% src = tar = stance = phase{1}
% edge = impact = phase{2}



%
'xMinusCont'
x_s: src.OptVarTable.x(end) -> gait.states{1}.x(:,end)
x_e: edge.OptVarTable.x(1)  -> gait.states{2}.x  (slack)
    % {1} <-> {end} - position
    x_s = src.Plant.States.x;
    x_e = SymVariable('xp',size(x_s));
    x_cont_src = SymFunction(['xMinusCont_' edge.Name],x_s-x_e,{x_s,x_e});
    {'DepVariables',[src.OptVarTable.x(end);edge.OptVarTable.x(1)]};
    edge.addConstraint('xMinusCont','first',x_src_cstr);
    
    
%    
'xPlusCont'    
x_e: edge.OptVarTable.xn(1) -> gait.states{2}.xn
x_t: tar.OptVarTable.x(1)   -> gait.states{1}.x(:,1)
    % {end} <-> {2} - position
    x_t = tar.Plant.States.x;
    x_e = edge.Plant.States.xn;
    x_cont_tar = SymFunction(['xPlusCont_' edge.Name],x_e-x_t,{x_e,x_t});
    {'DepVariables',[edge.OptVarTable.xn(1);tar.OptVarTable.x(1)]};
    edge.addConstraint('xPlusCont','first',x_tar_cstr);
    
    
%
'dxMinusCont'
dx_s: src.OptVarTable.dx(end) -> gait.states{1}.dx(:,end)
dx_e: edge.OptVarTable.dx(1)  -> gait.states{2}.dx (slack)
    % {1} <-> {end} - velocity
    dx_s = src.Plant.States.dx;
    dx_e = SymVariable('xp',size(x_s));
    dx_cont_src = SymFunction(['dxMinusCont_' edge.Name],dx_s-dx_e,{dx_s,dx_e});
    {'DepVariables',[src.OptVarTable.dx(end);edge.OptVarTable.dx(1)]};
    edge.addConstraint('dxMinusCont','first',dx_src_cstr);

    
%
'dxPlusCont'
dx_e: edge.OptVarTable.dxn(1) -> gait.states{2}.dxn
dx_t: tar.OptVarTable.dx(1)   -> gait.states{1}.dx(:,1)
    % {end} <-> {2} - velocity
    dx_t = tar.Plant.States.dx;
    dx_e = edge.Plant.States.dxn;
    dx_cont_tar = SymFunction(['dxPlusCont_' edge.Name],dx_e-dx_t,{dx_e,dx_t});
    {'DepVariables',[edge.OptVarTable.dxn(1);tar.OptVarTable.dx(1)]};
    edge.addConstraint('dxPlusCont','first',dx_tar_cstr);
    
x   : edge.OptVarTable.x(1)   -> gait.states{2}.x
xn  : edge.OptVarTable.xn(1)  -> gait.states{2}.xn 
dx  : edge.OptVarTable.dx(1)  -> gait.states{2}.dx
dxn : edge.OptVarTable.dxn(1) -> gait.states{2}.dxn

xMap:  R*x - xn == 0
dxMap: H*(dxn - R*dx) - J'*fimp
where H = H(xn), fimp is slack, and J = J(xn)

dxMap = f(dx,xn,dxn,fimp)

dxMap(1:7) == 0
xMap(3:7) == 0




%%
            
 
            
            % D(q) -> D(q^+)
                    M = subs(obj.Mmat, x, xn);
                    Gvec = subs(Gvec, x, xn);
                    % D(q^+)*(dq^+ - R*dq^-) = sum(J_i'(q^+)*deltaF_i)
                    delta_dq = M*(dxn - obj.R*dx) - Gvec;
                    obj.dxMap = SymFunction(['dxDiscreteMap' obj.Name],delta_dq,[{dx},{xn},{dxn},deltaF]);
                    
                    
   
% inertia matrix needed for impact
H = obj.Functions.H_matrix( qminus );

% contact jacobian for impact
J = obj.Functions.J_contact( qminus );

LHS = [ H , -J' ; J , zeros(size(J,1)) ];
RHS = [ H*qdminus ; zeros(size(J,1),1) ];




% 
% yFROST = LHS\RHS;
%         
% ImpF = yFROST(NB+1:end);

% idx = 1;
% for i=1:n_cstr
%     c_obj = cstr(i);
%     cstr_indices = idx:idx+c_obj.Dimension-1;
% 
%     % calculate the Jacobian
%     lambda.(c_obj.InputName) = ImpF(cstr_indices);
%     idx = idx + c_obj.Dimension;
% end

% dqplus = yFROST(1:NB);
% qplus  = qminus;
% 
% xn = [qplus; dqplus];

[obj, q_plus] = add_var( obj , 'QPlus' , NB , zeros(NB,1) , -Inf*ones(NB,1), Inf*ones(NB,1) , 'q_plus' );
%[obj, qd_plus] = add_var( obj , 'dQPlus' , NB , qd_guess , qd_lb , qd_ub , 'qd_plus' );
[obj, qd_plus] = add_var( obj , 'dQPlus' , NB , zeros(NB,1) , -Inf*ones(NB,1), Inf*ones(NB,1) , 'qd_plus' );



[obj, f_imp] = add_var( obj , 'fc_imp' , size(J,1) , zeros(size(J,1),1) , -Inf*ones(size(J,1),1) , Inf*ones(size(J,1),1) , 'fc_imp' );


obj = add_constraint(obj, LHS*[qd_plus;f_imp] - RHS , zeros(NB+size(J,1),1) , zeros(NB+size(J,1),1) , 'qdPlusCont' );     

%LHS*yFROST = RHS
%LHS*[qd_plus;f_imp] - RHS = 0
%obj = add_constraint(obj, yFROST(1:NB) - qd_plus , zeros(NB,1) , zeros(NB,1) , 'qPlusCont' );     


obj = add_constraint(obj, qminus - q_plus , zeros(NB,1) , zeros(NB,1) , 'qPlusCont' );     
%obj = add_constraint(obj, yFROST(1:NB) - qd_plus , zeros(NB,1) , zeros(NB,1) , 'qdPlusCont' );     



if 0
%{
    FROST INVERTS THE IMPACT MATRIX FOR 
    DISCRETE DYNAMICS, I DO THAT HERE TO 
    HAVE THE SAME FORMULATION FOR COMPARISON 
    BUT PROBABLY BETTER TO GO BACK TO 
    IMPLICIT DEFINITION IN THE FUTURE
%}
% xPlusCont 
% -> xPlus = x0

% xMinusCont
% -> xMinus swapped 

%
%
%FROST MATCH

%[obj, f_imp] = add_var( obj , 'fc_imp' , size(J,1) , 100*ones(size(J,1),1) , -1000*ones(size(J,1),1) , 1000*ones(size(J,1),1) , 'fc_imp' );

% IF USING EXPLICIT MAP, FC SHOULDNT BE DEFINED SINCE IT IS CALCULATED
[obj, f_imp] = add_var( obj , 'fc_imp' , size(J,1) , zeros(size(J,1),1) , -Inf*ones(size(J,1),1) , Inf*ones(size(J,1),1) , 'fc_imp' );
%
%obj = add_constraint(obj, LHS*[qd_plus;f_imp] - RHS , -obj.Settings.constraint_tol*ones(NB+size(J,1),1) , obj.Settings.constraint_tol*ones(NB+size(J,1),1) , 'impact' );     

obj = add_constraint(obj, q_plus - states_0.pos , zeros(NB,1) , zeros(NB,1) , 'qPlusCont' );     
obj = add_constraint(obj, qd_plus - states_0.vel , zeros(NB,1) , zeros(NB,1) , 'qdPlusCont' );     


%
obj = add_constraint(obj, q_plus - qminus , -obj.Settings.constraint_tol*ones(NB+size(J,1),1) , obj.Settings.constraint_tol*ones(NB+size(J,1),1) , 'impact' );     
% obj = add_constraint(obj, LHS*[qd_plus;f_imp] - RHS , -obj.Settings.constraint_tol*ones(NB+size(J,1),1) , obj.Settings.constraint_tol*ones(NB+size(J,1),1) , 'impact' );     
%
%
end
            
            
if 0    
    % impact
    [obj,q_plus,qd_plus] = apply_impact(obj, rbm, qminus, qdminus);        

end
            
            
            % periodicity constraints
            [obj] = enforce_periodicity(obj, rbm, [states_0.pos; states_0.vel], [q_plus;qd_plus] );
            
            
            
            
            
            warning('add variables for new x0 states')
            warning('add constrainints with x0')


    end


end
