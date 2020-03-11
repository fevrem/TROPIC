function [obj, rbm] = LoadConstraints(obj, rbm, varargin)

arguments
    obj (1,1) NLP
    rbm (1,1) DynamicalSystem
end

arguments (Repeating)
    varargin (1,2) cell
end

        

[obj.Problem, rbm] = Opt.ConList(obj.Problem, rbm, obj.Settings.ncp);

% validate attributes for required fields
fields_list = {rbm.States.q,...
    rbm.States.dq,...
    rbm.States.ddq,...
    rbm.Inputs.u};

for i = 1:numel(fields_list)
    
    validateattributes(fields_list{i}.('LowerBound'), {'double'}, {'size',[numel(fields_list{i}.('sym')),obj.Settings.ncp]}, mfilename, [fields_list{i}.('ID'), '.LowerBound'])
    validateattributes(fields_list{i}.('UpperBound'), {'double'}, {'size',[numel(fields_list{i}.('sym')),obj.Settings.ncp]}, mfilename, [fields_list{i}.('ID'), '.UpperBound'])
    
end



if obj.Problem.SlewRate.Bool
    rbm.Inputs.du = Var('du', numel(rbm.Inputs.u.sym));
    rbm.Inputs.du.LowerBound = obj.Problem.SlewRate.LowerBound*ones(numel(rbm.Inputs.du.sym),obj.Settings.ncp);
    rbm.Inputs.du.UpperBound = obj.Problem.SlewRate.UpperBound*ones(numel(rbm.Inputs.du.sym),obj.Settings.ncp);
end


for i = 1:numel(rbm.Contacts) 
    if ~isa(rbm.Contacts{i},'Contact')
        error('Error.\nContact no.%d is initialized in user-constraint list but must be declared using the Contact class first.', i) 
    end
         
    validateattributes(rbm.Contacts{i}.Fc.('LowerBound'), {'double'}, {'size',[numel(rbm.Contacts{i}.Fc.sym),obj.Settings.ncp]}, mfilename, [rbm.Contacts{i}.Fc.ID, '.UpperBound'])
    
end



if obj.Problem.Trajectory.Bool
    
    switch obj.Problem.Trajectory.PolyType
        case 'Bezier'
            
            validateattributes(obj.Problem.Trajectory.PolyCoeff.('LowerBound'), {'double'}, {'size',[numel(rbm.Inputs.u.sym),obj.Problem.Trajectory.PolyOrder+1]}, mfilename, [obj.Problem.Trajectory.PolyCoeff.ID, '.LowerBound'])
            validateattributes(obj.Problem.Trajectory.PolyCoeff.('UpperBound'), {'double'}, {'size',[numel(rbm.Inputs.u.sym),obj.Problem.Trajectory.PolyOrder+1]}, mfilename, [obj.Problem.Trajectory.PolyCoeff.ID, '.UpperBound'])

                
            
            
            
        otherwise
            error('Error.\nCurrently supported virtual constraints are of type ''Bezier'', not %s.', obj.Problem.Trajectory.PolyType)
        
    end
end





end