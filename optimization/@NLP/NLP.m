classdef NLP %< handle
    % NonlinearProgram defines an abstract class for general nonlinear
    % programing problems
    % 
    %
    % @author Ayonga Hereid @date 2016-10-26
    % 
    % Copyright (c) 2016, AMBER Lab
    % All right reserved.
    %
    % Redistribution and use in source and binary forms, with or without
    % modification, are permitted only in compliance with the BSD 3-Clause 
    % license, see
    % http://www.opensource.org/licenses/bsd-license.php
    
    
    %% Protected properties
    properties (SetAccess=immutable, GetAccess=public)
        % The name of the problem
        %
        % @type char
        %Type
        
        Settings
        
        
        
    end
    
    
    properties (SetAccess=protected , GetAccess=public)
    
        
        Functions
        
        %Objective 
        
        Problem
        
        %Trajectory
        
        %Seed
        
        Vars
        
        VarsName
        
        VarsInit
        
        VarsLB
        
        VarsUB
        
        VarsIdx
        
        Constr
        
        ConstrName
        
        ConstrLB
        
        ConstrUB
        
        Cost
        
        
        
    end
    
    
    properties (SetAccess=public, GetAccess=public)
        
        % The class option
        %
        % Required fields of options:
        %  DerivativeLevel: the user-defined derivative order (0, 1 or 2)
        %  to be used by a NLP solver. @type integer @default 1
        %  EqualityConstraintBoundary：a relaxation factor number for equality
        %  constraints. @type double
        % 
        % @type struct
        %Options 
        
        %Settings
        
        % An array contains all information regarding NLP
        % optimization variables
        %
        % @type NlpVariable
        %VariableArray
        
                
        % An array data stores objective functions
        %
        % @type NlpFunction
        %CostArray
        
        % A cell array data stores all constraints functions
        %
        % @type NlpFunction
        %ConstrArray
        
        
        
    end
    
    
    
    
    
    properties 
        % The solution of the NLP problem
        Sol
    end
    
    
    %% Public methods
    methods
        
        function obj = NLP(rbm, varargin)
            % The default class constructor function
            %
            % Parameters: 
            %  name: the name of the problem
            
            % required/supported functions
            
            arguments
               rbm (1,1) DynamicalSystem 
            end
            arguments (Repeating)
                varargin (1,2) cell
            end
            
%             obj.Functions.DynamicsODE = [];
%             obj.Functions.ConstrainedDynamicsODE = [];
%             obj.Functions.InertiaMatrix = []; % needed for impact equation
%             obj.Functions.ContactJacobian = []; % needed for impact equation


            %if nargin == 0
                % just return the constructed object (called from MultiphaseNLP)
            
            obj.Vars = {};
            obj.VarsName = {};
            obj.VarsIdx = {};
            obj.Constr = {};
            obj.ConstrName = {};
            
            % supported functions (set by ConfigFunctions)
            %obj.Functions = ParseFunctions();
            obj.Functions = UserFunctions();    
            
            if nargin <= 1
                return;
            else    
                for i = 1:nargin-1
                    tempArg = varargin{i};
                    argIn.(tempArg{1}) = tempArg{2};
                end
            end


            if isfield(argIn, 'NFE')
                validateattributes(argIn.NFE, {'double'}, {'nonnegative'})
                obj.Settings.nfe = argIn.NFE;
            end
            
            if isfield(argIn, 'CollocationScheme')                
                validateattributes(argIn.CollocationScheme, {'char'}, {'size',[1,NaN]})
                mustBeMember(argIn.CollocationScheme, {'HermiteSimpson','Trapezoidal'})
                obj.Settings.CollocationScheme = argIn.CollocationScheme;
            end
            
            if isfield(argIn, 'LinearSolver')                
                validateattributes(argIn.LinearSolver, {'char'}, {'size',[1,NaN]})
                mustBeMember(argIn.LinearSolver, {'ma27','ma57','mumps'})
                obj.Settings.LinearSolver = argIn.LinearSolver;
            end
                        
            
            if isfield(argIn, 'ConstraintTolerance')                
                validateattributes(argIn.ConstraintTolerance, {'double'}, {'nonnegative'})
                obj.Settings.ConstraintTolerance = argIn.ConstraintTolerance;
            end
            
            
 

            switch obj.Settings.CollocationScheme
                case 'Trapezoidal'
                    obj.Settings.ncp = obj.Settings.nfe + 1;
                case 'HermiteSimpson'
                    obj.Settings.ncp = 2*obj.Settings.nfe + 1;
                otherwise
                    error('Error.\n%s is not a currently supported collocation method.\nOptions are ''Trapezoidal'' and ''HermiteSimpson''.', obj.Settings.CollocationScheme)
            end
                
                
 
            obj.Problem = ConstraintList();
            
            return
            
            
            obj.Objective = objective_fun(obj);
            
            
            obj.Problem = Opt.load_constraints(obj, rbm);
            
            obj.Vars = {};
            obj.VarsName = {};
            obj.VarsIdx = {};

            obj.Constr = {};
            
            obj.ConstrName = {};
            
            
            % settings  
            %obj.Options = 
            % default options
            %obj.Options = struct('DerivativeLevel', 1, 'EqualityConstraintBoundary', 0);
            
            
            
            
        end
        
    end
    
    %% Function definitions
    methods
        
%         [obj] = configure_functions(obj, rbm);
        [obj] = ConfigFunctions(obj, rbm);    
        
        [obj] = AddVirtualConstraints(obj, rbm);
        
        [obj] = ParseNLP(obj, rbm);
        
        %[obj, phase_var] = AddPhaseVariable(obj, rbm, t, T)

        [obj, rbm] = LoadConstraints(obj, rbm);
        
        %[obj] = LoadConstraints(obj, rbm, varargin)
        
        
        
        [funs] = extra_functions(obj,rbm);
        
        [obj] = load_seed(obj, rbm, seed, varargin);
        
        [seed] = gen_manual_seed(obj, rbm, ncp, tf);
        
        [obj, t_minus] = add_phase_variable(obj, rbm, tf, T)

        %[obj] = load_constraints(obj, rbm);
        
        %obj.Settings.obj_fun = objective_fun(obj);

        [obj] = configure_nlp(obj,rbm,varargin)
                
        [obj,varName] = add_var(obj,name,len,init,lb,ub,varargin)
        
        %[obj, running_cost] = integrate_FE(obj, rbm, h, running_cost, cost_k, cost_previous, q_k, dq_k, x_previous, f_k, f_previous, u_k, u_previous, du_k, du_previous, idxArg )
        [obj, running_cost] = integrate_FE(obj, rbm, h, running_cost, cost_k, cost_previous, states_k, states_previous, control_k, control_previous, f_k, f_previous, idxArg )
  
        [obj,states] = add_states(obj,rbm,idxArg)
        
        [obj] = add_constraint(obj,constr, lb, ub, varargin)
        
        [obj,control] = add_control(obj,rbm,idxArg)
        
        [obj] = monotonic_phase_var(obj, rbm, states, idxArg)        
        
        [obj] = initial_pose(obj, rbm, states, idxArg );

        [obj] = final_pose(obj, rbm, states, idxArg );
        
        [obj] = enforce_phase_var(obj, rbm, q_0, q_k, t_plus, t_minus)

        [obj] = torso_constraints(obj, rbm, states, idxArg )

        [obj] = swing_foot_constraints(obj, rbm, states, idxArg )
            
        [obj] = stance_foot_constraints(obj, rbm, states, idxArg )

    
        [obj, s_k] = compute_phase_var(obj, rbm, q_k, t_k, t_minus, t_plus);
    
        [obj,force] = add_contact_force(obj,rbm,idxArg)
        
        [obj, coeff] = add_bezier_coefficients(obj, rbm, plt_opt)

        [obj] = enforce_friction(obj, rbm, force)

        [obj] = enforce_dynamics(obj, rbm, states, control, idxArg )

        [obj] = enforce_hybrid_invariance(obj, rbm, states, coeff, t_plus, t_minus , idxArg )

        [obj] = enforce_ZD_invariance(obj, rbm, states, a, s, t_minus, t_plus, idxArg)
 
        [obj] = add_torso(obj, rbm, pos, vel, idxArg)
        
        [obj, slewrate] = add_slew_rate(obj, rbm, idxArg )

        [obj] = step_variables(obj, rbm, states, tf, idxArg)

        [obj,q_minus,qd_plus] = apply_impact(obj, rbm, q_minus, qd_minus)

        [obj] = one_step_periodic(obj, rbm, states_0, states, idxArg)
  
        [obj] = enforce_periodicity(obj, rbm, x0, xPlus)

        %[obj] = final_cost(obj, rbm, pos, vel, acc, control)
        running_cost = final_cost(obj, rbm, running_cost, states, control, tf)%q, qd, qdd, u, ud, tf)

        [sol,info] = solve_nlp(obj, rbm)
        
            
        
        
            
        
        
        [obj] = regVariable(obj, vars);
        
        [obj] = update(obj);
        
        [obj] = regObjective(obj, funcs);
        
        [obj] = regConstraint(obj, funcs);
        
        [nVar, lowerbound, upperbound] = getVarInfo(obj);
        
        [x0] = getInitialGuess(obj, method);
        
        obj = setOption(obj, varargin);
       
        compileConstraint(obj, export_path, varargin);
        
        compileObjective(obj, export_path, varargin);
    end
        
    
end

