function seed = gen_manual_seed(obj,rbm,ncp,tf_guess)

% if strcmpi(class(rbm.model.q),'sym') == false
%     if nargin == 4
%     
%         casadi = varargin{1};
%         
%     end
% end

% animation options (use this to debug seed)
anim_options.bool = false;

anim_options.skip_frame    = 1;
anim_options.save_movie    = false;
anim_options.movie_name    = [rbm.name , '.mp4'];
anim_options.movie_quality = 100; % scalar between [0 100], default 75
anim_options.movie_fps     = 30;  % frame rate, default 30

% views to show. Options: {'3D','frontal','sagittal','top'}
anim_options.views = {'3D','frontal','sagittal','top'};
%anim_options.views = {'3D','sagittal'};


% create a light object or not
anim_options.lights = true;

%anim_options.axis.x = [-0.5 1.5];
%anim_options.axis.y = [-1.0 1.0];
%anim_options.axis.z = [0 1.5];


%anim_options_final.axis = anim_options.axis;


% % number of collocation points
% ncp = 101;
% 
% % final time
% tf_guess = 1;




% time
t_guess = linspace(0,tf_guess,ncp);




switch rbm.dynamics
    
    case 'hybrid'

        % intermediate pose
        q_half = Opt.manual_q_half;
        t_half = t_guess(floor(ncp/2)+1);
        Anim.animate(rbm,t_half,q_half,anim_options)

        % final pose
        q_end = Opt.manual_q_end;
        t_end = t_guess(ncp);
        Anim.animate(rbm,t_end,q_end,anim_options)

        % interpolate from t_half to t_end
        [q_half2end,qd_half2end,t_half2end] = Optim.interp_half2end(rbm.model,ncp,t_guess,q_half,q_end);
        Anim.animate(rbm,t_half2end,q_half2end,anim_options)

        % get x_minus equivalent
        %[q_minus, qd_minus] = Model.get_x_minus( rbm );

%         if strcmpi(class(rbm.model.q),'sym') == true
% 
%             % get x_minus equivalent
%             [q_minus, qd_minus] = Model.get_x_minus( rbm );
% 
%             q_minus  = double( subs( q_minus   , sym2cell(rbm.model.q)' , num2cell(q_end)'));
%             qd_minus = double( subs( qd_minus  , sym2cell([rbm.model.q;rbm.model.qd])' , num2cell([q_end;qd_half2end(:,end)])'));
% 
%         else

            import casadi.*

            q_minus  = obj.Functions.q_minus( q_end );
            qd_minus = obj.Functions.qd_minus( q_end , qd_half2end(:,end) );


            q_minus  = casadi2double(q_minus);
            qd_minus = casadi2double(qd_minus);

            % ENFORCE PERIODICITY ON POSITION OF STANCE KNEE INSTEAD
            if strcmpi(rbm.name,'Spatial 5-Link Biped')
                if sign(q_minus(4)) == sign(q_end(4))
                    q_minus(4) = -q_minus(4);
                end
            end

            
%         end

        % flip x_minus
        [q_minus_flipped, qd_minus_flipped] = Model.flip_map( rbm , q_minus , qd_minus );
        if strcmpi(class(rbm.model.q),'sym') == true
            q_minus_flipped  = double(q_minus_flipped);
            qd_minus_flipped = double(qd_minus_flipped);
        else
            if strcmpi(class(q_minus_flipped),'double') == false
                q_minus_flipped  = casadi2double(q_minus_flipped);
                qd_minus_flipped = casadi2double(qd_minus_flipped);    
            end
        end




        % impact
        % [q_plus,qd_plus] = Model.apply_impact( rbm , q_minus_flipped , qd_minus_flipped , 'numeric' );
        [q_plus,qd_plus] = Sim.apply_impact( rbm , q_minus_flipped , qd_minus_flipped , 'numeric' );
        
        switch rbm.dimensions
            case '2D'
        %if strcmpi(rbm.name(1:6),'planar')
                q_plus(1:2) = q_end(1:2);
            
            case '3D'
        %elseif strcmpi(rbm.name(1:7),'spatial')
                q_plus(1:3) = q_end(1:3);
        %end

            otherwise 
                error('not supported')
        end

        % for 3D biped, this should give me the position
        % of the stance knee w.r.t the stance foot
        % used for periodicity on stance angles

%
%
%
%
%
%
%{
        p_st_foot_end = casadi.functions.p_stance_foot([ q_end ; zeros(12,1) ]);
        p_st_foot_minus = casadi.functions.p_stance_foot([ q_minus ; zeros(12,1) ]);
        p_st_foot_fl = casadi.functions.p_stance_foot([ q_minus_flipped ; zeros(12,1) ]);
        p_st_foot_plus = casadi.functions.p_stance_foot([ q_plus ; zeros(12,1) ]);

        p_st_knee_end = casadi.functions.p_stance_knee([ q_end ; zeros(12,1) ]);
        p_st_knee_minus = casadi.functions.p_stance_knee([ q_minus ; zeros(12,1) ]);
        p_st_knee_fl = casadi.functions.p_stance_knee([ q_minus_flipped ; zeros(12,1) ]);
        p_st_knee_plus = casadi.functions.p_stance_knee([ q_plus ; zeros(12,1) ]);

        p_st_knee_wrt_end = casadi.functions.p_stance_knee_wrt_foot([ q_end ; zeros(12,1) ]);
        p_st_knee_wrt_minus = casadi.functions.p_stance_knee_wrt_foot([ q_minus ; zeros(12,1) ]);
        p_st_knee_wrt_fl = casadi.functions.p_stance_knee_wrt_foot([ q_minus_flipped ; zeros(12,1) ]);
        p_st_knee_wrt_plus = casadi.functions.p_stance_knee_wrt_foot([ q_plus ; zeros(12,1) ]);
%}
%
%
%
%
%
%


        % interpolate from t_half to t_end
        [q_start2half,qd_start2half,t_start2half] = Optim.interp_start2half(rbm.model,ncp,t_guess,q_plus,q_half);
        Anim.animate(rbm,t_start2half,q_start2half,anim_options)

        %'q- and q+'
        %[q_plus,q_end]

        % Compare v_com
        [x_arg(1:2*rbm.model.NB).val] = numToCell( [q_minus;qd_minus] );
        v_com_minus = v_com(x_arg(:).val);
        [x_arg(1:2*rbm.model.NB).val] = numToCell( [q_minus_flipped;qd_minus_flipped] );
        v_com_minus_flipped = v_com(x_arg(:).val);
        %[v_com_minus,v_com_minus_flipped]

        % seed: full step
        t_guess  = [ t_start2half  , t_half2end(2:end)];
        q_guess  = [ q_start2half  , q_half2end(:,2:end)  ];
        qd_guess = [ qd_start2half , qd_half2end(:,2:end) ];



    case 'continuous'
        
        window_width = 1.1*sum([rbm.model.l{:}]);
        
        % set axes if provided (p_com.mex otherwise)
        anim_options.axis.x = [-window_width window_width];
        anim_options.axis.y = [-window_width window_width];
        anim_options.axis.z = [-window_width window_width];


        % start pose
        q_start = Opt.manual_q_start;
        t_start = t_guess(1);
        Anim.animate( rbm , t_start , q_start , anim_options );

        % final pose
        q_end = Opt.manual_q_end;
        t_end = t_guess(ncp);
        Anim.animate( rbm , t_end , q_end , anim_options )

        % interpolate from t_half to t_end
        %[q_half2end,qd_half2end,t_half2end] = Optim.interp_half2end(rbm.model,ncp,t_guess,q_half,q_end);
        %Anim.animate(rbm,t_half2end,q_half2end,anim_options)
        
        
        % ------------------ interpolate from 0 to T -------------------- %
        q_guess  = zeros(rbm.model.NB,ncp);
        qd_guess = zeros(rbm.model.NB,ncp);
        for m = 1:rbm.model.NB
            q_guess(m,:)  = q_start(m) + t_guess/t_end*(q_end(m)-q_start(m));
            qd_guess(m,:) = 1/t_end*(q_end(m)-q_start(m)) * ones(1,length(t_guess));
        end

        
    otherwise
        
        error('not supported')

end


seed.t = t_guess;
seed.q = q_guess;
seed.qd = qd_guess;


                

end
