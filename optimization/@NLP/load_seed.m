function obj = load_seed(obj, rbm, seed_arg, varargin)
    



seed_opt = struct();

for i = 1:length(varargin)
    seed_opt.(varargin{i}{1}) = varargin{i}{2};
end

if ~isfield(seed_opt,'plot')
    seed_opt.plot = false;
end

% if isfield(seed_opt,'ncp')
%     ncp = seed_opt.ncp;
% else
%     ncp = 25;
% end

ncp = obj.Settings.ncp;



if isempty(seed_arg)
    


        
        
        fprintf('\n')
        dline(1,'-')
        fprintf('\nGenerate seed manually...')
        fprintf('\n')
        dline(1,'-')
        fprintf('\n')
        
        
    


        
        
        %rbm = varargin{1};

        %seed_arg = varargin{2};

        %ncp_arg = varargin{3};

        %tf_arg = varargin{4};

        %casadi = varargin{5};

        %nlp = varargin{6};

        %plt_opt = varargin{7};


        
        
        
        
        
        if isfield(seed_opt,'final_time')
            tf = seed_opt.final_time;
        else
            tf = 1;
        end

        
        %if nargin == 5
        %    casadi = varargin{3};
        
%         seed = Optim.gen_manual_seed( obj, rbm , ncp , tf , casadi );
        seed = gen_manual_seed(obj, rbm , ncp , tf );

        
        %else
        %    seed = Optim.gen_manual_seed(rbm,ncp,tf_guess);
        %end

        for i = 1:rbm.model.NB
            seed.qdd(i,:) = gradient( seed.qd(i,:) , seed.t );
        end
        
        
        
        

 %   otherwise
else      


        %else % use the seed provided

        file2load = seed_arg

        fprintf('\n')
        dline(1,'-')
        fprintf('\nLoad existing seed:\n\t')
        fprintf(file2load)
        fprintf('\n')
        dline(1,'-')
        fprintf('\n')

        gait_data = load([file2load '.mat'])

        %gait_data
        
        %loaded_seed = load('seed_FROST.mat')
        %loaded_seed = load([gait_data '.mat'])

        fdnames = fieldnames(gait_data)
        
        seed = gait_data.(fdnames{1})


        %seed = gait_data.optimal_solution;

        %ncp = varargin{3};

        % modify number of collocation points ( can both reduce or increase ncp )
        % only possible when passing a save seed
        if size(seed.q,2) ~= ncp
            t_new = linspace(seed.t(1),seed.t(end),ncp);
            for i = 1:size(seed.q,2)
                q_new(i,:)  = interp1( seed.t , seed.q(:,i)' , t_new );
                qd_new(i,:) = interp1( seed.t , seed.qd(:,i)' , t_new );
            end

            if isfield(seed,'qdd')
                for i = 1:size(seed.qdd,2)
                    qdd_new(i,:)  = interp1( seed.t , seed.qdd(:,i) , t_new );
                end
            end

            if isfield(seed,'u')
                for i = 1:size(seed.u,2)
                    u_new(i,:)  = interp1( seed.t , seed.u(:,i)' , t_new );
                end
            end

            if isfield(seed,'du')
                for i = 1:size(seed.du,2)
                    du_new(i,:)  = interp1( seed.t , seed.du(:,i)' , t_new );
                end
            end

            if isfield(seed,'Fc')
                for i = 1:size(seed.Fc,2)
                    Fc_new(i,:)  = interp1( seed.t , seed.Fc(:,i)' , t_new );
                end
            end

            seed.t   = t_new;
            seed.q   = q_new;
            seed.qd  = qd_new;

            if exist('qdd_new','var')
                seed.qdd  = qdd_new;
            end
            if exist('u_new','var')
                seed.u  = u_new;
            end
            if exist('du_new','var')
                seed.du  = du_new;
            end        
            if exist('Fc_new','var')
                seed.Fc  = Fc_new;
            end  

        end


        if ~isfield(seed,'du') && isfield(seed,'u') 
            for i = 1:size(seed.u,2)
                seed.du(i,:) = gradient( seed.u(:,i)', seed.t );
            end
        end

        if ~isfield(seed,'qdd') && isfield(seed,'qd') 
            for i = 1:size(seed.qd,2)
                seed.qdd(i,:) = gradient( seed.qd(:,i)', seed.t );
            end
        end

        % change order of bezier polynomial if not correct
        switch obj.Problem.desired_trajectory.option
            case 'virtual-constraint'
                
                switch obj.Problem.desired_trajectory.type
                    case 'bezier'
                        
                        if isfield(seed,'alpha')
                            if size(seed.alpha,2) ~= obj.Problem.desired_trajectory.order+1
                                seed.alpha = Optim.init_bezier( rbm , [ seed.q' ; seed.qd' ] , size(rbm.model.B,2) , obj.Problem.desired_trajectory.order+1 , seed_opt.plot );
                            end
                        end
                        
                    otherwise
                        error('only bezier so far')
                end
        end
        
        
        
        
        if 1
            warning('overwriting seed')
            % overwitee with new modle of FROST
            
            seed.q(1,1:ncp) = 0*ones(1,ncp);
            seed.q(2,1:ncp) = 1.25*ones(1,ncp);
            %seed.q(3,1:ncp) = (0-pi/2)*ones(1,ncp);
            seed.q(3,1:ncp) = (0)*ones(1,ncp);
            seed.q(4,1:ncp) = 3.5*ones(1,ncp);
            seed.q(5,1:ncp) = 1.25*ones(1,ncp);
            seed.q(6,1:ncp) = 3.5*ones(1,ncp);
            seed.q(7,1:ncp) = 1.25*ones(1,ncp);
            
            seed.qd = zeros(7,ncp);
            seed.qdd = zeros(7,ncp);
            
            seed.u = zeros(4,ncp);
            seed.du = seed.u;
            
            seed.Fc = 100*ones(2,ncp);
            
            seed.alpha = zeros(4,6);
            
            warning('dont forget torso offset')
        
        end
        
        
end

obj.Seed = seed;




% animate it here
anim_options.bool = seed_opt.plot;
anim_options.views = {'3D','frontal','sagittal','top'};
anim_options.skip_frame = 1;
anim_options.save_movie = false;
anim_options.lights = true;

% anim_options.movie_name    = 'seed.mp4';
% anim_options.movie_quality = 100; % scalar between [0 100], default 75
% anim_options.movie_fps     = 30;  % frame rate, default 30


switch rbm.dynamics
    case 'continuous'
        
        window_width = 1.1*sum([rbm.model.l{:}]);
        
        % set axes if provided (p_com.mex otherwise)
        anim_options.axis.x = [-window_width window_width];
        anim_options.axis.y = [-window_width window_width];
        anim_options.axis.z = [-window_width window_width];
        
end

Anim.animate( rbm , seed.t , seed.q , anim_options )


    

end