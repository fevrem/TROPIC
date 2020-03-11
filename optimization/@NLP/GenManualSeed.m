function [q_seed,qd_seed,qdd_seed,t_seed] = GenManualSeed(obj, rbm)

arguments 
    obj (1,1) NLP
    rbm (1,1) DynamicalSystem
end
    

% Guess initial pose
[qGuess, tGuess] = Opt.ManualInit();


validateattributes(tGuess, {'double'}, {'size', [1,1]})
mustBePositive(tGuess)

validateattributes(qGuess, {'double'}, {'size', [numel(rbm.States.q.sym),3]})

ncp = obj.Settings.ncp;

t_seed = linspace(0,tGuess,ncp);


t_seed1 = t_seed(1:ceil(ncp/2));
[q_start2half, qd_start2half] = InterpSeed1(obj, t_seed1, qGuess(:,1), qGuess(:,2));

% interpolate from t_half to t_end

t_seed2 = t_seed(ceil(ncp/2):end);
[q_half2end, qd_half2end] = InterpSeed2(obj, t_seed2, qGuess(:,2), qGuess(:,3));%, ncp,t_guess,q_half,q_end);

q_seed = [q_start2half, q_half2end(:,2:end)];
qd_seed = [qd_start2half, qd_half2end(:,2:end)];

validateattributes(q_seed, {'double'}, {'size',[numel(rbm.States.q.sym), ncp]})        
validateattributes(qd_seed, {'double'}, {'size',[numel(rbm.States.q.sym), ncp]})        


qdd_seed = zeros(numel(rbm.States.q.sym),ncp);
for i = 1:numel(rbm.States.q.sym)
    qdd_seed(i,:) = gradient(qd_seed(i,:),t_seed);
end



return

Anim.animate(rbm,t_half2end,q_half2end,anim_options)
        
        
       
% B may not be square and invertible 
rbm.InputMap*rbm.InputMap'



%tGuess = linspace(0,tfGuess,obj.Settings.ncp);



fkmfr

%%
t0 = 0;






Anim.animate(rbm,t_half,q_half,anim_options)


             % intermediate pose
        q_half = Opt.manual_q_half;
        t_half = t_guess(floor(ncp/2)+1);
        Anim.animate(rbm,t_half,q_half,anim_options)
        
        
        
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
