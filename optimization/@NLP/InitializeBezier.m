function [alpha] = InitializeBezier(obj, rbm, x0, t0)
% n is the number of Bezier coefficient per actuated DOF
% M is the degree of the Bezier curve (M = n-1)
% aDOF is the number of actuated DOF

aDOF = numel(rbm.Inputs.u.sym);
n = obj.Problem.Trajectory.PolyOrder+1;

plt.bool = false;

robot = rbm.Model;

NB = robot.nd;


switch obj.Problem.Trajectory.PolyPhase
    case 'time-based'
        s_seed = (t0 - t0(1))/(t0(end)-t0(1));
                
    case 'state-based'
        s_seed = (robot.c*x0(1:NB,:) - robot.c*x0(1:NB,1))/(robot.c*x0(1:NB,end) - robot.c*x0(1:NB,1));

    otherwise 
        error('''time-based'' and ''state-based'' are the supported options')
end

% has to
mustBePositive(gradient(s_seed, t0))



% could be time based too


if plt.bool
    fig = figure;
    co = Anim.matlab_color();
    [interpreter,linewidth,fontname,fontsize] = Anim.plt_settings();
end



% Degree of Bezier polynomial
M = n - 1;

% pre-allocate
alpha = zeros(aDOF,n);



% find the actuated DOF



if size( rbm.InputMap , 2 ) == 1
    idx_aDOF = find( rbm.InputMap == 1);
else
    idx_aDOF = find( sum(rbm.InputMap')' == 1);
end

if numel(idx_aDOF) ~= aDOF
    error('Inconsistent number of actuated DOF')
end




%[n_rows,n_cols] = Plt.subplot_dim(length(idx_aDOF));
plot_size = Anim.numSubplots(length(idx_aDOF));
n_cols = plot_size(1);
n_rows = plot_size(2);
        
        
    


fprintf('\nBezier parameters seed: fminunc\n') 
for k = 1:aDOF
    
    if plt.bool 
        subplot(n_rows,n_cols,k); hold on
%         plot(s_seed,x0(6+k,:),'linewidth',2)

        plot(s_seed,x0(idx_aDOF(k),:),'linewidth',linewidth)        
        
        
        %ylabel(['x_{' num2str(idx_aDOF(k)) '}'])
        
        %ls /Users/martinfevre/Desktop/'Optimization-Based Robotics'/Project/TOPIC/Anim
        
        Anim.lablz('gait-cycle', s_seed, ['$x_{' num2str(idx_aDOF(k)) '}$'] , interpreter , fontname , fontsize )

        
        
    end
    
    
    for i = 1:n
%         f{k}(i) = interp1( s_seed , x0(6+k,:)' , (i-1)/M );
%         df{k}(i) = interp1( s_seed , x0(6+12+k,:)' , (i-1)/M );
        f{k}(i) = interp1( s_seed , x0(idx_aDOF(k),:)' , (i-1)/M );
        df{k}(i) = interp1( s_seed , x0(idx_aDOF(k)+NB,:)' , (i-1)/M );        
        
                
        if i == 1
            a0 = f{k}(i);
        elseif i == n
            aend = f{k}(i);
        end
    end

    a1 = df{k}(1)*(robot.c*x0(1:NB,end)-robot.c*x0(1:NB,1))/(M*robot.c*x0(NB+1:2*NB,1)) + a0;

    aendminus1 = -df{k}(n)*(robot.c*x0(1:NB,end)-robot.c*x0(1:NB,1))/(M*robot.c*x0(NB+1:2*NB,end)) + aend;

    syms('a_vars',[1,n],'real')
    a_vars(1) = a0;
    a_vars(2) = a1;
    a_vars(end) = aend;
    a_vars(end-1) = aendminus1;
        
      

    bez = 0;
    for m = 0:M
        bez = bez + a_vars(m+1)*factorial(M)/(factorial(m)*factorial(M-m))*s_seed.^m.*(1-s_seed).^(M-m);
    end

    x = a_vars(3:end-2);

%     fh2 = matlabFunction((bez-x0(6+k,:))*(bez-x0(6+k,:))','vars',{x}); 
    fh2 = matlabFunction((bez-x0(idx_aDOF(k),:))*(bez-x0(idx_aDOF(k),:))','vars',{x}); 

    
    
    options = optimoptions('fminunc','Display','none','Algorithm','quasi-newton');

    [xfinal,fval,exitflag,output2] = fminunc(fh2,ones(1,length(x)),options);

    if exitflag
        fprintf(['alpha_' num2str(k) '[0:' num2str(M) '] --> Local minimum found\n'])
    else
        error('fminunc could not find a set of Bezier parameters!')
    end



    for bezier_idx = 3:M-1
        a_vars(bezier_idx) = xfinal(bezier_idx-2);
    end

    a_vars = double(a_vars);

    if plt.bool 
        bez_num = 0;
        for m = 0:M
            bez_num = bez_num + a_vars(m+1)*factorial(M)/(factorial(m)*factorial(M-m))*s_seed.^m.*(1-s_seed).^(M-m);
        end
        
        
        plot(s_seed,bez_num,'linewidth',linewidth)
        
        scatter(0,a_vars(1),'ko','filled')
        for idx = 1:M
            scatter(idx/M,a_vars(idx+1),'ko','filled')
        end
        

        plot([0,1/M],[a_vars(1),a_vars(2)],'m:','LineWidth',linewidth)
        plot([(M-1)/M,M/M],[a_vars(end-1),a_vars(end)],'m:','LineWidth',linewidth)
        
        grid on
    end


    alpha(k,:) = a_vars;

end




if plt.bool 
%     supt = suptitle('');
%     supt.FontWeight = 'bold';
    
    fig.NumberTitle = 'off';
    fig.Name = 'Bezier Coefficients (Seed)';
    
    
end


end

