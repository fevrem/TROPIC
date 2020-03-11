function [nlp, a] = add_bezier_coefficients(nlp, rbm, plt_opt)



% Bezier coefficients
switch nlp.Problem.desired_trajectory.option
    case 'virtual-constraint'
        switch nlp.Problem.desired_trajectory.type
            case 'bezier'

                
                
                
%                 % get seed for Bezier coefficients
%                 if ~isfield( nlp.Seed , 'alpha' ) || nlp.Settings.desired_trajectory.poly_order+1 ~= size(nlp.Seed.alpha,2)
%                     nlp.Seed.alpha = Optim.init_bezier( rbm , [ nlp.Seed.q ; nlp.Seed.qd ] , size(rbm.model.B,2) , nlp.Settings.poly_order+1 , plt_opt );
%                     '1'
%                     warning('reguessing bezier')
%                     '2'
%                 elseif isfield(nlp.Seed,'alpha')
%                     nlp.Seed.alpha = nlp.Seed.alpha;
%                 else
%                     error('fix')
%                 end
% 
% 
% 
% 
%                 if isfield(nlp.Seed,'alpha_lb') && isfield(nlp.Seed,'alpha_ub')
%                     alpha_lb = nlp.Seed.alpha_lb;
%                     alpha_ub = nlp.Seed.alpha_ub;
%                 else
%                     alpha_lb = nlp.Seed.alpha - 5;
%                     alpha_ub = nlp.Seed.alpha + 5;
%                 end
                
                
                % if they were not specified in seed, they are fminunc
                
                

                
                if ~isfield(nlp.Seed,'alpha')
                 %   alpha_guess = nlp.Seed.alpha;
                %else
                    nlp.Seed.alpha = zeros(size(nlp.Problem.desired_trajectory.alpha_lb));
                    
                end
                alpha_guess = nlp.Seed.alpha;
                
                alpha_lb = nlp.Problem.desired_trajectory.alpha_lb;
                alpha_ub = nlp.Problem.desired_trajectory.alpha_ub;
                
                
            % %     if isfield( rbm.model ,'body_type' )
            % % 
            % %         if size( rbm.model.B , 2 ) == 1
            % %             idx_aDOF = find( rbm.model.B == 1);
            % %         else
            % %             idx_aDOF = find( sum(rbm.model.B')' == 1);
            % %         end
            % % 
            % %         for k = 1:numel(idx_aDOF)
            % %             if strcmpi( 'wheel' , rbm.model.body_type{idx_aDOF(k)} )
            % %                 alpha_lb(k,:) = alpha_lb(k,:) - 10;
            % %                 alpha_ub(k,:) = alpha_ub(k,:) + 10;
            % %             end
            % %         end
            % % 
            % %     end



                %seed.alpha_lb = alpha_lb;

                % number of bezier coefficients
                n_coeff = numel(nlp.Seed.alpha);


                % Bezier coefficients
                [nlp , a] = add_var(nlp, 'alpha' , n_coeff, reshape(alpha_guess, n_coeff, 1) , reshape(alpha_lb, n_coeff, 1) , reshape(alpha_ub, n_coeff, 1) , 'alpha');


            otherwise
                
                a = [];

        end
        
    otherwise
        
        a = [];
        
end
    
end
