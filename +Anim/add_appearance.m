function body_appearance = add_appearance(body_type, body_prms, joint_prms, varargin)
% function body_appearance = add_appearance(body_type, body_length, body_axis, body_color, joint_size, varargin)
% add apperance to robot link 
%
% body_type: char
%   currently supported body types are 'base', 'slender-rod', and 'reaction-wheel'
%
% body_length: 1x1 double
%
% body_axis: 1x3 double
%   indicate body alignment 
%
% body_color: 1x3 double
%
% joint_size: 1x1 double
%   plot a sphere of radius joint_size at end of link

%% Argument Validation

arguments
    body_type (1,:) char {mustBeMember(body_type, {'base','joint','slender-rod','reaction-wheel'})}    
    body_prms (1,:) cell
    joint_prms (1,:) cell
end

arguments (Repeating)
    varargin (1,1) logical
end

if nargin == 4
    debug_mode = varargin{1};
else
    debug_mode = false;
end


%% Assign body appearance


switch body_type
    case {'base','joint'}
        body_appearance = {};
        
    case 'slender-rod'
      

        body_appearance = {};
        if ~isempty(body_prms)
            for i = 1:numel(body_prms)
                body_i = body_prms{i};


                % validate then extract arguments
                body = extract_prms(body_type, body_i);

                body_appearance = [body_appearance,...
                    'colour', body.color,...
                    'facets', 10,...
                    'cyl', [zeros(1,3); body.length*body.axis], body.radius];

            end
        end


        
    case 'reaction-wheel'
        
        error('Pull this from previous package')
        
        
    otherwise 
        error('MyComponent:incorrectType',...
            'Error. \nBody type must be ''base'', ''joint'', ''slender-rod'', or ''reaction-wheel'', not %s.', body_type)
   
    
end





if ~isempty(joint_prms)
    for i = 1:numel(joint_prms)
        joint_i = joint_prms{i};
        
        fd_names = fieldnames(joint_i);
        for k = 1:numel(fd_names)

           % verify argument
           validate_joint_prms(fd_names{k})

        end

        
        body_appearance = [body_appearance,...
            'colour', .2*ones(1,3),...
            'facets', 15,...
            'sphere', joint_i.distance*joint_i.axis, joint_i.radius];
     
        
    end
end
        
        

if debug_mode
    % add xi-yi-zi coordinate systems for debugging  
    %   xi: blue
    %   yi: green
    %   zi: red    
    body_appearance = [body_appearance,...
        'colour', [0 0 1], 'cyl', [0 0 0; .1 0 0], .005, ...
        'colour', [0 1 0], 'cyl', [0 0 0; 0 .1 0], .005, ...
        'colour', [1 0 0], 'cyl', [0 0 0; 0 0 .1], .005];
  
end
    
  


end


function body = extract_prms(body_type, body_prms)
    if ~isempty(body_prms)
        fd_names = fieldnames(body_prms);
        for i = 1:numel(fd_names)

           % verify argument
           validate_body_prms(body_type,fd_names{i})

           % extract argument
           body.(fd_names{i}) = body_prms.(fd_names{i});

        end
    else
        error('''body_prms'' is a required argument for body type %s.', body_type)
    end

end
     

function validate_body_prms(body_type,arg_type)

switch body_type
    case 'slender-rod'
        arg_type_list = {'length', 'axis', 'color', 'radius'};
        
    case 'reaction-wheel'
       
        
    otherwise
        error('MyComponent:incorrectType',...
            'Error. \nBody type must be ''base'', ''joint'', ''slender-rod'', or ''reaction-wheel'', not %s.', body_type)
   
end

mustBeMember(arg_type, arg_type_list)
  
end


function validate_joint_prms(arg_type)

arg_type_list = {'radius', 'axis', 'distance'};
        
mustBeMember(arg_type, arg_type_list)
  
end
