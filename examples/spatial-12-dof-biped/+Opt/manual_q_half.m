function q_guess_half = manual_q_half()
%{
When t = tf/2;

% ------------------------- intermediate pose --------------------------- %
% just guess acceptable pose

%}

% keep them away from 0

% q_guess_half = [ 0.5 ; % x_stance
%                 -0.6 ; % y_stance
%                 0 ; % z_stance
%                 pi+0.05 ; % phi_z
%                 pi/2+0.1 ; % phi_x
%                 pi-0.05 ; % phi_y
%                 deg2rad(15) ; % q1
%                 -deg2rad(20) ; % q2
%                 pi/2 + pi/65 ; % q3
%                 pi/2 - 0.1 ; % q4
%                 pi/2 + 0.90 ; % q5
%                 pi/8 ]; % q6

q_guess_half = [0 ; % x_stance
                0 ; % y_stance
                0 ; % z_stance
                pi ; % phi_z
                pi/2+pi/35 ; % phi_x
                pi ; % phi_y
                deg2rad(5) ; % q1
                -deg2rad(5) ; % q2
                pi/2+pi/35 ; % q3
                pi/2-pi/35 ; % q4
                pi/2 + pi/2.7 ; % q5
                pi/4.8 ]; % q6
            
end