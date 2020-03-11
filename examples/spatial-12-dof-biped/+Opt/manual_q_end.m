function q_guess_end = manual_q_end()

% ----------------------------- final pose ------------------------------ %
% just guess acceptable pose using animation option
q_guess_end = [ 0 ; % x_stance
                0 ; % y_stance
                0 ; % z_stance
                pi ; % phi_z
                pi/2+pi/35 ; % phi_x
                pi-deg2rad(10) ; % phi_y
                deg2rad(15) ; % q1
                -deg2rad(5) ; % q2
                pi/2+pi/35 ; % q3
                pi/2-pi/35 ; % q4
                pi/2 + pi/2.4 ; % q5
                pi/12 ]; % q6=   
            
end