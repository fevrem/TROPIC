function [ Jc , Jcd ] = get_jacobian( model , q , qd )


% propagate kinematics & solve IK for impact relabelling
[ Tr , ~ , ~ ,~ ] = prop_kin( model, q );




R_0_to_1  = Tr{1}(1:3,1:3);
R_0_to_2  = Tr{2}(1:3,1:3);
R_0_to_3  = Tr{3}(1:3,1:3);
R_0_to_4  = Tr{4}(1:3,1:3);     
R_0_to_5  = Tr{5}(1:3,1:3);     
R_0_to_6  = Tr{6}(1:3,1:3);

R_4_to_4 = R_0_to_4 * R_0_to_4';
R_5_to_4 = R_0_to_4 * R_0_to_5';
R_6_to_4 = R_0_to_4 * R_0_to_6';


% angular velocity of body frame in {4} due to phi_z_dot
w1 = simplify(R_4_to_4) * [ 0 ; 0 ; qd(4)];

% angular velocity of {b} in {4} due to phi_x_dot
w2 = simplify(R_5_to_4) * [ 0 ; 0 ; qd(5)];

% angular velocity of {b} in {4} due to phi_y_dot
w3 = simplify(R_6_to_4) * [ 0 ; 0 ; qd(6)];

% total angular velocity of {b} in {4}
w_b_in_4 = w1 + w2 + w3;

Jc = [ eye(3) , zeros(3,9) ;
       jacobian( w_b_in_4(3) , qd ) ];
   
Jcd = [ jacobian( Jc(1,:) * qd , q ) ; 
        jacobian( Jc(2,:) * qd , q ) ;
        jacobian( Jc(3,:) * qd , q ) ;
        jacobian( Jc(4,:) * qd , q ) ];




    
    


end