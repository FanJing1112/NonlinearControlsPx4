function des_state = get_reference(t,radius)
des_x = radius*cos(t); des_xdot = -radius*sin(t);
des_y = radius*sin(t); des_ydot = radius*cos(t);
des_z = -sqrt(t+0.1); des_zdot = -1/(2*(t + 1/10)^(1/2));
% des_z = -sqrt(t+0.01); des_zdot = -1/(2*(t + 1/100)^(1/2));
% des_z = -sqrt(t+0.5); des_zdot = -1/(2*(t + 5/10)^(1/2)); des_zdot2 = 1/(4*(t + 5/10)^(3/2)); 
des_psi = sin(t+0.1)/(t+0.1); des_psidot = cos(t + 1/10)/(t + 1/10) - sin(t + 1/10)/(t + 1/10)^2;
% des_psi = t; des_psidot = 1;
% des_state = [des_xdot;des_x;des_ydot;des_y;des_zdot;des_z;0;0;0;0;des_psidot;des_psi];
%simplified state
des_state = [des_x;des_y;des_z;des_psi;0;0;des_xdot;des_ydot;des_zdot;...
    0;0;des_psidot;0;0];
end