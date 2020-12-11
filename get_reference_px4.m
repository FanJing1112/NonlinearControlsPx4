function des_state = get_reference_px4(t)
des_x = cos(t); des_xdot = -sin(t);
des_y = sin(t); des_ydot = cos(t);
des_z = -sqrt(t+0.1); des_zdot = -1/(2*(t + 1/10)^(1/2));
des_psi = sin(t+0.1)/(t+0.1); des_psidot = cos(t + 1/10)/(t + 1/10) - sin(t + 1/10)/(t + 1/10)^2;
% des_psi = t; des_psidot = 1;
des_state = [des_xdot;des_x;des_ydot;des_y;des_zdot;des_z;0;0;0;0;des_psidot;des_psi];

%simplified state
% des_state = [des_x;des_y;des_z;des_psi;0;0;des_xdot;des_ydot;des_zdot;...
%     0;0;des_psidot;0;0];
end