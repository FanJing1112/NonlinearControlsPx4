function [des_state,des_accel] = get_reference_z(t,radius)
des_x = radius*cos(t); des_xdot = -radius*sin(t); des_xdot2 = -radius*cos(t); des_xdot3 = radius*sin(t); des_xdot4 = radius*cos(t);
des_y = radius*sin(t); des_ydot = radius*cos(t);  des_ydot2 = -radius*sin(t); des_ydot3 = -radius*cos(t); des_ydot4 = radius*sin(t);
des_z = -sqrt(t+0.1); des_zdot = -1/(2*(t + 1/10)^(1/2)); des_zdot2 = 1/(4*(t + 1/10)^(3/2)); 
des_zdot3 = -3/(8*(t + 1/10)^(5/2)); des_zdot4 = 15/(16*(t + 1/10)^(7/2));

% des_z = -sqrt(t+0.5); des_zdot = -1/(2*(t + 5/10)^(1/2)); des_zdot2 = 1/(4*(t + 5/10)^(3/2)); 
% des_zdot3 = -3/(8*(t + 0.5)^(5/2));des_zdot4 = 15/(16*(t + 0.5)^(7/2));
des_psi = sin(t+0.1)/(t+0.1); des_psidot = cos(t + 1/10)/(t + 1/10) - sin(t + 1/10)/(t + 1/10)^2;
% des_psidot2 = (2*sin(t + 1/10))/(t + 1/10)^3 - sin(t + 1/10)/(t + 1/10) - (2*cos(t + 1/10))/(t + 1/10)^2; 
% des_psi = t; des_psidot = 1;
% %%%%%%%% Comment in to write the function for Px4%%%%%%%%%%%%%
% while(des_psi>pi)
%     des_psi = des_psi - 2*pi;
% end
% %%%%%%%%%%%%%%%%%%%%%%%%%
des_psidot2 = 0; 
% des_state = [des_xdot;des_x;des_ydot;des_y;des_zdot;des_z;0;0;0;0;des_psidot;des_psi];
%simplified state
des_state = [des_x;des_xdot;des_xdot2;des_xdot3;
            des_y;des_ydot;des_ydot2;des_ydot3;
            des_z;des_zdot;des_zdot2;des_zdot3
            des_psi;des_psidot];
        des_accel = [des_xdot4;des_ydot4;des_zdot4;des_psidot2];
end