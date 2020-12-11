close all; clear all;
nstates = 12;
ninputs =4;
J_x = 0.05; J_y = 0.05; J_z = 0.09; m = 0.8; g = 9.8;
params.radius = 1;
params.parameters = [J_x J_y J_z m g];
% des_state = [0;0;0;1;2;3;pi/3;pi/3;pi/6;0;0;0];
% [u, p_n, v, p_e, w, h, p, \phi, q, \theta, r, \psi]
% des_state = [0;0;0;0;0;-1;0;0;0;0;0;0];
% gravity_pwm = m*g/k_1/4;
% des_input = [0;0;0;0];
% des_input = gravity_pwm*ones(4,1);
Q_trans = eye(6);
Q_rot = 0.005*eye(6);
% Q_trans = 0.05*eye(6);
% Q_rot = 0.005*eye(6);
Q=blkdiag(Q_trans,Q_rot);
% Q = 0.01*eye(12);
% % Q = 0.1*eye(12);
% % Q(3,3) = 10;
% % Q(4,4) = 10;
% Q(8,8) = 0.0001;
Q(7,7) = 0.05;

Q(9,9) = 0.05;

R = 0.1*eye(4);

des_input(1) = m*g;

[A,B,C,K] = calc_normal_form_mats;
% params.gains = K;
% params.des_state = des_state;
% params.des_input = des_input;
% states = [X Y Z psi theta phi X_dot Y_dot Z_dot zeta eta psi_dot theta_dot phi_dot]
x0 = zeros(14,1);
% % initialize with some x y error
% x0(1) = -3;
% x0(2) = -2; 
x0(10) = m*g; % initialize with initial thrust counteract gravity
end_time = 25;
timespan = 0:0.1:25;
[tout,xout] = ode45(@(t,x)dynamics_feedback_linearization(t,x,params),timespan,x0);
des_states = zeros(size(xout));
uout = [];
for ii = 1:numel(tout)
   des_states(ii,:) = get_reference(tout(ii),params.radius)'; 
   [dxdt,u] = dynamics_feedback_linearization(tout(ii),xout(ii,:)',params);
   uout = [uout;u'];
end

%%%%%%%%%%%%%%% LOAD IN STANDARD LQR OUTPUT %%%%%%%%%%%%%%%%%%%%%
lqr_output = load("lqr_tracking_output_matlab.mat");

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% uout = [];
% for ii = 1:size(xout,1)
%     [dxdt,u] = dynamics_output([],xout(ii,:)',params);
%     uout = [uout;u'];
% end
state_titles = ["MATLAB: North Position Time Series and Error","MATLAB: East Position Time Series and Error","MATLAB: Altitude Time Series and Error ","MATLAB: Yaw Time Series and Error"];
% state_titles = ["North Position Time Series and Error","East Position Time Series and Error","Altitude Time Series and Error ","Yaw Time Series and Error"];
state_labels = ["$p_x$ [m]","$p_y$ [m]","$p_z$ [m]","$\psi$ [rad]"];
error_labels = ["$e_x$ [m]","$e_y$ [m]","$e_z$ [m]","$e_\psi$ [rad]"];
px4_notation = [2,4,6,12];
for ii = 1:4
% figure(ii)
figure('Renderer', 'painters', 'Position', [10 10 800 900])
subplot(2,1,1)
h1 = plot(tout,des_states(:,ii),'k')
% PLOT NORMAL LQR TRAJ
ylabel(state_labels(ii));
hold on
h2 = plot(tout,xout(:,ii),'b--')
hold on
h3 =plot(lqr_output.tout,lqr_output.xout(:,px4_notation(ii)),'r--')
title(state_titles(ii));
subplot(2,1,2)

h1 = plot(tout,des_states(:,ii)-des_states(:,ii),'k-')
hold on
h2 = plot(tout,des_states(:,ii)-xout(:,ii),'b--')
hold on
h3 = plot(lqr_output.tout,des_states(:,ii)-lqr_output.xout(:,px4_notation(ii)),'r--')
% PLOT NORMAL LQR ERROR
xlabel('Time [s]');
ylabel(error_labels(ii));
legend([h1,h2,h3],"Desired Trajectory","Feedback Linearization Controlller","LQR Controller");
end


figure(5)
h1 = plot3(des_states(:,1),des_states(:,2),-des_states(:,3),'k');
hold on
h2 = plot3(xout(:,1),xout(:,2),-xout(:,3),'b--');
hold on
h3 = plot3(lqr_output.xout(:,px4_notation(1)),lqr_output.xout(:,px4_notation(2)),-lqr_output.xout(:,px4_notation(3)),'r--')
grid on
xlabel("x pos [m]");
ylabel("y pos [m]");
zlabel("z pos [m]");
% axis([-5,5,-5,5,0,15]);
title("MATLAB: Translational Tracking");
axis([-5,5,-5,5,0,8]);
skip = 2;
% legend([h1,h3],"Desired Trajectory","LQR Controller");
legend([h1,h2,h3],"Desired Trajectory","Feedback Linearization Controlller","LQR Controller");

figure(6)
title_input_vec = ["$F_z$","$\tau_\phi$","$\tau_\theta$","$\tau_\psi$"];
for ii = 1:4
   subplot(1,4,ii);
   
   h1 = plot(tout,uout(:,ii),'b')
   hold on
   
   h2 = plot(tout,lqr_output.uout(:,ii),'r--')
   xlabel('Time [s]');

   if(ii == 1)
       ylabel('Input');
   end
   title(title_input_vec(ii));
   if(ii>1)
      ylim([-1,1]) 
   end
end
legend([h1,h2],"Feedback Linearization Controlller","LQR Controller");
% for ii = 1:skip:size(xout,1)
%     plot3(xout(ii,2),xout(ii,4),-xout(ii,6),'b.');
% %     pause(0.001);
% end


% legend([h1;h2],"Trajectory","Desired State");
% %% % plot error
% title_vec = ["$\dot{x}$","$x$","$\dot{y}$","$y$","$\dot{z}$","$z$","$\dot{\phi}$","$\phi$","$\dot{\theta}$","$\theta$","$\dot{\psi}$","$\psi$"];
% % title_vec = ["$x$","$y$","$z$","$\dot{x}$","$\dot{y}$","$\dot{z}$","$\theta$","$\phi$","$\psi$","$\dot{\theta}$","$\dot{\phi}$","$\dot{\psi}$"];
% figure(2)
% error = des_states - xout;
% % error = xout - des_state';
% % des_states = repmat(des_state',size(xout,1),1);
% for ii = 1:nstates
%    subplot(2,6,ii);
%    
%    h1 = plot(tout,error(:,ii));
%    if(ii>6)
%        xlabel('Time [s]');
%    end
%    if(ii==7||ii == 1)
%        ylabel('State');
%    end
% %    hold on
% %    h2 = plot(tout,des_states(:,ii),'r--')
%    title(title_vec(ii));
%    ylim([-1,1]);
% end
% % legend([h1;h2],"Trajectory","Desired State");
% % figure(3)
% % title_input_vec = ["$F_z$","$\tau_\phi$","$\tau_\theta$","$\tau_\psi$"];
% % for ii = 1:4
% %    subplot(1,4,ii);
% %    
% %    h1 = plot(tout,uout(:,ii))
% %    xlabel('Time [s]');
% % 
% %    if(ii == 1)
% %        ylabel('Input');
% %    end
% %    title(title_input_vec(ii));
% %    if(ii>1)
% %       ylim([-1,1]) 
% %    end
% % end