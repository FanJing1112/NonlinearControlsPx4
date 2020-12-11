close all; clear all;
% Load in feedbacklinearization error
params.radius = 1;
% states_feedbacklin = load('output_logs_feedbacklin/output_files_newspiral_feedbackLin4/output_files/state.txt');
% control_inputs_feedbacklin = load('output_logs_feedbacklin/output_files_newspiral_feedbackLin4/output_files/control_input.txt');

% states_feedbacklin = load('output_logs_feedbacklin/state.txt');
% control_inputs_feedbacklin = load('output_logs_feedbacklin/control_input.txt');

states_feedbacklin = load('output_logs_feedbacklin/output_files_0_0/output_files/state.txt');
control_inputs_feedbacklin = load('output_logs_feedbacklin/output_files_0_0/output_files/control_input.txt');

% states_feedbacklin = load('output_logs_feedbacklin/output_files_1_20/output_files/state.txt');
% control_inputs_feedbacklin = load('output_logs_feedbacklin/output_files_1_20/output_files/control_input.txt');
% states_feedbacklin = load('output_logs_feedbacklin/output_files_1_130/output_files/state.txt');
% control_inputs_feedbacklin = load('output_logs_feedbacklin/output_files_1_130/output_files/control_input.txt');
% states_feedbacklin = load('output_logs_feedbacklin/output_files_0_90/output_files/state.txt');
% control_inputs_feedbacklin = load('output_logs_feedbacklin/output_files_0_90/output_files/control_input.txt');

% states_feedbacklin = load('output_logs_feedbacklin/state_1_900.txt');
% control_inputs_feedbacklin = load('output_logs_feedbacklin/control_input_1_900.txt');
% states_feedbacklin = load('output_logs_lqr/state.txt');
% control_inputs_feedbacklin = load('output_logs_lqr/control_input.txt');
% Load in lqr data
states_lqr = load('output_logs_lqr/state.txt');
control_inputs_lqr = load('output_logs_lqr/control_input.txt');
P = load('new_pe.txt');
% states = readmatrix('output_logs/state.txt');
% ekf = readmatrix('output_logs/ekf.txt');
% state_estimation = readmatrix('output_logs/state_estimation.txt');
% control_inputs = readmatrix('output_logs/control_input.txt');
% control_inputs_feedbacklin(:,2) = control_inputs_feedbacklin(:,2)*32;
% control_inputs_feedbacklin(:,3) = control_inputs_feedbacklin(:,3)*4;
% control_inputs_feedbacklin(:,4) = control_inputs_feedbacklin(:,4)*4;
% control_inputs_feedbacklin(:,5) = control_inputs_feedbacklin(:,5)*0.05;

%%%%%%%%%%%%
%% % plot error
% des_state = [0;0;0;0;0;-1;0;0;0;0;0;0];
nstates = 12;
title_vec = ["$\dot{x}$","$x$","$\dot{y}$","$y$","$\dot{z}$","$z$","$\dot{\phi}$","$\phi$","$\dot{\theta}$","$\theta$","$\dot{\psi}$","$\psi$"];
% title_vec = ["$x$","$y$","$z$","$\dot{x}$","$\dot{y}$","$\dot{z}$","$\theta$","$\phi$","$\psi$","$\dot{\theta}$","$\dot{\phi}$","$\dot{\psi}$"];
end_time = 25;
% end_time = 15;
xout_feedbacklin = states_feedbacklin(:,2:13);
uout_feedbacklin = control_inputs_feedbacklin(:,2:end);
tout_feedbacklin = (states_feedbacklin(:,1)- states_feedbacklin(1,1))/1000000;
end_idx_feedbacklin = find(tout_feedbacklin>=end_time,1,'first');
xout_feedbacklin = xout_feedbacklin(1:end_idx_feedbacklin,:);
tout_feedbacklin = tout_feedbacklin(1:end_idx_feedbacklin);
uout_feedbacklin = uout_feedbacklin(1:end_idx_feedbacklin,:);

xout_lqr = states_lqr(:,2:13);
uout_lqr = control_inputs_lqr(:,2:end);
tout_lqr = (states_lqr(:,1)- states_lqr(1,1))/1000000;
end_idx_lqr = find(tout_lqr>=end_time,1,'first');
xout_lqr = xout_lqr(1:end_idx_feedbacklin,:);
tout_lqr = tout_lqr(1:end_idx_feedbacklin);
uout_lqr =uout_lqr(1:end_idx_feedbacklin,:);
% figure(2)
% error = xout - des_state';
des_states_feedbacklin = zeros(size(xout_feedbacklin,1),14);
lyapunov = 0;
lyapunov_vec_feedbacklin = tout_feedbacklin;
for ii = 1:numel(tout_feedbacklin) 
   des_states_feedbacklin(ii,:) = get_reference(tout_feedbacklin(ii),params.radius)'; 
   des_state_px4 = get_reference_px4(tout_feedbacklin(ii))';
   lyapunov_feedbacklin = (des_state_px4-xout_feedbacklin(ii,:))*P*(des_state_px4-xout_feedbacklin(ii,:))';
   lyapunov_vec_feedbacklin(ii) = lyapunov_feedbacklin;
end

lyapunov_vec_lqr = tout_lqr;
for ii = 1:numel(tout_lqr) 
   des_states_lqr(ii,:) = get_reference(tout_lqr(ii),params.radius)'; 
   des_state_px4 = get_reference_px4(tout_lqr(ii))';
   lyapunov_lqr = (des_state_px4-xout_lqr(ii,:))*P*(des_state_px4-xout_lqr(ii,:))';
   lyapunov_vec_lqr(ii) = lyapunov_lqr;
end
%%%%%%%%%%%%%%% LOAD IN STANDARD LQR OUTPUT %%%%%%%%%%%%%%%%%%%%%
% lqr_output = load("lqr_tracking_output_matlab.mat");

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% uout = [];
% for ii = 1:size(xout,1)
%     [dxdt,u] = dynamics_output([],xout(ii,:)',params);
%     uout = [uout;u'];
% end
state_titles = ["PX4: North Position Time Series and Error","PX4: East Position Time Series and Error","PX4: Altitude Time Series and Error ","PX4: Yaw Time Series and Error"];
state_labels = ["$p_x$ [m]","$p_y$ [m]","$p_z$ [m]","$\psi$ [rad]"];
error_labels = ["$e_x$ [m]","$e_y$ [m]","$e_z$ [m]","$e_\psi$ [rad]"];
px4_notation = [2,4,6,12];
for ii = 1:4
% figure(ii)
figure('Renderer', 'painters', 'Position', [10 10 800 900])
subplot(2,1,1)
h1 = plot(tout_feedbacklin,des_states_feedbacklin(:,ii),'k')
% PLOT NORMAL LQR TRAJ
ylabel(state_labels(ii));
hold on
h2 =plot(tout_feedbacklin,xout_feedbacklin(:,px4_notation(ii)),'b--')
h3 =plot(tout_lqr,xout_lqr(:,px4_notation(ii)),'r--')
title(state_titles(ii));
xlim([0,end_time]);
subplot(2,1,2)

h1 = plot(tout_feedbacklin,des_states_feedbacklin(:,ii)-des_states_feedbacklin(:,ii),'k-')
hold on
h2 =plot(tout_feedbacklin,des_states_feedbacklin(:,ii)-xout_feedbacklin(:,px4_notation(ii)),'b--')
h3 = plot(tout_lqr,des_states_lqr(:,ii)-xout_lqr(:,px4_notation(ii)),'r--')
% PLOT NORMAL LQR ERROR
xlabel('Time [s]');
ylabel(error_labels(ii));
xlim([0,end_time]);
legend([h1,h2,h3],"Desired Trajectory","Feedback Linearization Controlller","LQR Controller");

end


figure(5)
h1 = plot3(des_states_feedbacklin(:,1),des_states_feedbacklin(:,2),-des_states_feedbacklin(:,3),'k');
hold on
h2 = plot3(xout_feedbacklin(:,px4_notation(1)),xout_feedbacklin(:,px4_notation(2)),-xout_feedbacklin(:,px4_notation(3)),'b--')
h3 = plot3(xout_lqr(:,px4_notation(1)),xout_lqr(:,px4_notation(2)),-xout_lqr(:,px4_notation(3)),'r--')
legend([h1,h2,h3],"Desired Trajectory","Feedback Linearization Controlller","LQR Controller");
grid on
xlabel("x pos [m]");
ylabel("y pos [m]");
zlabel("z pos [m]");
title("PX4: Translational Tracking");
axis([-5,5,-5,5,0,8]);
skip = 2;
% legend([h1,h2,h3],"Desired Trajectory","Feedback Linearization Controlller","LQR Controller");


figure(6)
h1 = plot(tout_feedbacklin,uout_feedbacklin(:,1),'r--')
title("$F$");
figure(7)
h1 = plot(tout_feedbacklin,uout_feedbacklin(:,2),'r--')
title("$\tau_\phi$");
figure(8)
h1 = plot(tout_feedbacklin,uout_feedbacklin(:,3),'r--')
title("$\tau_\theta$");
figure(9)
h1 = plot(tout_feedbacklin,uout_feedbacklin(:,4),'r--')
title("$\tau_\psi$");

% Should we come up with a better lyapunov function?
figure(10)
h1 = plot(tout_feedbacklin,lyapunov_vec_feedbacklin,'b--')
hold on
h2 = plot(tout_lqr,lyapunov_vec_lqr,'r--')
title('PX4: Lyapunov Time Series');
xlabel('Time [s]');
ylabel('Lyapunov Levelset');
xlim([0,end_time]);
legend([h1,h2],"Feedback Linearization Controlller","LQR Controller");

figure(11)
title_input_vec = ["$F_z$","$\tau_\phi$","$\tau_\theta$","$\tau_\psi$"];
for ii = 1:4
   subplot(1,4,ii);
   
   h1 = plot(tout_feedbacklin,uout_feedbacklin(:,ii),'b')
   hold on
   
   h2 = plot(tout_lqr,uout_lqr(:,ii),'r--')
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
% 
% figure(7)
% plot(tout,lyapunov_function(:,2))
% title('Lyapunov Function Time series')
% xlabel('Time');
% ylabel('Lyapunov Function Output')