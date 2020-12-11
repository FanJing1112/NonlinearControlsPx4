close all; clear all;
% Load in feedbacklinearization error
params.radius = 1;
% states_feedbacklin = load('output_logs_feedbacklin/state.txt');
% control_inputs_feedbacklin = load('output_logs_feedbacklin/control_input.txt');

% states_feedbacklin = load('output_logs_feedbacklin/output_files_0_0/output_files/state.txt');
% control_inputs_feedbacklin = load('output_logs_feedbacklin/output_files_0_0/output_files/control_input.txt');

%%%%%%%%%%%%
%% % plot error
% des_state = [0;0;0;0;0;-1;0;0;0;0;0;0];
parameter_variations_strings = ["0_90","0_80","0_70","0_60","0_50","0_40","0_30","0_20","0_10","0_0",...
                        "1_10","1_20","1_30","1_40","1_50","1_60","1_70","1_80","1_90","1_100",...
                        "1_110","1_120","1_130"];
parameter_variations = [0.1:0.1:2.3];                   
mse_variations = zeros(numel(parameter_variations_strings),1);
for kk = 1:numel(parameter_variations_strings)
    % Load in data
    states_feedbacklin = load("output_logs_feedbacklin/output_files_"+parameter_variations_strings(kk)+"/output_files/state.txt");
    % Trim to 25 seconds
end_time = 25;
xout_feedbacklin = states_feedbacklin(:,2:13);
tout_feedbacklin = (states_feedbacklin(:,1)- states_feedbacklin(1,1))/1000000;
end_idx_feedbacklin = find(tout_feedbacklin>=end_time,1,'first');
xout_feedbacklin = xout_feedbacklin(1:end_idx_feedbacklin,:);
tout_feedbacklin = tout_feedbacklin(1:end_idx_feedbacklin);


des_states_feedbacklin = zeros(size(xout_feedbacklin,1),14);
mse = 0;
se_vec = tout_feedbacklin;
P = zeros(12,12);
P(2,2) = 1;P(4,4) = 1;P(6,6) = 1;P(12,12) = 1;
for ii = 1:numel(tout_feedbacklin) 
   des_states_feedbacklin(ii,:) = get_reference(tout_feedbacklin(ii),params.radius)'; 
   des_state_px4 = get_reference_px4(tout_feedbacklin(ii))';
   se = (des_state_px4-xout_feedbacklin(ii,:))*P*(des_state_px4-xout_feedbacklin(ii,:))';
   mse = mse + se;
   se_vec(ii) = se;
end
mse_variations(kk) = mse;
end
%% Calculate for lqr
states_lqr = load('output_logs_lqr/state.txt');
control_inputs_lqr = load('output_logs_lqr/control_input.txt');
xout_lqr = states_lqr(:,2:13);
tout_lqr = (states_lqr(:,1)- states_lqr(1,1))/1000000;
end_idx_lqr = find(tout_lqr>=end_time,1,'first');
xout_lqr = xout_lqr(1:end_idx_feedbacklin,:);
tout_lqr = tout_lqr(1:end_idx_feedbacklin);

lyapunov_vec_lqr = tout_lqr;
se_vec_lqr = se_vec;
mse_lqr = 0;
for ii = 1:numel(tout_lqr) 
   des_state_px4 = get_reference_px4(tout_lqr(ii))';
   se_lqr = (des_state_px4-xout_lqr(ii,:))*P*(des_state_px4-xout_lqr(ii,:))';
      mse_lqr = mse_lqr + se_lqr;
   se_vec_lqr(ii) = se_lqr;
end

figure(1);
h1 = plot(parameter_variations,mse_variations,'b');
hold on
h2 = plot([parameter_variations(1),parameter_variations(end)],[mse_lqr,mse_lqr],'r--');
legend([h1,h2],'Feedback Linearization Controller', 'Constant LQR');
xlim([parameter_variations(1),parameter_variations(end)])
ylabel('MSE');
xlabel('Ratio of actual inertial terms');
title('Mean Squared Error when Varying Percieved Inertia $[J_{xx},J_{yy}]$');



