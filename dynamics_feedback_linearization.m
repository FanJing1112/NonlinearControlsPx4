function [dxdt,u] = dynamics_feedback_linearization(t,x,params)
%[u, p_n, v, p_e, w, h, p, \phi, q, \theta, r, \psi]
% des_state = get_reference(t);
[des_state_z,des_accel_z] = get_reference_z(t,params.radius);
% u = params.gains*(des_state-x) + params.des_input;
%     u = params.gains*(params.des_state-x) + params.des_input;
%     take any input larger than the max "1" and set it to 1
%     u(u>1) = ones(sum(u>1),1);
%     dxdt = calc_dyn_out(x,zeros(size(u)),params.parameters);
%     dxdt = calc_dyn_out(x,u,params.parameters);

[A_c,B_c,C_c,K_c] = calc_normal_form_mats();
% params.parameters is [J_x J_y J_z m g]
z = calc_z(x,params.parameters);
v = des_accel_z + K_c*(des_state_z - z); % bottom of page 14
% z_r = calc_z(des_state,params.parameters);
% Delta = calc_Delta(x,params.parameters);
u_bar = calc_feedback_linearization(x,v,params.parameters);
u = [x(10);u_bar(2:4)];
% If this was px4 you need to integrate eta zeta with u_bar here to
% calculate what u is equal to. u = zeta
dxdt = calc_dyn_augmented(x,u_bar,params.parameters);
end