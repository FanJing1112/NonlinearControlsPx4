close all; clear all;
% states
syms px py pz u v w phi theta psi p q r zeta eta t real
% Inputs
% syms F_l F_r F_b F_f tau_phi tau_theta tau_psi
% syms delta_l delta_r delta_b delta_f k_1 k_2 real
syms F F_dot tau_phi tau_theta tau_psi u1_bar v1 v2 v3 v4 real
% Parameters
syms J_x J_y J_z m g ell k_1 k_2 real

% F_l = k_1*delta_l; F_r = k_1*delta_r; F_b = k_1*delta_b; F_f = k_1*delta_f;
% tau_l = k_2*delta_l; tau_r = k_2*delta_r; tau_b = k_2*delta_b; tau_f = k_2*delta_f;
% 
% F = F_f+F_r +F_b+F_l;
% tau_phi = ell*(F_l-F_r);
% tau_theta = ell*(F_f-F_b);
% tau_psi = -tau_f+tau_r -tau_b+tau_l;
trans_vel = [cos(theta)*cos(psi) ,  sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi) , cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
    cos(theta)*sin(psi) ,  sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi) , cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
    sin(theta),-sin(phi)*cos(theta) , -cos(phi)*cos(theta)]*[u; v; w];
trans_vel(3) = -trans_vel(3);
trans_accel = [r*v - q*w ; p*w - r*u ; q*u - p*v] +...
    [-g*sin(theta) ; g*cos(theta)*sin(phi) ; g*cos(theta)*cos(phi)] +...
    (1/m)*[0 ; 0 ; -F];
angular_vel = [1 , sin(phi)*tan(theta) , cos(phi)*tan(theta) ;
    0 , cos(phi) , -sin(phi) ;
    0 , sin(phi)/cos(theta) , cos(phi)/cos(theta)]* [p;q;r];
angular_accel= [(J_y-J_z)/J_x*q*r;
    (J_z-J_x)/J_y*p*r;
    (J_x-J_y)/J_z*p*q] + ...
    [1/J_x*tau_phi;
    1/J_y*tau_theta;
    1/J_z*tau_psi];
states = [u px v py w pz p phi q theta r psi]';
% states = [px u py v pz w phi p theta q psi r]';
% states = [px py pz u v w phi theta psi p q r]';
inputs = [F tau_phi tau_theta tau_psi]';
% inputs = [delta_f delta_r delta_b delta_l]';
% dynamics = [trans_vel;trans_accel;angular_vel;angular_accel];
dynamics = [trans_accel(1);trans_vel(1);...
            trans_accel(2);trans_vel(2);...
            trans_accel(3);trans_vel(3);...
            angular_accel(1);angular_vel(1);...
            angular_accel(2);angular_vel(2);...
            angular_accel(3);angular_vel(3)];

% Calc Jacobians
Dxf = jacobian(dynamics,states);
Duf = jacobian(dynamics,inputs);
flow = dynamics - Duf*inputs;

% C_mat = jacobian(output,states);
%% What happens if we take Lie derivatives of not augmented system?
states_2 = [px py pz psi theta phi u v w r q p]';
dynamics_2 = [trans_vel(1);trans_vel(2);trans_vel(3);
            angular_vel(3);angular_vel(2);angular_vel(1);
            trans_accel(1);trans_accel(2);trans_accel(3); % remove the force and make it eta
            angular_accel(3);angular_accel(2);angular_accel(1)];
h = [px;py;pz;psi]; 
Dxf_2 = jacobian(dynamics_2,states_2);
Duf_2 = jacobian(dynamics_2,inputs);
flow_2 = dynamics_2 - Duf_2*inputs;
for ii = 1:numel(h)
    hi = h(ii);
    Lfhi = simplify(jacobian(hi,states_2)*flow_2);
    Lf_2hi = simplify(jacobian(Lfhi,states_2)*flow_2);
    Lf_3hi = jacobian(Lf_2hi,states_2)*flow_2;
    latex(jacobian(Lfhi,states_2)*Duf_2)
    jacobian(Lf_2hi,states_2)*Duf_2
    jacobian(Lf_3hi,states_2)*Duf_2
%     Lfhi = simplify(jacobian(hi,states_2)*flow_2);
%     Lf_2hi = simplify(jacobian(Lfhi,states_2)*flow_2);
%     Lf_3hi = jacobian(Lf_2hi,states_2)*flow_2;
%     jacobian(Lfhi,states_2)*Duf_2
%     jacobian(Lf_2hi,states_2)*Duf_2
%     jacobian(Lf_3hi,states_2)*Duf_2
    % generating z = T(x) eq (4)
%     if(ii ==4)
%         % psi doesn't need the other lie derivatives
%         T_i = [hi;Lfhi];
%         Lg_Lf_1hi = jacobian(Lfhi,states_augmented)*Duf_augmented;
%         Delta =[Delta;Lg_Lf_1hi];
%         b_vec = [b_vec;Lf_2hi];
%     else
%         Lf_3hi = jacobian(Lf_2hi,states_augmented)*flow_augmented;
%         Lf_4hi = jacobian(Lf_3hi,states_augmented)*flow_augmented;
%         Lg_Lf_3hi = jacobian(Lf_3hi,states_augmented)*Duf_augmented;
%         % generating z = T(x) eq (4)
%         T_i = [hi;Lfhi;Lf_2hi;Lf_3hi];
%         %  Fill in Delta and b eq (6)
%         Delta =[Delta;Lg_Lf_3hi];
%         b_vec = [b_vec;Lf_4hi];
%     end
%     T_diffeomorphism = [T_diffeomorphism;T_i];
end

%% Feedback Linearization Paper Quan
%%% Following their notation as best as possible for debugging
%%% states = [X Y Z psi theta phi X_dot Y_dot Z_dot psi_dot theta_dot...
%%% phi_dot zeta eta p q r] Note that Baird r and p are swapped
%%% NOte that this paper is using simplified dynamics
states_augmented = [px py pz psi theta phi u v w zeta eta r q p]';
dynamics_augmented = [trans_vel(1);trans_vel(2);trans_vel(3);
            angular_vel(3);angular_vel(2);angular_vel(1);
            trans_accel(1);trans_accel(2);trans_accel(3)+F/m-zeta/m; % remove the force and make it eta
            eta;u1_bar;
            angular_accel(3);angular_accel(2);angular_accel(1)];
%%% Testing out simplified world coordinates
% dynamics_augmented = [states_augmented(7);states_augmented(8);states_augmented(9);
%             angular_vel(3);angular_vel(2);angular_vel(1);
%             -1/m*(cos(phi)*cos(psi)*sin(theta) + sin(phi)*sin(psi))*zeta;
%             -1/m*(cos(phi)*sin(psi)*sin(theta) - sin(phi)*cos(psi))*zeta;
%             g-1/m*(cos(phi)*cos(theta))*zeta;
% %             trans_accel(1);trans_accel(2);trans_accel(3)+F/m-eta/m; % remove the force and make it eta
%             eta;u1_bar;
%             angular_accel(3);angular_accel(2);angular_accel(1)];
inputs_augmented = [u1_bar;tau_psi;tau_theta;tau_phi];
% Output equation 
h = [px;py;pz;psi]; 

% Calculate f(x) and g(x) where xdot = f(x) + g(x)u_augmented
Dxf_augmented = jacobian(dynamics_augmented,states_augmented);
Duf_augmented = jacobian(dynamics_augmented,inputs_augmented);
flow_augmented = dynamics_augmented - Duf_augmented*inputs_augmented;

T_diffeomorphism = [];
Delta = [];
b_vec = [];
for ii = 1:numel(h)
    hi = h(ii);
    Lfhi = simplify(jacobian(hi,states_augmented)*flow_augmented);
    Lf_2hi = simplify(jacobian(Lfhi,states_augmented)*flow_augmented);
    Lg_Lfhi = simplify(jacobian(Lfhi,states_augmented)*Duf_augmented)
    Lg_Lf_2hi = simplify(jacobian(Lf_2hi,states_augmented)*Duf_augmented)
    % generating z = T(x) eq (4)
    if(ii ==4)
        % psi doesn't need the other lie derivatives
        T_i = [hi;Lfhi];
        Lg_Lf_1hi = jacobian(Lfhi,states_augmented)*Duf_augmented;
        latex(Lg_Lf_1hi)
        Delta =[Delta;Lg_Lf_1hi];
        b_vec = [b_vec;Lf_2hi];
    else
        Lf_3hi = simplify(jacobian(Lf_2hi,states_augmented)*flow_augmented);
        Lf_4hi = jacobian(Lf_3hi,states_augmented)*flow_augmented;
        Lg_Lf_3hi = simplify(jacobian(Lf_3hi,states_augmented)*Duf_augmented);
        latex(Lg_Lf_3hi)
        % generating z = T(x) eq (4)
        T_i = [hi;Lfhi;Lf_2hi;Lf_3hi];
        %  Fill in Delta and b eq (6)
        Delta =[Delta;Lg_Lf_3hi];
        b_vec = [b_vec;Lf_4hi];
    end
    T_diffeomorphism = [T_diffeomorphism;T_i];
end
% simplify everything at the end
b_vec = simplify(b_vec);
Delta = simplify(Delta);
T_diffeomorphism = simplify(T_diffeomorphism);

% Define the linearized system
%%% General linearized A_c form
A_1 = [0,1,0,0;0,0,1,0;0,0,0,1;0,0,0,0]; A_2 = [0,1;0,0];
A_c = blkdiag(A_1,A_1,A_1,A_2);
%%% General linearized B_c form
B_1 = zeros(4,4); B_2 = B_1; B_3 = B_1; B_4 = zeros(2,4);
B_1(4,1) = 1; B_2(4,2) = 1; B_3(4,3) = 1; B_4(2,4) = 1;
B_c = [B_1;B_2;B_3;B_4];
%%% General linearized C_c form
C_1 = [1,0,0,0]; C_2 = [1,0];
C_c = blkdiag(C_1,C_1,C_1,C_2);

% Control law eq (7)
Delta_inv = (inv(Delta));
alpha = -Delta_inv*b_vec;
beta = Delta_inv;
linearized_inputs = [v1;v2;v3;v4]; % bottom of page 14
u_bar = alpha + beta*linearized_inputs;

Q = eye(14);
% R = 0.01*eye(4); % testing other R values
R = 0.001*eye(4);
K_c = lqr(A_c,B_c,Q,R);
parameters = [J_x J_y J_z m g ell k_1 k_2];
% Make matlab functions
% matlabFunction(Dxf,'File','calc_A_out','Vars',[{states},{inputs},{parameters}])
% matlabFunction(Duf,'File','calc_B_out','Vars',[{states},{inputs},{parameters}])

% % Commented out to not screw with working functions
% matlabFunction(Delta,'File','calc_Delta','Vars',[{states_augmented},{parameters}])
% matlabFunction(T_diffeomorphism,'File','calc_z','Vars',[{states_augmented},{parameters}])
% matlabFunction(dynamics,'File','calc_dyn_out','Vars',[{states},{inputs},{parameters}])
% matlabFunction(dynamics_augmented,'File','calc_dyn_augmented','Vars',[{states_augmented},{inputs_augmented},{parameters}])
% matlabFunction(u_bar,'File','calc_feedback_linearization','Vars',[{states_augmented},{linearized_inputs},{parameters}])
% matlabFunction(sym(A_c),sym(B_c),sym(C_c),sym(K_c),'File','calc_normal_form_mats');
latex(sym(A_c))
latex(sym(B_c))
latex(sym(C_c))