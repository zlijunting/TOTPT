function veh = veh_params()

veh = struct();

veh.mu = 1.0;
veh.g = 9.81;

veh.m = 1250;
veh.Iz = 1050;
veh.lf = 1.4;
veh.lr = 1.4;
veh.l = veh.lf+veh.lr;
veh.hc = 0.35;
veh.Iw = 1.2;
veh.rw = 0.3;

rho = 1.2;
A = 1.6;
Cd = 0.3;
Cl = -0.6;
veh.drag_coeff = rho*Cd*A;
veh.lift_coeff = rho*Cl*A;

veh.wt = 1.5;
veh.ws = 0.2;

%% Brake and torque distribution
% Tf = kt*Tt + kb*Tb
% Tr = (1-kt)*Tt + (%1*kb)*Tb
% kt = 0

veh.kb = 0.65;

%% Tire parameters

% veh.Bx = 18.5782;
% veh.Cx = 1.3383;
% veh.d1x = 0.9472;
% veh.d2x = 322.9126;
veh.Bx = 18;
veh.Cx = 1.3;
veh.d1x = 0.95;
veh.d2x = 320;


% veh.By = 13.0979;
% veh.Cy = 1.4467;
% veh.d1y = 0.9393;
% veh.d2y = 320.5750;
veh.By = 13;
veh.Cy = 1.5;
veh.d1y = 0.95;
veh.d2y = 320;




% veh.mux = 0.95;
% veh.muy = 0.95;
veh.mux = 1.0;
veh.muy = 1.0;


%% Scaling factors
veh.V_s = 100;
veh.beta_s = 1;
veh.gamma_s = 1;
veh.ax_bar_s = veh.g;
veh.ay_bar_s = veh.g;
veh.s_s = 30;
veh.n_s = 0.1;
veh.xi_s = 1;
veh.Tt_s = 1000*2;
veh.Tb_s = 1000*4;
veh.delta_s = pi/8;
veh.omega_s = veh.V_s/veh.rw;

% x_s_tro = [veh.V_s; veh.beta_s; veh.gamma_s; veh.ax_bar_s; veh.ay_bar_s;
%     veh.s_s; veh.n_s; veh.xi_s; veh.Tt_s; veh.Tb_s; veh.delta_s];

% veh.x_s_nmpc = [veh.V_s; veh.beta_s; veh.gamma_s; veh.ax_bar_s; veh.ay_bar_s;
%     veh.s_s; veh.n_s; veh.xi_s; veh.Tt_s; veh.Tb_s; veh.delta_s];

%% State an input limits
% states
veh.V_min = 0;
veh.V_max = 250/3.6;
veh.beta_min = -pi/4;
veh.beta_max =  pi/4;
veh.gamma_min = -pi/2;
veh.gamma_max =  pi/2;
veh.ax_bar_min = -3*veh.g;
veh.ax_bar_max =  3*veh.g;
veh.ay_bar_min = -3*veh.g;
veh.ay_bar_max =  3*veh.g;
veh.s_min =   0;
veh.s_max = 500;
veh.n_min = -4;
veh.n_max =  4;
veh.xi_min = -pi/4;
veh.xi_max =  pi/4;
veh.n_min = -4;
veh.n_max =  4;

% inputs
veh.Tt_min =  0;
veh.Tt_max =  1200*2; % 2 wheels
veh.Tb_min = -1200*4; % 4 wheels
veh.Tb_max = 0;
veh.delta_min = -pi/4;
veh.delta_max =  pi/4;

veh.omega_min = 0;
veh.omega_max = veh.V_max/veh.rw;

% rate of inputs
veh.Tt_dot_min = -1500*2;
veh.Tt_dot_max =  1500*2;
veh.Tb_dot_min = -1500*4;
veh.Tb_dot_max =  1500*4;
% veh.Tt_dot_min = -1000*2;
% veh.Tt_dot_max =  1000*2;
% veh.Tb_dot_min = -1000*4;
% veh.Tb_dot_max =  1000*4;
veh.delta_dot_min = -22.5*pi/180;
veh.delta_dot_max =  22.5*pi/180;




end
