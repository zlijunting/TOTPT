function model = ocp_model_racecar()

import casadi.*

%% horizon parameters
model.N = 30;
model.T = 1.5; % time horizon length
model.Ts = model.T/model.N;

%% Parameters

veh = veh_params();

%% States
% x = [v beta gamma ax_bar ay_bar s n xi Tt Tb delta]'
nx = 11; 

x = SX.sym('x',nx);

V_s = 100;
beta_s = 1;
gamma_s = 1;
ax_bar_s = veh.g;
ay_bar_s = veh.g;
s_s = 30;
n_s = 0.1;
xi_s = 1;
Tt_s = 4e3; % 2 wheels
Tb_s = 12e3; % 4 wheels
delta_s = pi/8;
x_s = [V_s; beta_s; gamma_s; ax_bar_s; ay_bar_s; s_s; n_s; xi_s; Tt_s; Tb_s; delta_s];

V_n = x(1);
V = V_s*V_n;

beta_n = x(2);
beta = beta_s*beta_n;

gamma_n = x(3);
gamma = gamma_s*gamma_n;

ax_bar_n = x(4);
ax_bar = ax_bar_s*ax_bar_n;

ay_bar_n = x(5);
ay_bar = ay_bar_s*ay_bar_n;

s_n = x(6);
s = s_s*s_n;

n_n = x(7);
n = n_s*n_n;

xi_n = x(8);
xi = xi_s*xi_n;

Tt_n = x(9);
Tt = Tt_s*Tt_n;

Tb_n = x(10);
Tb = Tb_s*Tb_n;

delta_n = x(11);
delta = delta_s*delta_n;



%% Differential State Variables 
% only for irk

xdot = SX.sym('xdot',nx); 

%% Control inputs
nu = 3; % [Tt_dot, Tb_dot, dot]

u = SX.sym('u',nu);

dot_Tt_s = Tt_s;
dot_Tb_s = Tb_s;
dot_delta_s = delta_s;
u_s = [dot_Tt_s; dot_Tb_s; dot_delta_s];

dot_Tt_n = u(1);
dot_Tt = dot_Tt_s*dot_Tt_n;

dot_Tb_n = u(2);
dot_Tb = dot_Tb_s*dot_Tb_n;

dot_delta_n = u(3);
dot_delta = dot_delta_s*dot_delta_n;


%% Disturbance(time varing parameters)
np = 2; % [kappa, Vx_ref]

p = SX.sym('p',np);
kappa = p(1);
V_ref = p(2);


%% triangle functions
cosbd = cos(beta-delta);
cosb = cos(beta);
sinbd = sin(beta-delta);
sinb = sin(beta);

cosd = cos(delta);
sind = sin(delta);

%% aerodynamics forces

Vx = V*cosb;
Vy = V*sinb;

F_drag = 0.5*veh.drag_coeff*Vx^2;
F_lift = 0.5*veh.lift_coeff*Vx^2;


%% load transfer

t1 = veh.g*veh.lr/veh.l - ax_bar*veh.hc/veh.l;
t2 = veh.g*veh.lf/veh.l + ax_bar*veh.hc/veh.l;
t3 = veh.hc*ay_bar/(veh.wt*veh.g);

Fzfl = 0.5*veh.m*t1 - veh.m*t1*t3 - 1/4*F_lift;
Fzfr = 0.5*veh.m*t1 + veh.m*t1*t3 - 1/4*F_lift;
Fzrl = 0.5*veh.m*t2 - veh.m*t2*t3 - 1/4*F_lift;
Fzrr = 0.5*veh.m*t2 + veh.m*t2*t3 - 1/4*F_lift;

Fzf = Fzfl + Fzfr;
Fzr = Fzrl + Fzrr;

alphafl = atan((Vy+veh.lf*gamma)/(Vx-0.5*gamma*veh.wt)) - delta;
alphafr = atan((Vy+veh.lf*gamma)/(Vx+0.5*gamma*veh.wt)) - delta;
alpharl = atan((Vy-veh.lr*gamma)/(Vx-0.5*gamma*veh.wt));
alpharr = atan((Vy-veh.lr*gamma)/(Vx+0.5*gamma*veh.wt));

Fyfl = MF_Fy(veh,alphafl,Fzfl);
Fyfr = MF_Fy(veh,alphafr,Fzfr);
Fyrl = MF_Fy(veh,alpharl,Fzrl);
Fyrr = MF_Fy(veh,alpharr,Fzrr);

Fyf = Fyfl + Fyfr;
Fyr = Fyrl + Fyrr;


%% lower controller

Tf = veh.kb*Tb;
Tr = Tt + (1-veh.kb)*Tb;
Fxf = Tf/veh.rw;
Fxr = Tr/veh.rw;

dFzf = 0.5*(Fzfr-Fzfl);
dFzr = 0.5*(Fzrr-Fzrl);

Fxfl = 0.5*Fxf*(1-dFzf/Fzf);
Fxfr = 0.5*Fxf*(1+dFzf/Fzf);

Fxrl = 0.5*Fxr*(1-dFzr/Fzr);
Fxrr = 0.5*Fxr*(1+dFzr/Fzr);

%% System dynamics
ax = (-Fyf*sind + Fxf*cosd + Fxr - F_drag)/veh.m;
ay = ( Fyf*cosd + Fxf*sind + Fyr)/veh.m;

Ft = Fxf*cosbd + Fxr*cosb + Fyf*sinbd + Fyr*sinb - F_drag*cosb;
Fn = Fyf*cosbd + Fyr*cosb - Fxf*sinbd - Fxr*sinb + F_drag*sinb;
Mz = veh.lf*(Fyf*cosd+Fxf*sind) - veh.lr*Fyr + 0.5*veh.wt*((Fxfr-Fxfl)*cosd+(Fyfl-Fyfr)*sind+(Fxrr-Fxrl));

% vehicle dynamics
V_dot = Ft/veh.m;
beta_dot = Fn/(veh.m*V)-gamma;
gamma_dot = Mz/veh.Iz;

tau = model.Ts/2;
dot_ax_bar = (1/tau)*(ax-ax_bar);
dot_ay_bar = (1/tau)*(ay-ay_bar);

% path tracking dynamics
chi = xi+beta;
s_dot = V*cos(chi)/(1-kappa*n);
n_dot = V*sin(chi);
xi_dot = gamma-kappa*s_dot; 


% dynamics
f_expl = [V_dot; beta_dot; gamma_dot; dot_ax_bar; dot_ay_bar; 
    s_dot; n_dot; xi_dot; dot_Tt; dot_Tb; dot_delta]./x_s;


%% populate structure

model.name = 'nmpc';

model.nx = nx;
model.nu = nu;
model.np = np;

model.x = x;
model.xdot = xdot;
model.u = u;
model.p = p;

model.expr_f_expl = f_expl;
model.expr_f_impl = xdot-f_expl;

% Scaling vector
model.x_s = x_s;
model.u_s = u_s;

% State bound
model.x_min = [veh.V_min; veh.beta_min; veh.gamma_min; veh.ax_bar_min; veh.ay_bar_min; 
    veh.s_min; veh.n_min; veh.xi_min; veh.Tt_min; veh.Tb_min; veh.delta_min]./x_s;
model.x_max = [veh.V_max; veh.beta_max; veh.gamma_max; veh.ax_bar_max; veh.ay_bar_max; 
    veh.s_max; veh.n_max; veh.xi_max; veh.Tt_max; veh.Tb_max; veh.delta_max]./x_s;

model.x_min = min(model.x_min, -1e-3*ones(nx,1));
model.x_max = max(model.x_max, 1e-3*ones(nx,1));

% Input bound
model.u_min = [veh.Tt_dot_min; veh.Tb_dot_min; veh.delta_dot_min]./u_s;
model.u_max = [veh.Tt_dot_max; veh.Tb_dot_max; veh.delta_dot_max]./u_s;




%% Constraints
% Actuator orthogonality
BrTh = 100*Tt_n*Tb_n; % BrTh

% Tire ellipse
mux = veh.mux;
muy = veh.muy;
mufl_ineq = Fxfl^2/(mux*Fzfl)^2 + Fyfl^2/(muy*Fzfl)^2;
mufr_ineq = Fxfr^2/(mux*Fzfr)^2 + Fyfr^2/(muy*Fzfr)^2;
murl_ineq = Fxrl^2/(mux*Fzrl)^2 + Fyrl^2/(muy*Fzrl)^2;
murr_ineq = Fxrr^2/(mux*Fzrr)^2 + Fyrr^2/(muy*Fzrr)^2;

model.constraint.expr = vertcat(BrTh, mufl_ineq, mufr_ineq, murl_ineq, murr_ineq);



%% Cost Function
eV_max = 1;
ebeta_max = 0.05;
n_max = 0.1;
chi_max = 0.05;
beta_ref = atan(delta*veh.lr/veh.l);
symx = vertcat((beta-beta_ref)/ebeta_max, (V-V_ref)/eV_max, n/n_max, chi/chi_max);
symu = vertcat(dot_Tt/dot_Tt_s, dot_Tb/dot_Tb_s, dot_delta/dot_delta_s);

Q = diag([1, 1, 1, 1]);
R = diag([1, 1, 10]);

model.expr_ext_cost_e = 0.5*symx'*Q*symx;
model.expr_ext_cost = 0.5*symx'*Q*symx + 0.5*symu'*R*symu;



%% Tire Model

% function Fy = MF_Fy(alpha,Fz)
% By = 13.0979;
% Cy = 1.4467;
% d1y = 0.9393;
% d2y = 320.5750;
% Fy = -(d1y*Fz+d2y)*sin(Cy*atan(By*alpha));
% end

function Fy = MF_Fy(veh,alpha,Fz)
Fy = -(veh.d1y*Fz+veh.d2y)*sin(veh.Cy*atan(veh.By*alpha));
end




end