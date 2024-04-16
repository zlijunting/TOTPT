 import casadi.*

veh = veh_params();

%% Vehicle Model - state variables

nx = 9; % number of state variables

% longitudinal velocity [m/s]
V_n = SX.sym('v_n');
V_s = 100;
V = V_s * V_n;

% sideslip angle [rad]
beta_n = SX.sym('beta_n');
beta_s = 1;
beta = beta_s * beta_n;

% yaw rate [rad/s]
gamma_n = SX.sym('gamma_n');
gamma_s = 1;
gamma = gamma_s * gamma_n;

% lateral distance to centreline [m] - left of centreline => n > 0; right => n < 0
n_n = SX.sym('n_n');
n_s = 5;
n = n_s * n_n;

% course angle to centreline tangent direction [rad]
xi_n = SX.sym('xi_n');
xi_s = 1;
xi = xi_s * xi_n;


omega_s = V_s/veh.rw;

% angular velocity front left tyre [rad/s]
omega_fl_n = SX.sym('omega_fl_n');
omega_fl = omega_s * omega_fl_n;

% angular velocity front right tyre [rad/s]
omega_fr_n = SX.sym('omega_fr_n');
omega_fr = omega_s * omega_fr_n;

% angular velocity rear left tyre [rad/s]
omega_rl_n = SX.sym('omega_rl_n');
omega_rl = omega_s * omega_rl_n;

% angular velocity rear right tyre [rad/s]
omega_rr_n = SX.sym('omega_rr_n');
omega_rr = omega_s * omega_rr_n;


% states vector (scaled)
x = [V_n beta_n gamma_n n_n xi_n omega_fl_n omega_fr_n omega_rl_n omega_rr_n]';  

% scaling factors
x_s = [V_s beta_s gamma_s n_s xi_s omega_s omega_s omega_s omega_s]';


% state limits
omega_max = veh.V_max/veh.rw; 
x_min = [    -1e-3; -pi/4; -pi/2; -4; -pi/4;     0;     0;     0;     0]./x_s;
x_max = [veh.V_max;  pi/4;  pi/2;  4;  pi/4; omega_max; omega_max; omega_max; omega_max]./x_s;



%% Vehicle model - control variables (inputs)

nu = 3; % number of control variables

% driving torque (Nm)
Tt_n = SX.sym('Tt_n');
% Tt_s = veh.Tt_abs;
Tt_s = 2e3;
% Tt_s = 4e3;
% Tt_s = veh.m*veh.g*veh.rw;
Tt = Tt_s * Tt_n;

% braking torque [Nm]
Tb_n = SX.sym('Tb_n');
% Tb_s = veh.Tb_abs;
Tb_s = 4e3;
% Tb_s = 12e3;
% Tb_s = veh.m*veh.g*veh.rw;
Tb = Tb_s * Tb_n;

% steer angle [rad]
delta_n = SX.sym('delta_n');
delta_s = pi/8;
% delta_s = 0.1;
delta = delta_s * delta_n;

% inputs vector (scaled)
u = [Tt_n Tb_n delta_n]'; 

% scaling factors for inputs
u_s = [Tt_s Tb_s delta_s]'; 

% input limits
u_min = [veh.Tt_min veh.Tb_min veh.delta_min]'./u_s;
u_max = [veh.Tt_max veh.Tb_max veh.delta_max]'./u_s;

%Constraints on the rate of inputs (units of each input per second)
duk_lb = [veh.Tt_dot_min veh.Tb_dot_min veh.delta_dot_min]'./u_s; %lower bound
duk_ub = [veh.Tt_dot_max veh.Tb_dot_max veh.delta_dot_max]'./u_s; %upper bound
% duk_ub = [ 1e4;  1e4;  1]./u_s; %upper bound
% duk_lb = [-1e4; -1e4; -1]./u_s; %lower bound

% Regularisation factors
rdu = [1; 1; 10]; % 
rdy = [1; 1]; % first derivatives of the aux variables
% rdy = [0.1; 0.1]; % first derivatives of the aux variables
% rdu2 = [0.01; 0.01; 1]; % second derivatives of the inputs
% rdy2 = [0.015; 0.015]; % second derivatives of the aux variables

%% Vehicle model - additional variables

nz = 2; % Number of aux variables

% longitudinal acceleration
ax_bar_n = SX.sym('bar_ax_n');
ax_bar_s = veh.g;
ax_bar = ax_bar_s * ax_bar_n;

% lateral acceleration
ay_bar_n = SX.sym('bar_ay_n');
ay_bar_s = veh.g;
ay_bar = ay_bar_s * ay_bar_n;

% aux. variable vector (scaled)
z = [ax_bar_n ay_bar_n]';
% scaling factors for aux. variable
z_s = [ax_bar_s ay_bar_s]';

% aux. variables limits
% y_min = [-inf -inf]';
% y_max = [ inf  inf]';
z_min = [-2*veh.g -2*veh.g]';
z_max = [ 2*veh.g  2*veh.g]';


%% Vechicle model - parameter variables
%Symbolic variables that are not decision variables of the NLP - defined as such because their value changes (it is like a variable value parameter)

kappa = SX.sym('kappa'); % kappa > 0 for left turns

pv = kappa; % collect variable parameters

%% Vehicle model - equations
%Calculate variables of the system other than the states and inputs

cosd = cos(delta);
sind = sin(delta);
cosb = cos(beta);
sinb = sin(beta);
cosbd = cos(beta-delta);
sinbd = sin(beta-delta);


% longitudinal and lateral velocities
vx = V*cosb;
vy = V*sinb;

% Resistance forces
%-aerodynamic forces [N]
f_drag = 0.5*veh.drag_coeff*vx^2;
f_lift = 0.5*veh.lift_coeff*vx^2;


% Tire velocities

vxfl = (vx-0.5*veh.wt*gamma)*cosd + (vy+veh.lf*gamma)*sind;
vxfr = (vx+0.5*veh.wt*gamma)*cosd + (vy+veh.lf*gamma)*sind;
vxrl = (vx-0.5*veh.wt*gamma);
vxrr = (vx+0.5*veh.wt*gamma);

vyfl = (vy+veh.lf*gamma)*cosd - (vx-0.5*veh.wt*gamma)*sind;
vyfr = (vy+veh.lf*gamma)*cosd - (vx+0.5*veh.wt*gamma)*sind;
vyrl = (vy-veh.lr*gamma);
vyrr = (vy-veh.lr*gamma);

% % Tire sideslip angles
% alphafl = atan(vyfl/vxfl);
% alphafr = atan(vyfr/vxfr);
% alpharl = atan(vyrl/vxrl);
% alpharr = atan(vyrr/vxrr);
% 
% % Tire longitudinal slips
% lambdafl = (veh.rw*omega_fl-vxfl)/vxfl;
% lambdafr = (veh.rw*omega_fr-vxfr)/vxfr;
% lambdarl = (veh.rw*omega_rl-vxrl)/vxrl;
% lambdarr = (veh.rw*omega_rr-vxrr)/vxrr;


% vertical tyre forces [N]
t1 = veh.g*veh.lr/veh.l - ax_bar*veh.hc/veh.l;
t2 = veh.g*veh.lf/veh.l + ax_bar*veh.hc/veh.l;
t3 = veh.hc*ay_bar/veh.wt/veh.g;

fzfl = 0.5*veh.m*t1 - veh.m*t1*t3 - f_lift/4;
fzfr = 0.5*veh.m*t1 + veh.m*t1*t3 - f_lift/4;
fzrl = 0.5*veh.m*t2 - veh.m*t2*t3 - f_lift/4;
fzrr = 0.5*veh.m*t2 + veh.m*t2*t3 - f_lift/4;

fzf = fzfl + fzfr;
fzr = fzrl + fzrr;


% Tire longitudinal slips
SXfl = (veh.rw*omega_fl-vxfl)/vxfl;
SXfr = (veh.rw*omega_fr-vxfr)/vxfr;
SXrl = (veh.rw*omega_rl-vxrl)/vxrl;
SXrr = (veh.rw*omega_rr-vxrr)/vxrr;

% Tire lateral slips
syfl = vyfl/vxfl;
syfr = vyfr/vxfr;
syrl = vyrl/vxrl;
syrr = vyrr/vxrr;

sfl = sqrt(SXfl^2+syfl^2);
sfr = sqrt(SXfr^2+syfr^2);
srl = sqrt(SXrl^2+syrl^2);
srr = sqrt(SXrr^2+syrr^2);

fxfl = (SXfl/sfl)*MF_Fx(veh,sfl,fzfl);
fxfr = (SXfr/sfr)*MF_Fx(veh,sfr,fzfr);
fxrl = (SXrl/srl)*MF_Fx(veh,srl,fzrl);
fxrr = (SXrr/srr)*MF_Fx(veh,srr,fzrr);
fxf = fxfl + fxfr;
fxr = fxrl + fxrr;

fyfl = (syfl/sfl)*MF_Fy(veh,sfl,fzfl);
fyfr = (syfr/sfr)*MF_Fy(veh,sfr,fzfr);
fyrl = (syrl/srl)*MF_Fy(veh,srl,fzrl);
fyrr = (syrr/srr)*MF_Fy(veh,srr,fzrr);
fyf = fyfl + fyfr;
fyr = fyrl + fyrr;


mufl = (fxfl^2+fyfl^2)^(0.5)/fzfl;
mufr = (fxfr^2+fyfr^2)^(0.5)/fzfr;
murl = (fxrl^2+fyrl^2)^(0.5)/fzrl;
murr = (fxrr^2+fyrr^2)^(0.5)/fzrr;


% Wheel torque
dfzf = 0.5*(fzfr-fzfl);
dfzr = 0.5*(fzrr-fzrl);

% kb = fzf/(veh.m*veh.g);
kb = 0.6;
Tf = kb*Tb;
Tfl = 0.5*Tf*(1-dfzf/fzf);
Tfr = 0.5*Tf*(1+dfzf/fzf);

Tr = Tt + (1-kb)*Tb;
Trl = 0.5*Tr*(1-dfzr/fzr);
Trr = 0.5*Tr*(1+dfzr/fzr);



%% Vehicle model - state derivatives
% Define derivatives of the state-space model

Fx = fxf*cosd + fxr - fyf*sind - f_drag;
Fy = fyf*cosd + fyr + fxf*sind;
ax = Fx/veh.m;
ay = Fy/veh.m;

Mz = veh.lf*(fyf*cosd+fxf*sind) - veh.lr*fyr + 0.5*veh.wt*((fxfr-fxfl)*cosd+(fyfl-fyfr)*sind) + 0.5*veh.wt*(fxrr-fxrl);

Ft = fxf*cosbd + fxr*cosb + fyf*sinbd + fyr*sinb - f_drag*cosb;
Fn = fyf*cosbd + fyr*cosb - fxf*sinbd - fxr*sinb + f_drag*sinb;


% Change of independent variable
chi = xi+beta;
sf = (1-n*kappa)/(V*cos(chi));

% State derivatives equations
xdot = [
    Ft/veh.m;
    Fn/(V*veh.m)-gamma;
    Mz/veh.Iz;
    V*sin(chi);
    gamma - kappa/sf;
    (Tfl-fxfl*veh.rw)/veh.Iw;
    (Tfr-fxfr*veh.rw)/veh.Iw;
    (Trl-fxrl*veh.rw)/veh.Iw;
    (Trr-fxrr*veh.rw)/veh.Iw;
    ]./x_s;

dx = sf*xdot;


%-Collect output variables
y_aero = [f_drag; f_lift];
y_tyre = [fxfl; fxfr; fxrl; fxrr; fyfl; fyfr; fyrl; fyrr; fzfl; fzfr; fzrl; fzrr];
y_slip = [SXfl; SXfr; SXrl; SXrr; atan(syfl); atan(syfr); atan(syrl); atan(syrr)];
y_acc = [ax; ay];
y_torque = [Tfl; Tfr; Trl; Trr];
y_mu = [mufl; mufr; murl; murr];
y_pwr = [Tfl*omega_fl; Tfr*omega_fr; Trl*omega_rl; Trr*omega_rr];
y_xdot = xdot;



function Fx = MF_Fx(veh,lambda,Fz)
Fx = (veh.d1x*Fz+veh.d2x)*sin(veh.Cx*atan(veh.Bx*lambda));
end

function Fy = MF_Fy(veh,alpha,Fz)
Fy = -(veh.d1y*Fz+veh.d2y)*sin(veh.Cy*atan(veh.By*alpha));
end












