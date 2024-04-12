% clear, clc, close all
import casadi.*

% rel_track_path = ('\tracks_smooth\');

veh = veh_params();

% track_name = 'MyTrack';
% Track options: Austin, BrandsHatch, Budapest, Catalunya, Hockenheim, IMS, 
% Melbourne, MexicoCity, Montreal, Monza, MoscowRaceway, Norisring, 
% Nuerburgring, Oschersleben, Sakhir, SaoPaulo, Sepang, Shanghai, 
% Silverstone, Sochi, Spa, Spielberg, Suzuka, YasMarina, Zandvoort

track = readtable(['tracks_smooth\' track_name '_smooth.csv']);

elapsedTime = 0; tic %Start time counter

%%
OPT_e = 1e-3;

%% Vehicle model
run('tro_model_racecar.m');
% tro_model_racecar

% Continuous time dynamics and objective function
f_dyn = Function('f_dyn',{x,u,z,pv},{dx,sf},{'x','u','z','pv'},{'dx','sf'});

% Change of variable
f_sf = Function('sf',{x,kappa},{sf},{'x','kappa'},{'sf'});

%% Optimal Control Problem - Path Constraints

nh = 6; % number of constraints

% friciton circle
mux = veh.mux;
muy = veh.muy;
mufl_ineq = 1 - ((fxfl^2)/(mux^2*fzfl^2) + (fyfl^2)/(muy^2*fzfl^2));
mufr_ineq = 1 - ((fxfr^2)/(mux^2*fzfr^2) + (fyfr^2)/(muy^2*fzfr^2));
murl_ineq = 1 - ((fxrl^2)/(mux^2*fzrl^2) + (fyrl^2)/(muy^2*fzrl^2));
murr_ineq = 1 - ((fxrr^2)/(mux^2*fzrr^2) + (fyrr^2)/(muy^2*fzrr^2));

% longitudinal acc.
ax_eq = (ax_bar - ax)/ax_bar_s;

% lateral acc.
ay_eq = (ay_bar - ay)/ay_bar_s;

% brake and throttle overlap
BrTh = Tt_n*Tb_n;

% engine power
pwr_rl = Trl*omega_rl/(Tt_s*omega_s);
pwr_rr = Trr*omega_rr/(Tt_s*omega_s);
pwr_lb = -500e3/(Tt_s*omega_s);
pwr_ub = 150e3/(Tt_s*omega_s);

% Collect constraints
h = [mufl_ineq mufr_ineq murl_ineq murr_ineq ax_eq ay_eq BrTh pwr_rl pwr_rr]';
h_lb = [  0   0   0   0 0-OPT_e 0-OPT_e 0-OPT_e pwr_lb pwr_lb]';
h_ub = [inf inf inf inf 0+OPT_e 0+OPT_e 0+OPT_e pwr_ub pwr_ub]';

% Path constraints
h_eq = Function('h_eq', {x, u, z, pv}, {h}, {'x','u','z','pv'}, {'h'}); 


%% NLP - Collocation, polynomial coefficients

d = 3; %-degree of interpolating polynomial

% Collocation points in normalised interval [0,1]
tau = collocation_points(d, 'legendre');

% Collocation linear maps
[C,D,B] = collocation_coeff(tau); 


%% NLP - Collocation, discretisation

%-number of grid intervals (the number of points, Xk, is N+1)
N = length(track.s) - 1;

%-length of the discretisation interval
h = diff(track.s)'; % 1xN

step = 1:N+1;
step_col = kron(ones(1,N),tau) + kron(1:N,ones(1,d));
step_full = kron(ones(1,N),[0 tau]) + kron(1:N,ones(1,d+1));
step_full = [step_full N+1];


%-interpolate path curvature and track boundaries at knot and collocation points
kappa_interp = interpolant('kappa_interp','linear',{step},track.kappa);
kappa_knot = kappa_interp(step);
kappa_col = kappa_interp(step_col);

wr_interp = interpolant('wr_interp','linear',{step},track.wr);
wr_knot = wr_interp(step);
wr_col = wr_interp(step_col);

wl_interp = interpolant('wl_interp','linear',{step},track.wl);
wl_knot = wl_interp(step);
wl_col = wl_interp(step_col);

s_interp = interpolant('s_interp','linear',{step},track.s);



%% NLP - formulation

% Decision variables
Xk = SX.sym('Xk', nx,N+1);   % States
Uk = SX.sym('Uk', nu,N+1);   % Inputs
Zk = SX.sym('Yk', nz,N+1);   % Aux variables
Xkj = SX.sym('Xkj', nx,N*d); % Helper states for the collocation constraints


% Linear interpolation
duk = diff(Uk')'./repmat(h,nu,1); %derivative of the input at each interval (du/ds)
dyk = diff(Zk')'./repmat(h,nz,1); %derivative of the aux. variables at each interval (dy/ds)
% Second derivatives (for regularisarion and minimising oscillations)
duk2 = diff(duk')'; duk2 = [duk2 duk2(:,end)];
dyk2 = diff(dyk')'; dyk2 = [dyk2 dyk2(:,end)]; 


% Scaling of the objective function
J_s = 1; %Note that using 's_full(end)/vx_s' here instead of 1 would make the objective function be close to 1;
% J_s = 10;
% J_s = 0.1;


%% NLP - boundary conditions

% if vehicle starts at center of the track, n0 = 0;

% Optimal Control Problem - Boundary constraints
V_start = 1;
% V_start = nan;  % no limit on initial velocity
Xi = [V_start; nan; nan; nan; nan; nan; nan; nan; nan];
%-final
Vf = nan;
Xf = [Vf; nan; nan; nan; nan; nan; nan; nan; nan];


x0_min = max(x_min, Xi./x_s-OPT_e); 
x0_max = min(x_max, Xi./x_s+OPT_e);
xf_min = max(x_min, Xf./x_s-OPT_e); 
xf_max = min(x_max, Xf./x_s+OPT_e);

% %-Initial state
% gb = {Xk(:,1)}; lbg = x0_min; ubg = x0_max; %Note that gb, lbg and ubg are being initialised here
% 
% %-Final state
% gb = [gb(:); {Xk(:,end)}]; lbg = [lbg; xf_min]; ubg = [ubg; xf_max];

%-boundary constraints
gb = [{Xk(:,1)}; {Xk(:,end)}]; lbg = [x0_min; xf_min]; ubg = [x0_max; xf_max];



%% NLP - initial guesses (constant values)
%-States
V_0 = V_start*ones(1,N+1);
beta_0 = OPT_e*ones(1,N+1);
gamma_0 = OPT_e*ones(1,N+1);
n_0 = OPT_e*ones(1,N+1);
xi_0 = zeros(1,N+1);
omega_fl_0 = V_0/veh.rw;
omega_fr_0 = V_0/veh.rw;
omega_rl_0 = V_0/veh.rw;
omega_rr_0 = V_0/veh.rw;

%-Inputs
Tt_0 = 100*ones(1,N+1); 
Tb_0 = OPT_e*ones(1,N+1); 
delta_0 = OPT_e*ones(1,N+1);

%-Aux variables
ax_bar_0 = OPT_e*ones(1,N+1);
ay_bar_0 = OPT_e*ones(1,N+1);

% Collect initial guesses
x0 = [V_0; beta_0; gamma_0; n_0; xi_0; omega_fl_0; omega_fr_0; omega_rl_0; omega_rr_0]./x_s;
xc0 = reshape(kron(x0(:,1:end-1),ones(1,d)),d*N*nx,1); %constant interpolation between x0
u0 = [Tt_0; Tb_0; delta_0]./u_s;
z0 = [ax_bar_0; ay_bar_0]./z_s;


%% NLP - formulation, collocation constraints, path constraints and objective function

% Monitored variables
dt = SX.zeros(1,N);

% Loop over discretisation points to create collocation constraints
gck = {}; % Collector for collocation constraints
J = 0; % Initialise objective function

for k = 0:N-1
    % Collocation constraints
    X = [Xk(:,k+1) Xkj(:,d*k+(1:d))]; % Concatenate states

    %-Dynamics
    %%-calculate derivatives of the approximating polynomial at the collocation points
    dPi = X*C; 

    %%-calculate derivatives of the system at the collocation points
    [dXkj, Qk] = f_dyn(Xkj(:,d*k+(1:d)), Uk(:,k+1), Zk(:,k+1), kappa_col(d*k+(1:d))); 

    %-State of approximating polynomial the end of the colocation interval
    Xk_end = X*D;

    gck = [gck(:); {h(k+1)*dXkj(:) - dPi(:)}; {Xk_end-Xk(:,k+2)}]; % Add collocation constraints

    % Integrate quadrature function
    dtk = Qk*B*h(k+1);
    J = J + dtk + sumsqr(rdu.*duk(:,k+1)) + sumsqr(rdy.*dyk(:,k+1));
    
    % Collect contribution to time of each interval
    dt(k+1) = dtk;

end

% veh.ws = 0.5;
% state limits at knot points
Xk_min = repmat(x_min,N+1,1);
Xk_min(4:nx:end) = full(-wr_interp(step)+(veh.wt/2+veh.ws))/n_s;

Xk_max = repmat(x_max,N+1,1);
Xk_max(4:nx:end) = full(wl_interp(step)-(veh.wt/2+veh.ws))/n_s;

% state limits at collocation points
Xkj_min = repmat(x_min,N*d,1);
Xkj_min(4:nx:end) = full(-wr_interp(step_col)+(veh.wt/2+veh.ws))/n_s;

Xkj_max = repmat(x_max,N*d,1);
Xkj_max(4:nx:end) = full(wl_interp(step_col)-(veh.wt/2+veh.ws))/n_s;


% Path constraints
ghk = h_eq(Xk, Uk, Zk, pv); 
ghk = {ghk(:)};

% Rate of inputs constraints
Sfk = f_sf(Xk,kappa_knot); % calculate Sf to 'undo' the change of independent variable
dsdt_k = 1./Sfk;
duk_t = duk.*repmat(dsdt_k(1:end-1),nu,1); % calculate time derivatives: du/dt = du/ds * 1/sf = du/ds * ds/dt
gduk = {duk_t(:)};


%% NLP - define NLP variables
%-decision variables
w = [Xk(:); Uk(:); Zk(:); Xkj(:)]; % Collect all the decision variables [states; inputs; aux. variables; helper states]
lbw = [Xk_min; repmat(u_min(:),N+1,1); repmat(z_min(:),N+1,1); Xkj_min]; % Lower bounds
ubw = [Xk_max; repmat(u_max(:),N+1,1); repmat(z_max(:),N+1,1); Xkj_max]; % Upper bounds
w0 = [x0(:); u0(:); z0(:); xc0(:)]; % Collect initial guesses for the decision variables

assert(size(lbw,1) == size(w,1));
assert(size(ubw,1) == size(w,1));
assert(size(w0,1) == size(w,1));

%-constraints
g = [gb(:);gck(:);ghk(:);gduk(:)]; %[boundary conditions; collocation constraints; path constraints; rate of inputs]
g = vertcat(g{:}); 
lbg = [lbg; zeros((d+1)*N*nx,1);repmat(h_lb,N+1,1);repmat(duk_lb,N,1)]; % Lower bounds 
ubg = [ubg; zeros((d+1)*N*nx,1);repmat(h_ub,N+1,1);repmat(duk_ub,N,1)]; % Upper bounds 

assert(size(lbg,1) == size(g,1));
assert(size(lbg,1) == size(g,1));


%% NLP - output functions
f_dt_opt = Function('f_dt_opt',{w},{dt});
f_aero = Function('f_aero',{x,u,z,pv}, {y_aero});
f_tyre = Function('f_tyre',{x,u,z,pv},{y_tyre});
f_slip = Function('f_alpha',{x,u,z,pv},{y_slip});
f_acc = Function('f_acc',{x,u,z,pv},{y_acc});
f_torque = Function('f_torque',{x,u,z,pv},{y_torque});
f_mu = Function('f_mu',{x,u,z,pv},{y_mu});
f_pwr = Function('f_pwr',{x,u,z,pv},{y_pwr});
f_xdot = Function('f_xdot',{x,u,z,pv},{y_xdot});



%% NLP - solve 
opts = struct;
opts.ipopt.max_iter = 1000; %max number of iterations.
% opts.ipopt.fixed_variable_treatment = 'make_constraint'; %with default options, multipliers for the decision variables are wrong for equality constraints. Change the Tfixed_variable_treatmentT to hmake_constrainte or  relax_boundsf to obtain correct results (comment from Casadi's documentation. No difference has been observed in the solution or the solver when disabling this option)
opts.ipopt.tol = 1e-3;
% opts.ipopt.acceptable_tol = 1e-4;
% opts.ipopt.print_options_documentation = 'yes';
% options = struct('ipopt',struct('print_level',0),'print_time',false);


%-nlp problem [objective function 'f', decision variables 'x', constraints 'g']
nlp = struct('f', J, 'x', w, 'g', g); 

% Create solver
solver = nlpsol('solver', 'ipopt', nlp, opts);

elapsedTime(end+1) = toc;
tic;

% Solve the NLP
sol_nlp = solver('x0', w0, 'lbx', lbw, 'ubx', ubw, 'lbg', lbg, 'ubg', ubg);

elapsedTime(end+1) = toc;
tic
%% Postprocessing - Collect NLP variables

sol = struct;

%-Collect decision variables in numeric array
sol.w_opt = full(sol_nlp.x);
sol.x_opt = reshape(sol.w_opt(1:nx*(N+1)),nx,N+1).*x_s;
sol.u_opt = reshape(sol.w_opt(nx*(N+1)+(1:nu*(N+1))),nu,N+1).*u_s;
sol.z_opt = reshape(sol.w_opt(nx*(N+1)+nu*(N+1)+(1:nz*(N+1))),nz,N+1).*z_s;
sol.xc_opt = reshape(sol.w_opt(nx*(N+1)+nu*(N+1)+nz*(N+1)+1:end),nx,N*d).*x_s;


%% Postprocessing - Reconstruct solution
% Collect values of x at all interpolation points 
%States, x_full = [X1 X11...X1j...X1d,..., Xk Xk1...Xkj...Xkd,..., XN XN1..XNj...XNd, XN+1]
sol.x_full = kron(sol.x_opt(:,1:N), [1 zeros(1,d)]) + reshape([zeros(nx,N); reshape(sol.xc_opt,nx*d,N)],nx,[]);
sol.x_full = [sol.x_full sol.x_opt(:,N+1)];
sol.u_full = interp1(step',sol.u_opt',step_full)';
sol.z_full = interp1(step',sol.z_opt',step_full)';

%% Postprocessing - Get output states
sol.aero = full(f_aero(sol.x_opt./x_s,sol.u_opt./u_s,sol.z_opt./z_s,kappa_knot))';
sol.tyre = full(f_tyre(sol.x_opt./x_s,sol.u_opt./u_s,sol.z_opt./z_s,kappa_knot))';
sol.slip = full(f_slip(sol.x_opt./x_s,sol.u_opt./u_s,sol.z_opt./z_s,kappa_knot))';
sol.acc = full(f_acc(sol.x_opt./x_s,sol.u_opt./u_s,sol.z_opt./z_s,kappa_knot))';
sol.torque = full(f_torque(sol.x_opt./x_s,sol.u_opt./u_s,sol.z_opt./z_s,kappa_knot))';
sol.mu = full(f_mu(sol.x_opt./x_s,sol.u_opt./u_s,sol.z_opt./z_s,kappa_knot))';
sol.pwr = full(f_pwr(sol.x_opt./x_s,sol.u_opt./u_s,sol.z_opt./z_s,kappa_knot))';
sol.xdot = (full(f_xdot(sol.x_opt./x_s,sol.u_opt./u_s,sol.z_opt./z_s,kappa_knot)).*x_s)';


%% Postprocessing - Get lap time
% Time
dt_opt_val = full(f_dt_opt(sol.w_opt));
%-time at grid points
sol.t_opt = [0 cumsum(dt_opt_val)];


%% Postprocessing - Reconstruct optimal racetrack
n_opt = sol.x_opt(4,:)';
wr_opt = track.wr + n_opt;
wl_opt = track.wl - n_opt;

nor = [-sin(track.phi) cos(track.phi)];
track.x_opt = track.x + diag(nor(:,1))*n_opt;
track.y_opt = track.y + diag(nor(:,2))*n_opt;
[s_opt, ~] = calc_length_xy([track.x_opt,track.y_opt]);
[phi_opt, kappa_opt] = calc_head_curv_num([track.x_opt,track.y_opt]);

v_opt = sol.x_opt(1,:)';
sol.track_opt = [track.x_opt track.y_opt wr_opt wl_opt s_opt phi_opt kappa_opt v_opt];

%% Save data
track_opt_table = array2table(sol.track_opt);
track_opt_table.Properties.VariableNames = {'x','y','wr','wl','s','phi','kappa','v'};

if ~exist(fullfile(pwd,'tracks_mintime'), 'dir')
   mkdir('tracks_mintime')
end
writetable(track_opt_table,['tracks_mintime\' track_name '_mintime' '.csv'])


if ~exist(fullfile(pwd,'tro_sol'), 'dir')
   mkdir('tro_sol')
end
save(['tro_sol\sol_' track_name '.mat'],'sol')