clear all, clc

%%
if(~isdeployed)
  cd(fileparts(which('nmpc_gen')));
end

if isfile('c_generated_code\CMakeCache.txt')
    delete('c_generated_code\CMakeCache.txt')
end
acados_env_variables_windows
% check_acados_requirements()

%% Parameters

veh = veh_params();

%%
opt_e = 1e-3;
V0 = 1;


%% model dynamics
model = ocp_model_racecar();

%% horizon parameters
N = model.N;
T = model.T; % time horizon length
Ts = model.Ts;

nx = model.nx;
nu = model.nu;
np = model.np;

x_s = model.x_s;
u_s = model.u_s;

%% model to create the solver
ocp_model = acados_ocp_model();

%% acados ocp model
ocp_model.set('name', model.name);
ocp_model.set('T', model.T);

% symbolics
ocp_model.set('sym_x', model.x);
ocp_model.set('sym_xdot', model.xdot);
ocp_model.set('sym_u', model.u);
ocp_model.set('sym_p', model.p);


% integrator type
sim_method = 'irk'; % erk, irk, irk_gnsf

% dynamics
if (strcmp(sim_method, 'erk'))
	ocp_model.set('dyn_type', 'explicit');
	ocp_model.set('dyn_expr_f', model.expr_f_expl);
elseif (strcmp(sim_method, 'irk'))
	ocp_model.set('dyn_type', 'implicit');
	ocp_model.set('dyn_expr_f', model.expr_f_impl);
end


%% cost function

ocp_model.set('cost_type','ext_cost');
ocp_model.set('cost_type_e','ext_cost');
ocp_model.set('cost_expr_ext_cost', model.expr_ext_cost);
ocp_model.set('cost_expr_ext_cost_e', model.expr_ext_cost_e);
ny_x = 3;
ny_e = ny_x;
ny_u = 3;
ny = ny_x + ny_u;

nbx = nx;
Jbx = eye(nx);
ocp_model.set('constr_Jbx', Jbx);
ocp_model.set('constr_lbx', model.x_min);
ocp_model.set('constr_ubx', model.x_max);


nbu = 3;
Jbu = eye(nbu,nu);
ocp_model.set('constr_Jbu', Jbu);
ocp_model.set('constr_lbu', model.u_min);
ocp_model.set('constr_ubu', model.u_max);

nh = 5;
ocp_model.set('constr_expr_h', model.constraint.expr);
ocp_model.set('constr_lh', [0, 0, 0, 0, 0]);
ocp_model.set('constr_uh', [0, 1.0, 1.0, 1.0, 1.0]);

Jsh = eye(nh);
ocp_model.set('constr_Jsh', Jsh);
ocp_model.set('cost_zl', zeros(nh,1));
ocp_model.set('cost_zu', zeros(nh,1));
ocp_model.set('cost_Zl', diag([1 10 10 10 10]));
ocp_model.set('cost_Zu', diag([1 10 10 10 10]));



%%
% set intial references
x0 = zeros(nx,1);
x0(1) = V0;
x0(9) = 1200; % Tt0
x0 = x0./model.x_s;
ocp_model.set('constr_x0', x0);


%% Solver parameters
compile_interface = 'auto';
codgen_model = 'true';
nlp_solver = 'sqp_rti'; % sqp, sqp_rti
% nlp_solver = 'sqp_rti'; % sqp, sqp_rti
qp_solver = 'full_condensing_hpipm'; % full_condensing_hpipm, partial_condensing_hpipm, full_condensing_qpoases
nlp_solver_exact_hessian = 'true'; % false=gauss_newton, true=exact   

% qp_solver_cond_N = N/5; % for partial condensing
% regularize_method = 'no_regularize';


%% acados ocp set opts
ocp_opts = acados_ocp_opts();
ocp_opts.set('compile_interface', compile_interface);
ocp_opts.set('codgen_model', codgen_model);
ocp_opts.set('param_scheme_N', model.N);
ocp_opts.set('nlp_solver', nlp_solver);
ocp_opts.set('nlp_solver_exact_hessian', nlp_solver_exact_hessian); 
ocp_opts.set('sim_method', sim_method);
ocp_opts.set('sim_method_num_stages', 2);
ocp_opts.set('sim_method_num_steps', 1)
ocp_opts.set('qp_solver', qp_solver);
% ocp_opts.set('qp_solver_iter_max', 50);

% ocp_opts.set('regularize_method', regularize_method);
% ocp_opts.set('qp_solver_cond_N', qp_solver_cond_N);
% ocp_opts.set('levenberg_marquardt', 1e-4);

% nlp solver
eps_nlp = 1e-4;
ocp_opts.set('nlp_solver_max_iter', 30);
ocp_opts.set('nlp_solver_tol_stat', eps_nlp);
ocp_opts.set('nlp_solver_tol_eq', eps_nlp);
ocp_opts.set('nlp_solver_tol_ineq', eps_nlp);
ocp_opts.set('nlp_solver_tol_comp', eps_nlp);
% ... see ocp_opts.opts_struct to see what other fields can be set

p0 = [0, V0];
ocp_opts.set('parameter_values', p0);


%% get available simulink_opts with default options
simulink_opts = get_acados_simulink_opts;

% manipulate simulink_opts

% inputs
% simulink_opts.inputs.cost_W = 1;
% simulink_opts.inputs.cost_W_e = 1;
% simulink_opts.inputs.x_init = 1;
% simulink_opts.inputs.reset_solver = 1;
simulink_opts.inputs.lbx = 1;
simulink_opts.inputs.ubx = 1;
simulink_opts.inputs.lh = 0;
simulink_opts.inputs.uh = 0;
simulink_opts.inputs.lbu = 0;
simulink_opts.inputs.ubu = 0;

% outputs
simulink_opts.outputs.utraj = 1;
simulink_opts.outputs.xtraj = 1;
simulink_opts.outputs.cost_value = 1;
simulink_opts.outputs.KKT_residual = 0;
simulink_opts.outputs.KKT_residuals = 1;


%% Generate acados OCP
% rmdir('c_generated_code','s')
% rmdir('build','s')

ocp = acados_ocp(ocp_model, ocp_opts, simulink_opts);


%% set initial trajectory
x_init = zeros(N+1,nx);
x_init(:,1) = V0*ones(N+1,1);
x_init(:,9) = 1200*ones(N+1,1);

x_init = x_init./model.x_s';

u_init = zeros(N,nu);

ocp.set('init_x', x_init');
ocp.set('init_u', u_init');
ocp.set('init_pi', zeros(nx, N));


%% Compile Sfunctions
cd c_generated_code

make_sfun; % ocp solver
% make_sfun_sim; % integrator

copyfile('acados_solver_sfunction_nmpc.mexw64','..\sim')
cd ..\sim
