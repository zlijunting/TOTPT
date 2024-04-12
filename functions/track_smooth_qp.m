function [n_opt, nor, dx, dy] = track_smooth_qp(track, n_bnd, weight)
% TRACK_SMOOTH_QP Optimizes lateral displacement of track waypoints using quadratic programming
% Input:
%   track - Track data as a matrix [x,y,wr,wl]
%   n_bnd - Lateral displacement boundaries for optimization
%   weight - Weighting structure for different terms in the objective function
% Output:
%   n_opt - Optimal lateral displacement for each waypoint
%   nor - Normal vectors at each waypoint
%   dx - Derivatives of x
%   dy - Derivatives of y

x = track(:,1);
y = track(:,2);
n = length(x);

% Calculate numerical derivatives for x and y coordinates
D = -1*eye(n);
D(1:n-1,2:n) = D(1:n-1,2:n) + eye(n-1);
D(n,n-1:n) = [-1 1];

ds = sqrt((D*x).^2+(D*y).^2);

D_1 = D./ds;
D_2 = D*D_1./ds;
D_3 = D*D_2./ds;
D_4 = D*D_3./ds;


% Calculater first to fourth order Derivative
dx_1 = D_1*x;
dx_2 = D_2*x;
dx_3 = D_3*x;
dx_4 = D_4*x;
dx = [dx_1,dx_2,dx_3,dx_4];

dy_1 = D_1*y;
dy_2 = D_2*y;
dy_3 = D_3*y;
dy_4 = D_4*y;
dy = [dy_1,dy_2,dy_3,dy_4];


% Calculate normal vector
phi = atan2(dy_1,dx_1);
nor = [-sin(phi) cos(phi)];

R_x = diag(nor(:,1));
R_y = diag(nor(:,2));

% Formulate the QP problem
Q_e = weight.lambda_e^2*eye(n);
Q_xy = weight.lambda_1^2*(D_1'*D_1) + weight.lambda_2^2*(D_2'*D_2)...
     + weight.lambda_3^2*(D_3'*D_3) + weight.lambda_4^2*(D_4'*D_4);

Q_xy_tilde = R_x*Q_xy*R_x + R_y*Q_xy*R_y;

H = Q_e + Q_xy_tilde;
H = (H + H')/2;   % Ensure H is symmetric

f_T = (x'*Q_xy*R_x + y'*Q_xy*R_y);
f = f_T';

opt = optimoptions('quadprog','Display','off');
n_opt = quadprog(H,f,[],[],[],[],n_bnd(:,1),n_bnd(:,2),[],opt);


end

