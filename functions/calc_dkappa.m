function [dkappa_1, dkappa_2] = calc_dkappa(x,y)
% CALC_DKAPPA Calculates the change in curvature (dkappa) for a given set of x and y coordinates
% Input:
%   x - A vector of x coordinates
%   y - A vector of y coordinates
% Output:
%   dkappa - A vector containing the first and second changes in curvature of the path defined by x and y

n = length(x); % Length of the coordinate vectors

% Initialize a difference matrix for derivative calculation
D = -1*eye(n);
D(1:n-1,2:n) = D(1:n-1,2:n) + eye(n-1);
D(n,n-1:n) = [-1 1];

% Calculate the difference in distances (ds) between successive points
ds = sqrt((D*x).^2+(D*y).^2);

% Derivative matrices normalized by ds
D_1 = D./ds;
D_2 = D*D_1./ds;
D_3 = D*D_2./ds;
D_4 = D*D_3./ds;

% First to fourth order derivatives of x
dx_1 = D_1*x;
dx_2 = D_2*x;
dx_3 = D_3*x;
dx_4 = D_4*x;
dx = [dx_1,dx_2,dx_3,dx_4]; % Compilation of x derivatives

% First to fourth order derivatives of y
dy_1 = D_1*y;
dy_2 = D_2*y;
dy_3 = D_3*y;
dy_4 = D_4*y;
dy = [dy_1,dy_2,dy_3,dy_4]; % Compilation of y derivatives

% Calculations of first and second change in curvature
dkappa_1 = dx_1.*dy_3 - dy_1.*dx_3;
dkappa_2 = dx_2.*dy_3 + dx_1.*dy_4 - dx_4.*dy_1 - dx_3.*dy_2;


end