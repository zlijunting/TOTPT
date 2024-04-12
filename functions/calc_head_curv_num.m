% function [phir, kappar] = calc_head_curv_num(track)
% -Inputs
% -track: path waypoints, [x, y] (N*2)
% 
% -Outputs
% -phir: heading angle (N*1)
% -kappar: curvature (N*1)
% 
% x = track(:,1);
% y = track(:,2);
% 
% n = size(track,1);
% 
% D = -eye(n);
% D(1:n-1,2:n) = D(1:n-1,2:n)+eye(n-1);
% D(end,end-1:end) = [-1 1];
% 
% ds = sqrt((D*x).^2+(D*y).^2);
% 
% D_1 = D./ds;
% D_2 = D*D_1./ds;
% 
% dx_1 = D_1*x;
% dx_2 = D_2*x;
% 
% dy_1 = D_1*y;
% dy_2 = D_2*y;
% 
% phir = atan2(dy_1,dx_1);
% kappar = (dx_1.*dy_2-dy_1.*dx_2)./((dx_1.^2+dy_1.^2).^(1.5));
% 
% end


function [phir, kappar] = calc_head_curv_num(track)
% CALC_HEAD_CURV_NUM Calculates the heading angle and curvature for a given path
% Input:
%   track - A matrix of path waypoints, with each row [x, y] representing a waypoint
% Output:
%   phir - A vector of heading angles for each waypoint
%   kappar - A vector of curvatures for each waypoint

% Extract x and y coordinates from track waypoints
x = track(:,1);
y = track(:,2);

% Determine the number of waypoints
n = size(track,1);

% Initialize a difference matrix for derivative calculation
D = -eye(n);
D(1:n-1,2:n) = D(1:n-1,2:n)+eye(n-1);
D(end,end-1:end) = [-1 1];

% Calculate the difference in distances (ds) between successive points
ds = sqrt((D*x).^2+(D*y).^2);

% Derivative matrices normalized by ds for first and second derivatives
D_1 = D./ds;
D_2 = D*D_1./ds;

% First and second derivatives of x and y
dx_1 = D_1*x;
dx_2 = D_2*x;
dy_1 = D_1*y;
dy_2 = D_2*y;

% Calculate the heading angle using atan2
phir = atan2(dy_1,dx_1);

% Calculate the curvature
kappar = (dx_1.*dy_2-dy_1.*dx_2)./((dx_1.^2+dy_1.^2).^(1.5));

end