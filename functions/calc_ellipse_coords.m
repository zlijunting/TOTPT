% function [x, y] = calc_ellipse_coords(x0, y0, a, b)
% %-Inputs
% %-x0, y0: %-ellipse centre coordinates 
% %-a: horizontal radius
% %-b: vertical radius
% %
% %-Outputs
% %-x, y: %-ellipse coordinates
% 
% t = -pi:0.01:pi;
% x = x0 + a*cos(t);
% y = y0 + b*sin(t);
% 
% end

function [x, y] = calc_ellipse_coords(x0, y0, a, b)
% CALC_ELLIPSE_COORDS Generates the x and y coordinates of an ellipse
% Input:
%   x0, y0 - The center coordinates of the ellipse
%   a - The horizontal radius of the ellipse
%   b - The vertical radius of the ellipse
% Output:
%   x, y - The x and y coordinates of the ellipse

% Define the parameter t which ranges from -pi to pi to cover the full ellipse
t = -pi:0.01:pi;

% Calculate the x coordinates of the ellipse
x = x0 + a*cos(t);

% Calculate the y coordinates of the ellipse
y = y0 + b*sin(t);

end