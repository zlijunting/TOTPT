function [s, ds, s_end] = calc_length_xy(track)
% CALC_LENGTH_XY Calculates the cumulative length along a path
% Input:
%   track - A matrix of path waypoints, with each row [x, y] representing a waypoint
% Output:
%   s - A vector of cumulative lengths from the start to each waypoint
%   ds - The distances between successive waypoints
%   s_end - The total length of the path

% Extract x and y coordinates from track waypoints
x = track(:,1);
y = track(:,2);

% Calculate Euclidean distances between successive waypoints
ds = sqrt(diff(x).^2 + diff(y).^2);

% Calculate the cumulative sum of distances, starting at 0 for the first waypoint
s = [0; cumsum(ds)];

% Total length of the path
s_end = s(end);

end