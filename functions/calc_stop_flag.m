function index_onelap = calc_stop_flag(track)
% CALC_STOP_FLAG Determines the index of the waypoint that signifies the end of one lap
% Input:
%   track - A matrix of path waypoints, with each row [x, y] representing a waypoint
% Output:
%   index_onelap - The index of the waypoint closest to the start, marking the end of one lap

% Starting point
p0 = track(1,1:2);

% Number of waypoints to skip to avoid immediate vicinity of the start
index_skip = 100;

% Sub-array of waypoints excluding the first few to avoid selecting the start point
waypts_search = track(1+index_skip:end,1:2);

% Calculate the distance of each point in waypts_search from the starting point
dist = sqrt(sum((waypts_search - p0).^2, 2));

% Find the index of the closest point to the start in the waypts_search array
[~, ind_min] = min(dist);

% Adjust the index to account for the skipped waypoints
index_onelap = ind_min + index_skip;

end