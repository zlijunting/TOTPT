% function [track_bnd_r, track_bnd_l] = calc_track_bnd(track_data)
% %-track_data, [x, y, wr, wl] (N*4)
% %
% %-Outputs
% %-track_bnd_r: right boundary coordinates
% %-track_bnd_r: left boundary coordinates
% 
% [phi, ~] = calc_head_curv_num(track_data(:,1:2));
% 
% nor_vec = [-sin(phi) cos(phi)];
% 
% track_bnd_r = track_data(:,1:2) - track_data(:,3).*nor_vec;
% track_bnd_l = track_data(:,1:2) + track_data(:,4).*nor_vec;
% 
% 
% end

function [track_bnd_r, track_bnd_l] = calc_track_bnd(track_data)
% CALC_TRACK_BND Calculates the right and left boundaries of a track
% Input:
%   track_data - A matrix containing track waypoints and widths, [x, y, wr, wl] (N*4)
% Output:
%   track_bnd_r - Right boundary coordinates of the track
%   track_bnd_l - Left boundary coordinates of the track

% Calculate the heading angle for each track waypoint
[phi, ~] = calc_head_curv_num(track_data(:,1:2));

% Calculate the normal vector for each waypoint
nor_vec = [-sin(phi) cos(phi)];

% Calculate the right boundary by subtracting the right width scaled by the normal vector
track_bnd_r = track_data(:,1:2) - track_data(:,3).*nor_vec;

% Calculate the left boundary by adding the left width scaled by the normal vector
track_bnd_l = track_data(:,1:2) + track_data(:,4).*nor_vec;

end