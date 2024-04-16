function [track_interp, s_interp] = calc_track_interp(track, ds_interp)
% CALC_TRACK_INTERP Interpolates track data at specified intervals
% Input:
%   track - Original track data as a matrix [x, y, wr, wl, ...], n*i
%   ds_interp - Step size for spline interpolation
% Output:
%   track_interp - Interpolated track data at the specified intervals
%   s_interp - Query points used for the interpolation

% Calculate the cumulative length along the path
[s, ~] = calc_length_xy(track(:,1:2));

% Calculate the endpoint for interpolation based on the step size
s_end_interp = fix(s(end)/ds_interp)*ds_interp;

% Generate a vector of query points for interpolation
s_interp = (0:ds_interp:s_end_interp)';

% Create a spline representation of the original track
track_pp = spline(s', track');

% Evaluate the spline at the interpolation points
track_interp = fnval(track_pp, s_interp)';

end