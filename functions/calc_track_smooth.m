function [n_opt_total, track_opt] = calc_track_smooth(track, n_bnd, weight, iter)
% CALC_TRACK_SMOOTH Smooths track data through iterative optimization
% Input:
%   track - Initial track data as a matrix [x, y, wr, wl]
%   n_bnd - Boundary constraints for smoothing
%   weight - Weighting factor for the optimization
%   iter - Number of iterations for the smoothing process
% Output:
%   n_opt_total - Cumulative optimization adjustments
%   track_opt - Optimized track data

N = size(track,1);
n_opt_total = zeros(N,1);

for i = 1:iter
    % Call the quadratic programming optimization function
    [n_opt, nor, ~, ~] = track_smooth_qp(track, n_bnd, weight);

    % Accumulate the optimization results
    n_opt_total = n_opt_total + n_opt;
    
    % Adjust track coordinates based on the optimization
    x_opt = track(:,1) + diag(nor(:,1))*n_opt;
    y_opt = track(:,2) + diag(nor(:,2))*n_opt;

    % Update the track with optimized coordinates
    track(:,1) = x_opt;
    track(:,2) = y_opt;

    % Adjust boundary constraints for the next iteration
    n_bnd(:,1) = n_bnd(:,1) - n_opt;    % Right boundary
    n_bnd(:,2) = n_bnd(:,2) - n_opt;    % Left boundary
end

% Update boundary widths based on total optimization adjustments
wr_opt = track(:,3) + n_opt_total;
wl_opt = track(:,4) - n_opt_total;

% Compile the optimized track data
track_opt = [x_opt, y_opt, wr_opt, wl_opt];

end