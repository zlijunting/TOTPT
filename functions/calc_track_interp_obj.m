function [track_obj] = calc_track_interp_obj(track,ds_interp)
%-track_data: [x, y, wr, wl, ...], n*i;
%-ds_interp: step size for spline interoplation
%
%-Outputs
%-track_interp: interpolated track data for given ds
%-s_interp: query points for interpolation

[s, ~] = calc_length_xy(track(:,1:2));
s_end_interp = fix(s(end)/ds_interp)*ds_interp;
s_interp = (0:ds_interp:s_end_interp)';

track_pp = spline(s',track');

track_interp = fnval(track_pp,s_interp)';

track_interp = [track_interp; track_interp(1,:)];

[s] = calc_length_xy(track)
[phi, kappa] = calc_head_curv_num(track_interp);

track_obj.x = track_interp(:,1);
track_obj.y = track_interp(:,2);
track_obj.wr = track_interp(:,3);
track_obj.wl = track_interp(:,4);
track_obj.s = s_interp;
track_obj.phi = phi;
track_obj.kappa = kappa;


end