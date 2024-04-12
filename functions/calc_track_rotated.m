function track_rotated = calc_track_rotated(track, theta)

track_xy = track(:,1:2);

po = (track(1,1:2))';

Rz = [cos(theta) -sin(theta);
      sin(theta)  cos(theta)];

pe = track_xy'-po;

track_xy_rotated = (Rz*pe + po)';

track_rotated = [track_xy_rotated track(:,3:end)];

end
