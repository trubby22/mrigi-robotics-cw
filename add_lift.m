function [res] = add_lift(xyz_waypoints)
lift = [0 0 10.0];
lift = lift ./1000;
a = xyz_waypoints(1, :) + lift;
b = xyz_waypoints(end, :) + lift;
res = cat(1, a, xyz_waypoints, b);
end