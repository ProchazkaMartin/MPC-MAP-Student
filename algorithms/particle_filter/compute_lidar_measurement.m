function [measurement] = compute_lidar_measurement(map, pose, lidar_config)
%COMPUTE_MEASUREMENTS Summary of this function goes here

measurement = zeros(1, length(lidar_config));

    for i = 1 : length(lidar_config)
        phi = pose(3) + lidar_config(i);
        intersections = ray_cast(pose(1:2), map.walls, phi)-pose(1:2);
        d = vecnorm(intersections, 2, 2);
        measurement(i) = min(d);
    end

end

