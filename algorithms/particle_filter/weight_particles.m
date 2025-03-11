function [weights] = weight_particles(particle_measurements, lidar_distances)
%WEIGHT_PARTICLES Summary of this function goes here

% p = exp(-0.5*((particle_measurements - lidar_distances)/5).^2);
% weights = prod(p,2);

weights = 1 ./ sqrt( sum((particle_measurements - lidar_distances).^2, 2) );
weights(isnan(weights)) = 0.000001;
sum(weights)

weights = weights / sum(weights);
s = sort(weights);
s(end)

end

