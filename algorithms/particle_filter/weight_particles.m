function [weights] = weight_particles(particle_measurements, lidar_distances)
%WEIGHT_PARTICLES Summary of this function goes here

pm = particle_measurements;
pm(isinf(pm)) = 10;
pm(isnan(pm)) = 10;

ld = lidar_distances;
ld(isinf(ld)) = 10;
ld(isnan(ld)) = 10;

% p = exp(-0.5*((particle_measurements - lidar_distances)/5).^2);
% weights = prod(p,2);

weights = 1 ./ sqrt( sum((pm - ld).^2, 2) );

weights(isnan(weights)) = 0.00001;
weights(weights<0.00001) = 0.00001;
weights = weights / sum(weights);

end

