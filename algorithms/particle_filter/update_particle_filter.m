function [particles] = update_particle_filter(read_only_vars, public_vars)
%UPDATE_PARTICLE_FILTER Summary of this function goes here

particles = public_vars.particles;
if ~public_vars.use_pf
    return;
end

% I. Prediction
for i=1:size(particles, 1)
    particles(i,:) = predict_pose(particles(i,:), public_vars.motion_vector, read_only_vars);
end

% II. Correction
measurements = zeros(size(particles,1), length(read_only_vars.lidar_config));
for i=1:size(particles, 1)
    meas = compute_lidar_measurement(read_only_vars.map, particles(i,:), read_only_vars.lidar_config);
    offset = offset_rotation(meas, read_only_vars.lidar_distances);
    meas = circshift(meas, offset);
    deltaPhi = -offset / length(read_only_vars.lidar_distances) * 2 * pi;
    particles(i,3) = mod(particles(i,3) + deltaPhi, 2*pi);
    measurements(i,:) = meas;
end
weights = weight_particles(measurements, read_only_vars.lidar_distances);

for i=1:size(particles, 1)
    if ~is_inside(particles(i,:), read_only_vars)
        weights(i) = 0.00001;
    end
end

% III. Resampling
% if mod(read_only_vars.counter, 2) == 0
particles = resample_particles(particles, weights, read_only_vars);
% end

end

