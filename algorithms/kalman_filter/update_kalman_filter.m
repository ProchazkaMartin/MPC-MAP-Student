function [mu, sigma] = update_kalman_filter(read_only_vars, public_vars)
%UPDATE_KALMAN_FILTER Summary of this function goes here

mu = public_vars.mu;
sigma = public_vars.sigma;

% I. Prediction
v = sum(public_vars.motion_vector(1:2))/2;
omega = (public_vars.motion_vector(1) - public_vars.motion_vector(2)) / read_only_vars.agent_drive.interwheel_dist;

u = [v, omega];
[mu, sigma] = ekf_predict(mu, sigma, u, public_vars.kf, read_only_vars.sampling_period);

% II. Measurement
z = [read_only_vars.gnss_position];
[mu, sigma] = kf_measure(mu, sigma, z, public_vars.kf);

end

