function [new_mu, new_sigma] = ekf_predict(mu, sigma, u, kf, sampling_period)
%EKF_PREDICT Summary of this function goes here

phi = mu(3);
v = u(1);
omega = u(2);

new_mu = mu + [v*sampling_period*cos(phi), v*sampling_period*sin(phi), omega*sampling_period];

G = [1, 0, -v*sampling_period*sin(phi); 0, 1, v*sampling_period*cos(phi); 0, 0, 1];

new_sigma = G*sigma*G' + kf.R;

end

