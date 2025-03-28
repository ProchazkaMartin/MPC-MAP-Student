function [public_vars] = init_kalman_filter(read_only_vars, public_vars)
%INIT_KALMAN_FILTER Summary of this function goes here

public_vars.kf.C = [1, 0, 0; 0, 1, 0];
public_vars.kf.R = [0.00005, 0, 0; 0, 0.00005, 0; 0, 0, 0.00005];
public_vars.kf.Q = [0.25, 0; 0, 0.25];

public_vars.mu = [mean(read_only_vars.gnss_history), 0];
public_vars.sigma = ones(3)*10;
public_vars.sigma(1:2,1:2) = cov(read_only_vars.gnss_history);

end

