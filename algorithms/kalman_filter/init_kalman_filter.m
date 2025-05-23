function [public_vars] = init_kalman_filter(read_only_vars, public_vars)
%INIT_KALMAN_FILTER Summary of this function goes here


if public_vars.last_use_pf ~= public_vars.use_pf && public_vars.init_active == 0
    public_vars.mu = read_only_vars.est_position_history(end,:);
    public_vars.sigma = eye(3)*0.01;
    return
end

public_vars.kf.C = [1, 0, 0; 0, 1, 0];
public_vars.kf.R = [0.0005, 0, 0; 0, 0.0005, 0; 0, 0, 0.0008];
public_vars.kf.Q = [0.25, 0; 0, 0.25];

position = mean(read_only_vars.gnss_history);
phi = estimate_orientation(position, read_only_vars);

public_vars.mu = [position, phi];
public_vars.sigma = eye(3);
public_vars.sigma(1:2,1:2) = cov(read_only_vars.gnss_history);
public_vars.sigma(3,3) = 0.01;

end

