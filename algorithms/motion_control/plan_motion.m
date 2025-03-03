function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Summary of this function goes here

% I. Pick navigation target
R = read_only_vars.mocap_pose(1:2);
phi = read_only_vars.mocap_pose(3);
epsilon = 0.5;
P = R + epsilon*[cos(phi), sin(phi)];

% public_vars.position_history = [public_vars.position_history ; R];

[public_vars, target, v_G] = get_target(read_only_vars, public_vars);
delta_GP = target - P;




% II. Compute motion vector

dP = public_vars.v_target/epsilon * delta_GP + v_G;

v = sum(dP .* [cos(phi), sin(phi)]);
omega = sum(dP .* [-sin(phi), cos(phi)]) / epsilon;
d = 0.2;

v_r = v + omega*d/2;
v_l = v - omega*d/2;

% read_only_vars.counter
public_vars.motion_vector = [v_r, v_l];

end