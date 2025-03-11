function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Summary of this function goes here

pose = read_only_vars.mocap_pose;
% pose = public_vars.estimated_pose;

% I. Pick navigation target
R = pose(1:2);
phi = pose(3);
epsilon = 0.2;
P = R + epsilon*[cos(phi), sin(phi)];

[public_vars, target, v_G] = get_target(public_vars, P);
delta_GP = target - P;

% II. Compute motion vector

dP = public_vars.v_target/epsilon * delta_GP + v_G;

if norm(dP) > read_only_vars.agent_drive.max_vel
    dP = dP/norm(dP)*read_only_vars.agent_drive.max_vel;
end
dP = dP / 2;%todo comment

v = sum(dP .* [cos(phi), sin(phi)]);
omega = sum(dP .* [-sin(phi), cos(phi)]) / epsilon;
d = read_only_vars.agent_drive.interwheel_dist;

v_r = v + omega*d/2;
v_l = v - omega*d/2;

public_vars.motion_vector = [v_r, v_l];
% public_vars.motion_vector = [0.01, 0];

end