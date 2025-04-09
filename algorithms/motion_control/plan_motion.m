function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Summary of this function goes here

d = read_only_vars.agent_drive.interwheel_dist;

if public_vars.init_active == 1
    public_vars.motion_vector = [0.13, -0.13];
    return;
end

min_distance = 0.4;
if min(read_only_vars.lidar_distances) < min_distance
    public_vars.path_planning_reqest = 1;
    v = 0.1;
    if read_only_vars.lidar_distances(1) < min_distance
        v = -0.1;
    end

    omega = 0.1;
    if read_only_vars.lidar_distances(2) < read_only_vars.lidar_distances(8)
        omega = -0.1;
    end

    if any(read_only_vars.lidar_distances([2,3]) < min_distance)
        omega = -0.2;
    elseif any(read_only_vars.lidar_distances([7,8]) < min_distance)
        omega = 0.2;
    end

    v_r = v + omega*d/2;
    v_l = v - omega*d/2;
    
    public_vars.motion_vector = [v_r, v_l];
    return
end

if isempty(public_vars.path)
    public_vars.motion_vector = [0, 0];
    return;
end

% slowdowns
speed_mult = 1;
if public_vars.use_pf
    speed_mult = 0.75;
else
    % ekf health
    health = trace(public_vars.sigma);
    delta = max([health-0.5, 0]);
    speed_mult = 1/(1+delta);
end




pose = public_vars.estimated_pose;

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
dP = dP * speed_mult;

v = sum(dP .* [cos(phi), sin(phi)]);
omega = sum(dP .* [-sin(phi), cos(phi)]) / epsilon;

v_r = v + omega*d/2;
v_l = v - omega*d/2;

public_vars.motion_vector = [v_r, v_l];

end