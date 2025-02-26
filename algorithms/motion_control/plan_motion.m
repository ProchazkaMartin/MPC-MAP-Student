function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Summary of this function goes here

% I. Pick navigation target

target = get_target(public_vars.estimated_pose, public_vars.path);


% II. Compute motion vector

if read_only_vars.counter < 140
    public_vars.motion_vector = [0.49, 0.5];
elseif read_only_vars.counter < 190
    public_vars.motion_vector = [0.4, 0.5];
elseif read_only_vars.counter < 300
    public_vars.motion_vector = [0.51, 0.5];
elseif read_only_vars.counter < 345
    public_vars.motion_vector = [0.5, 0.4];
else
    public_vars.motion_vector = [0.5, 0.5];
end

read_only_vars.counter

end