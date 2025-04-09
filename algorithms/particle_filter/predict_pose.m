function [new_pose] = predict_pose(old_pose, motion_vector, read_only_vars)
%PREDICT_POSE Summary of this function goes here

% motion_vector_noisy = motion_vector .* ( randn(1,2)/10 + 1 );
motion_vector_noisy = motion_vector + ( (randn(1,2)-0.5)/10 );
phi = old_pose(3);

v = mean(motion_vector_noisy)/2;
omega = (motion_vector_noisy(1) - motion_vector_noisy(2)) / read_only_vars.agent_drive.interwheel_dist;
deltapos = [cos(phi)*v, sin(phi)*v, omega] * read_only_vars.sampling_period;
new_pose = old_pose + deltapos ;

new_pose = new_pose + [( (randn(1,2))/40 ), 0];

end

