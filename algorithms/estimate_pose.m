function [estimated_pose] = estimate_pose(public_vars)
%ESTIMATE_POSE Summary of this function goes here

% estimated_pose = nan(1,3);
% estimated_pose = median(public_vars.particles);
estimated_pose = public_vars.mu;

end

