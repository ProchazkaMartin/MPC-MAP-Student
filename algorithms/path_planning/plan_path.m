function [path, public_vars] = plan_path(read_only_vars, public_vars)
%PLAN_PATH Summary of this function goes here

path = backtrack_from_precomputed(read_only_vars, public_vars);
path = smooth_path(path);

% planning_required = 0;
% if planning_required
% 
%     path = astar(read_only_vars, public_vars);
% 
%     path = smooth_path(path);
% 
% else
% 
%     path = public_vars.path;
% 
% end

end

