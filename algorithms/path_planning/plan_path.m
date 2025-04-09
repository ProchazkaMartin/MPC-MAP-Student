function [public_vars] = plan_path(read_only_vars, public_vars)
%PLAN_PATH Summary of this function goes here

    if public_vars.path_planning_reqest == 1
        public_vars.path_planning_reqest = 0;
    
        public_vars.path = backtrack_from_precomputed(read_only_vars, public_vars);
        public_vars.path = smooth_path(public_vars.path);
        public_vars.path = [public_vars.path; read_only_vars.map.goal];
        public_vars.path_index = 1;
    end


end

