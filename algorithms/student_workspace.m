function [public_vars] = student_workspace(read_only_vars,public_vars)
%STUDENT_WORKSPACE Summary of this function goes here

public_vars.use_pf = any(isnan(read_only_vars.gnss_position));

% 8. Perform initialization procedure
if (read_only_vars.counter == 1)
    public_vars.last_use_pf = false;
    public_vars.init_active = 1;

    % graph_voronoi(read_only_vars, public_vars)
    public_vars = init_pathplanning(read_only_vars, public_vars);
    if public_vars.use_pf
        public_vars = init_particle_filter(read_only_vars, public_vars);
    end
    
    public_vars.in_goal_counter = 0;
    public_vars.path_index = 1;
    public_vars.path_planning_reqest = 0;
    public_vars.v_target = 1 *read_only_vars.agent_drive.max_vel;
end


% handle switch
if public_vars.last_use_pf ~= public_vars.use_pf && public_vars.init_active == 0
    if public_vars.use_pf
        public_vars = init_particle_filter(read_only_vars, public_vars);
    else 
        public_vars = init_kalman_filter(read_only_vars, public_vars);
    end
end


% 9. Update particle filter
public_vars.particles = update_particle_filter(read_only_vars, public_vars);

% 10. Update Kalman filter
if read_only_vars.counter == 49
    public_vars = init_kalman_filter(read_only_vars, public_vars);
    public_vars.init_active = 0;
    public_vars.path_planning_reqest = 1;

end


if public_vars.init_active == 0
    if public_vars.use_pf
        public_vars.particles = update_particle_filter(read_only_vars, public_vars);
    else
        [public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);
    end
    % 11. Estimate current robot position
    public_vars.estimated_pose = estimate_pose(public_vars); % (x,y,theta)

    % check for glitch
    sample_diff = 10;
    if length(read_only_vars.est_position_history) > sample_diff
        delta_pos = public_vars.estimated_pose(1:2) - read_only_vars.est_position_history(end-sample_diff,1:2);
        if norm(delta_pos)/read_only_vars.sampling_period/sample_diff > 2*read_only_vars.agent_drive.max_vel
            public_vars.path_planning_reqest = 1;
        end
    end

    % check for invalid path
    if size(public_vars.path,1) == 1
        public_vars.path_planning_reqest = 1;
    end

    % resolve symmetry
    delta_pos_goal = public_vars.estimated_pose(1,2) - read_only_vars.map.goal;
    if norm(delta_pos_goal) < read_only_vars.map.goal_tolerance*0.8
        public_vars.in_goal_counter = public_vars.in_goal_counter +1;
    else
        public_vars.in_goal_counter = 0;
    end

    if public_vars.use_pf && public_vars.in_goal_counter > 10
        public_vars = init_particle_filter(read_only_vars, public_vars);
        public_vars.in_goal_counter = 0;
    end
end

% 12. Path planning
public_vars = plan_path(read_only_vars, public_vars);

% 13. Plan next motion command
public_vars = plan_motion(read_only_vars, public_vars);

public_vars.last_use_pf = public_vars.use_pf;
end

