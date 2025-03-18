function [public_vars] = student_workspace(read_only_vars,public_vars)
%STUDENT_WORKSPACE Summary of this function goes here

% 8. Perform initialization procedure
if (read_only_vars.counter == 1)
          
    public_vars = init_particle_filter(read_only_vars, public_vars);
    % public_vars = init_kalman_filter(read_only_vars, public_vars);
    
    public_vars.path_index = 1;
    public_vars.v_target = 1 *read_only_vars.agent_drive.max_vel;

    mult = read_only_vars.map.limits(3:4) - read_only_vars.map.limits(1:2);
    add = read_only_vars.map.limits(1:2);

    % public_vars.particles = rand(read_only_vars.max_particles, 3) .* [mult, 2*pi] + [add,0];


    % s1_x = [2:0.1:6];
    % s1_y = sin(s1_x*4)+8.5;
    % public_vars.path = [s1_x', s1_y'];
    % 
    % s2_raw = [6,7;4,6; 6,4; 4,2; 6,1; 8,0.5];
    % s2_y = [7:-0.1:0.5];
    % s2_x = spline(s2_raw(:,2),s2_raw(:,1),s2_y);
    % public_vars.path = [public_vars.path; s2_x', s2_y'];
    % 
    % s3 = [9,2; 8,4; 9,6; 9,9];
    % public_vars.path = [public_vars.path; s3];

    public_vars.path = [2,2; 5,8; 10,8; 15,6; 16,2];

end

% 9. Update particle filter
% public_vars.particles = update_particle_filter(read_only_vars, public_vars);

% 10. Update Kalman filter
if read_only_vars.counter == 49
    public_vars = init_kalman_filter(read_only_vars, public_vars);
elseif read_only_vars.counter >= 50
    [public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);
    % 11. Estimate current robot position
    public_vars.estimated_pose = estimate_pose(public_vars); % (x,y,theta)
end

% if read_only_vars.counter < 50
%     public_vars.estimated_pose = nan(1,3);
% end
% public_vars.estimated_pose = read_only_vars.mocap_pose;

% 12. Path planning
public_vars.path = plan_path(read_only_vars, public_vars);

% 13. Plan next motion command
if read_only_vars.counter < 50
    public_vars.motion_vector = [0.2, -0.2];
else
    public_vars = plan_motion(read_only_vars, public_vars);
end


end

