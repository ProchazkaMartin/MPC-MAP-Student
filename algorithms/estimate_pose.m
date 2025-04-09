function [estimated_pose] = estimate_pose(public_vars)
%ESTIMATE_POSE Summary of this function goes here

    if public_vars.use_pf
        particles = public_vars.particles;
        d = 0.2;
        near_zero = or(particles(:,3) < d, particles(:,3) < 2*pi-d);
        near_pi = and(particles(:,3) < pi+d, particles(:,3) > pi-d);

        if sum(near_zero) > sum(near_pi)
            first_half = particles(:,3) < pi;
            particles(first_half,:) = particles(first_half,:) + [0,0,2*pi];
            
        end

        estimated_pose = median(particles);
    else
        estimated_pose = public_vars.mu;
    end

end

