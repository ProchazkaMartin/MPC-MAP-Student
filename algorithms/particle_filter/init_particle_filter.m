function [public_vars] = init_particle_filter(read_only_vars, public_vars)
%INIT_PARTICLE_FILTER Summary of this function goes here

if public_vars.last_use_pf ~= public_vars.use_pf && public_vars.init_active == 0 
    n = read_only_vars.max_particles/2;
    public_vars.particles = ones(n, 3) .* read_only_vars.est_position_history(end,:);
    return
end


x_min = min(read_only_vars.map.gnss_denied(1:2:end));
x_max = max(read_only_vars.map.gnss_denied(1:2:end));

y_min = min(read_only_vars.map.gnss_denied(2:2:end));
y_max = max(read_only_vars.map.gnss_denied(2:2:end));

mult = [x_max - x_min, y_max - y_min];
add = [x_min, y_min];

public_vars.particles = rand(read_only_vars.max_particles, 3) .* [mult, 2*pi] + [add,0];


end

