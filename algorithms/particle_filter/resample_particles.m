function [new_particles] = resample_particles(particles, weights, read_only_vars)
%RESAMPLE_PARTICLES Summary of this function goes here


N = read_only_vars.max_particles/4;
N_rng = N/5;
w_index = 1;
w_cumulative = weights(1);
u_cumulative = rand()/N;

new_particles = zeros(N+N_rng, 3);

for i = 1:1:N
    while w_cumulative < u_cumulative
        w_cumulative = w_cumulative + weights(w_index);
        w_index = w_index + 1;
        if w_index > length(weights)
            w_index = length(weights);
        end
    end
    u_cumulative = u_cumulative + 1/N;
    new_particles(i, :) = particles(w_index,:);

end
% mult = read_only_vars.map.limits(3:4) - read_only_vars.map.limits(1:2);
% add = read_only_vars.map.limits(1:2);

x_min = min(read_only_vars.map.gnss_denied(1:2:end));
x_max = max(read_only_vars.map.gnss_denied(1:2:end));

y_min = min(read_only_vars.map.gnss_denied(2:2:end));
y_max = max(read_only_vars.map.gnss_denied(2:2:end));

mult = [x_max - x_min, y_max - y_min];
add = [x_min, y_min];

new_particles(N+1:end,:) = rand(N_rng, 3) .* [mult, 2*pi] + [add,0];
new_particles(:,3) = mod(new_particles(:,3),2*pi);
end

