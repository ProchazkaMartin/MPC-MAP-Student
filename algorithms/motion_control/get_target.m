function [public_vars,  target, v_G] = get_target(read_only_vars, public_vars)
%GET_TARGET Summary of this function goes here

wp_current= public_vars.path(public_vars.path_index,:);
wp_next = public_vars.path(public_vars.path_index+1,:);

last_wp_time = (read_only_vars.counter - public_vars.last_path_wp_timestamp)/50
p = last_wp_time * public_vars.v_target / norm(wp_next-wp_current)
if p >= 1
    p = 1;
    if public_vars.path_index+1 < length(public_vars.path)
        public_vars.path_index = public_vars.path_index + 1;
        public_vars.last_path_wp_timestamp = read_only_vars.counter;
    end
end

target = p*wp_next + (1-p)*wp_current;
v_G = (wp_next-wp_current) * public_vars.v_target / norm(wp_next-wp_current);

end

