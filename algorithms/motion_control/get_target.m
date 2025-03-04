function [public_vars,  target, v_G] = get_target(public_vars, P)
%GET_TARGET Summary of this function goes here


wp_distance = vecnorm(public_vars.path(public_vars.path_index:end,:) - P,2,2);
[~, index_offset] = min(wp_distance);

public_vars.path_index = public_vars.path_index + index_offset - 1;

if public_vars.path_index == length(public_vars.path)
    target = public_vars.path(end,:);
    v_G = [0,0];
    return
end

offset_vector = P - public_vars.path(public_vars.path_index,:);

to_next = public_vars.path(public_vars.path_index+1,:) - public_vars.path(public_vars.path_index,:);
to_next_scale = dot(offset_vector,to_next)/dot(to_next,to_next);

if public_vars.path_index == 1
    if to_next_scale < 0
        to_next_scale = 0;
    end
    target = to_next*to_next_scale + public_vars.path(public_vars.path_index,:);
    v_G = to_next/norm(to_next)* public_vars.v_target;
    return
end

to_prev = public_vars.path(public_vars.path_index-1,:) - public_vars.path(public_vars.path_index,:);
to_prev_scale = dot(offset_vector,to_prev)/dot(to_prev,to_prev);


if to_next_scale > 0
    target = to_next*to_next_scale + public_vars.path(public_vars.path_index,:);
    v_G = to_next/norm(to_next)* public_vars.v_target;
    return
end

if to_prev_scale > 0
    target = to_prev*to_prev_scale + public_vars.path(public_vars.path_index,:);
    v_G = -to_prev/norm(to_prev)* public_vars.v_target;
    return
end

target = public_vars.path(public_vars.path_index,:);
v_G = to_next/norm(to_next)* public_vars.v_target;
return

end

