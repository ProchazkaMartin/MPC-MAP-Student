function [path] = backtrack_from_precomputed(read_only_vars, public_vars)
    map_width = read_only_vars.discrete_map.limits(3)- read_only_vars.discrete_map.limits(1);
    map_height = read_only_vars.discrete_map.limits(4)- read_only_vars.discrete_map.limits(2);
    
    map_offset_x = read_only_vars.discrete_map.limits(1);
    map_offset_y = read_only_vars.discrete_map.limits(2);

    raw_c = (public_vars.estimated_pose(1)-map_offset_x) / map_width * read_only_vars.discrete_map.dims(1);
    raw_r = (public_vars.estimated_pose(2)-map_offset_y) / map_height* read_only_vars.discrete_map.dims(2);
    
    path = [];
    row = round(raw_r);
    col = round(raw_c);

    while true
        x = (col-1) /read_only_vars.discrete_map.dims(1) * map_width + map_offset_x;
        y = (row-1) /read_only_vars.discrete_map.dims(2) * map_height + map_offset_y;

        path = [path; x, y];
        
        direction_index = public_vars.map_direction(row, col);
        if direction_index == 0
            return;
        end

        row = row - public_vars.directions(direction_index, 1);
        col = col - public_vars.directions(direction_index, 2);

    end

end
