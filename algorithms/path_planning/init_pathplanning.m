function [public_vars] = init_pathplanning(read_only_vars, public_vars)


    map_visited = false(size(read_only_vars.discrete_map.map));
    map_distance = inf(size(read_only_vars.discrete_map.map));
    public_vars.map_direction = zeros(size(read_only_vars.discrete_map.map));

    public_vars.directions = [1,0; 1,1; 0,1; -1,1; -1,0; -1,-1; 0,-1; 1,-1];
    costs = [1, 1.44, 1, 1.44, 1, 1.44, 1, 1.44];
    
    row_limit = read_only_vars.discrete_map.dims(2);
    col_limit = read_only_vars.discrete_map.dims(1);

    goal_r = read_only_vars.discrete_map.goal(2);
    goal_c = read_only_vars.discrete_map.goal(1);

    map_distance(goal_r, goal_c) = 0;
    
    map_expanded = zeros(size(read_only_vars.discrete_map.map));
    kernel = [0, 0, 1, 1, 1, 0, 0;
              0, 1, 1, 1, 1, 1, 0;
              1, 1, 1, 1, 1, 1, 1;
              1, 1, 1, 1, 1, 1, 1;
              1, 1, 1, 1, 1, 1, 1;
              0, 1, 1, 1, 1, 1, 0;
              0, 0, 1, 1, 1, 0, 0;];
    kernel_offset = 3;

    for r_map = 1:row_limit
        for c_map = 1:col_limit
            patch = zeros(size(kernel));
            for r_offset = -kernel_offset:kernel_offset
                for c_offset = -kernel_offset:kernel_offset

                    new_row = r_map + r_offset;
                    if new_row > row_limit || new_row < 1
                        continue;
                    end
                    new_col = c_map + c_offset;
                    if new_col > col_limit || new_col < 1
                        continue;
                    end
                    r_patch = r_offset+kernel_offset+1;
                    c_patch = c_offset+kernel_offset+1;
                    patch(r_patch, c_patch) = read_only_vars.discrete_map.map(new_row, new_col) * kernel(r_patch, c_patch);
                end
            end
            map_expanded(r_map, c_map) = any(patch, 'all');
        end
    end
    % map_expanded = read_only_vars.discrete_map.map;

    while true 
        map_distance_copy = map_distance;
        map_distance_copy(map_visited) = inf;
        [min_distance, min_index] = min(map_distance_copy, [], 'all');
        if min_distance == inf
            return;
        end
        [row,col] = ind2sub(size(map_distance), min_index);

        map_visited(row, col) = true;

        for i = 1:8
            new_row = row + public_vars.directions(i, 1);
            if new_row > row_limit || new_row < 1
                continue;
            end
            new_col = col + public_vars.directions(i, 2);
            if new_col > col_limit || new_col < 1
                continue;
            end

            new_distance = min_distance + costs(i);
            if map_expanded(new_row, new_col) == 1
                new_distance = new_distance + 1e8;
            end
            if new_distance < map_distance(new_row, new_col)
                map_distance(new_row, new_col) = new_distance;
                public_vars.map_direction(new_row, new_col) = i;
            end

            if map_visited(new_row, new_col)
                continue;
            end
        end

    end

end

