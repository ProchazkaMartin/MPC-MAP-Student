function [map_distance] = distance_transform(map)
    map_visited = false(size(map));
    map_distance = inf(size(map));
    
    directions = [1,0; 1,1; 0,1; -1,1; -1,0; -1,-1; 0,-1; 1,-1];
    [row_limit, col_limit] = size(map);

    map_distance(map == 1) = 0;

    while true 
        map_distance_copy = map_distance;
        map_distance_copy(map_visited) = inf;
        [min_distance, min_index] = min(map_distance_copy, [], 'all');
        if min_distance == inf
            break;
        end
        [row,col] = ind2sub(size(map_distance), min_index);

        map_visited(row, col) = true;

        for i = 1:8
            new_row = row + directions(i, 1);
            if new_row > row_limit || new_row < 1
                continue;
            end
            new_col = col + directions(i, 2);
            if new_col > col_limit || new_col < 1
                continue;
            end

            if map(new_row, new_col) == 1
                map_visited(new_row, new_col) = true;
                continue;
            end

            new_distance = min_distance + 1;
            if new_distance < map_distance(new_row, new_col)
                map_distance(new_row, new_col) = new_distance;
            end

            if map_visited(new_row, new_col)
                continue;
            end
        end
    end
end

