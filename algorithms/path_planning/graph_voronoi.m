function [] = graph_voronoi(read_only_vars, public_vars)
    map_distance = distance_transform(read_only_vars.discrete_map.map);
    [row_limit, col_limit] = size(read_only_vars.discrete_map.map);
    skeleton = zeros(size(read_only_vars.discrete_map.map));
    
    for r = 2:row_limit-1
        for c = 2:col_limit-1
            if read_only_vars.discrete_map.map(r, c) == 1
                continue;
            end
            neighborhood = map_distance(r-1:r+1, c-1:c+1);
            if map_distance(r, c) == max(neighborhood, [], 'all')
                skeleton(r, c) = 1;
            end
        end
    end

    figure;
    imagesc(map_distance);      % Display the distance transform
    colormap(jet);         % Use a colorful colormap
    colorbar;              % Add a color scale
    axis equal;            % Keep aspect ratio
    hold on;
    
    % Overlay Skeleton in Black
    [rowS, colS] = find(skeleton == 1);  % Get skeleton pixel coordinates
    plot(colS, rowS, 'k.', 'MarkerSize', 10); % Plot skeleton points in black
    
    title('Distance Transform with Skeleton Overlay');
    hold off;

end

