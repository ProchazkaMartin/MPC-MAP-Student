function [offset] = offset_rotation(measurement, lidar_distances)
    score = zeros(size(lidar_distances));
    for i = 1:length(lidar_distances)
        shifted = circshift(measurement,i-1);
        score(i) = norm(lidar_distances - shifted);
    end
    [~, index] = min(score);
    offset = index - 1;
end

