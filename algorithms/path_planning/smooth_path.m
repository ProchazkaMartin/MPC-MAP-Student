function [new_path] = smooth_path(old_path)
%SMOOTH_PATH Summary of this function goes here

new_path = old_path;

a = 0.1;
b = 0.3;

while true
    delta_sum = [0, 0];
    for i = 2:length(old_path)-1
        d = old_path(i,:) - new_path(i,:);
        s = new_path(i-1,:) + new_path(i+1,:) -2*new_path(i,:);

        delta = a*d + b*s;
        new_path(i,:) = new_path(i,:) + delta;
        delta_sum = delta_sum + abs(delta);
    end
    if norm(delta_sum) < 0.1
        return;
    end
end

end

