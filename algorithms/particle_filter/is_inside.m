function [isin] = is_inside(particle, read_only_vars)

polygon_x = read_only_vars.map.gnss_denied(1:2:end);
polygon_y = read_only_vars.map.gnss_denied(2:2:end);

isin = inpolygon(particle(1), particle(2), polygon_x, polygon_y);
end

