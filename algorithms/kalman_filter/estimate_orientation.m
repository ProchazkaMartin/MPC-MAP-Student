function [phi] = estimate_orientation(position, read_only_vars)
    
    ld = read_only_vars.lidar_distances;
    ld(isinf(ld)) = 10;
    ld(isnan(ld)) = 10;

    meas_cnt = 90;
    score = inf(meas_cnt,1);
    for i = 1:meas_cnt
        phi_test = i/meas_cnt * 2*pi;
        meas = compute_lidar_measurement(read_only_vars.map, [position, phi_test], read_only_vars.lidar_config);
        meas(isinf(meas)) = 10;
        meas(isnan(meas)) = 10;
        score(i) = norm(meas - ld);
    end

    [~,index] = min(score);
    phi = index / meas_cnt * 2*pi;
end

