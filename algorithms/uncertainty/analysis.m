load("./data.mat")

%% task 2

sigma_lidar = std(public_vars.lidar_history');
sigma_gnss = std(public_vars.gnss_history');

figure(1)
for i = 1:8
    subplot(2,4,i)
    histogram(public_vars.lidar_history(i,:),30)
    title("LiDAR channel "+string(i));
    xlabel("d [m]");
    ylabel("count [-]");
end

figure(2)
for i = 1:2
    subplot(1,2,i)
    histogram(public_vars.gnss_history(i,:),30)
    title("GNSS axis "+string(i));
    xlabel("d [m]");
    ylabel("count [-]");
end

%% task 3

cm_lidar = cov(public_vars.lidar_history');
cm_gnss = cov(public_vars.gnss_history');

%% task 4

mu_lidar = mean(public_vars.lidar_history(1,:));
x_lidar = [mu_lidar-5*sigma_lidar(1):0.01:mu_lidar+5*sigma_lidar(1)];
norm_lidar = norm_pdf(x_lidar, sigma_lidar(1), mu_lidar);

mu_gnss = mean(public_vars.gnss_history(1,:));
x_gnss = [mu_gnss-5*sigma_gnss(1):0.01:mu_gnss+5*sigma_gnss(1)];
norm_gnss = norm_pdf(x_gnss, sigma_gnss(1), mu_gnss);

figure(3)
subplot(1,2,1)
histogram(public_vars.gnss_history(1,:),30,'Normalization','pdf')
title("GNSS axis 1");
xlabel("d [m]");
ylabel("pd [-]");
hold on
plot(x_gnss, norm_gnss, "LineWidth",4)
hold off

subplot(1,2,2)
histogram(public_vars.lidar_history(1,:),30,'Normalization','pdf')
title("LiDAR channel 1");
xlabel("d [m]");
ylabel("pd [-]");
hold on
plot(x_lidar, norm_lidar, "LineWidth",4)
hold off

