figure('Visible', 'off');
hold on;
x0=10;
y0=10;
width=1200;
height=400;
set(gcf,'position',[x0,y0,width,height])
xlim([0 35]);
for n = 6:15
    if n == 9 
        continue; 
    end
    filename = sprintf('csv/imu_raw_trip%d_1_trim.csv', n);
    data = readmatrix(filename, 'Delimiter', ',');
    data = data(2:end, :);
    time = data(:, 1);
    uniform_time = linspace(min(time), max(time), numel(time)); % resample
    data_columns = data(:, 2:4);
    uniform_data = interp1(time, data_columns(:, 1), uniform_time, 'linear');
    dt = mean(diff(uniform_time));
    b = fdesign.lowpass('N,F3dB', 4, 1, 1 / dt);
    d1 = design(b, 'butter');
    filteredData = filtfilt(d1.sosMatrix, d1.ScaleValues, uniform_data);
    jerk = diff(filteredData) / dt;
    plot(uniform_time(2:end), jerk);
end
hold off;
xlabel('Time');
ylabel('Jerk');
legend('jerk = 4', 'jerk = 2.13', 'jerk = 0.25')
title('Jerk with Jerk Limit Change');
set(findall(gcf,'-property','FontSize'),'FontSize',24)
saveas(gcf,'plots/filtered_jerk_change_resized.png');

figure('Visible', 'off');
hold on;
for n = 6:15
    if n == 9 
        continue; 
    end
    filename = sprintf('csv/imu_raw_trip%d_1_trim.csv', n);
    data = readmatrix(filename, 'Delimiter', ',');
    data = data(2:end, :);
    time = data(:, 1);
    uniform_time = linspace(min(time), max(time), numel(time)); % resample
    data_columns = data(:, 2:4);
    uniform_data = interp1(time, data_columns(:, 1), uniform_time, 'linear');
    dt = mean(diff(uniform_time));
    b = fdesign.lowpass('N,F3dB', 4, 2, 1 / dt);
    d1 = design(b, 'butter');
    filteredData = filtfilt(d1.sosMatrix, d1.ScaleValues, uniform_data);
    plot(time, filteredData);
end
hold off;
xlabel('Time');
ylabel('Acceleration X');
legend('Jerk = 4', 'Jerk = 2.13', 'Jerk = 0.25')
title('Acceleration with Acceleration Limit Change');
saveas(gcf,'plots/filtered_acc_change.png');
