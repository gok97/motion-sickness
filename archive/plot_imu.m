files = dir('csv/imu_*_trim.csv');
for i = 1:numel(files)
    disp(files(i).name);
    data = readmatrix(['csv/' files(i).name], 'Delimiter', ',');
    data = data(2:end, :);
    time = data(:, 1);
    uniform_time = linspace(min(time), max(time), numel(time)); % resample
    data_columns = data(:, 2:4);

    figure('Visible', 'off');
    plot(time, data_columns);
    xlabel('Time');
    ylabel('Acceleration');
    legend('LinearAccX', 'LinearAccY', 'LinearAccZ')
    title('Raw Acceleration vs. Time');
    [~, name, ~] = fileparts(files(i).name);
    name = name(18:end); % TEMP SOLUTION
    saveas(gcf, ['plots/acceleration/raw_', name, '.png']);

    uniform_data = interp1(time, data_columns(:, 1), uniform_time, 'linear');
    dt = mean(diff(uniform_time));
    b = fdesign.lowpass('N,F3dB', 4, 2, 1 / dt);
    d1 = design(b, 'butter');
    filteredData = filtfilt(d1.sosMatrix, d1.ScaleValues, uniform_data);

    figure('Visible', 'off');
    plot(time, data_columns(:, 1));
    hold on;
    plot(uniform_time, filteredData);
    hold off;
    xlabel('Time');
    ylabel('Acceleration X');
    legend('Raw', 'Filtered')
    title('IMU Acceleration X vs. Time');
    saveas(gcf, ['plots/filtered/filtered_', name, '.png']);

    jerk = diff(filteredData) / dt;
    figure('Visible', 'off');
    plot(uniform_time, filteredData);
    hold on;
    plot(uniform_time(2:end), jerk);
    hold off;
    xlabel('Time');
    ylabel('Jerk');
    legend('Acceleration', 'Jerk')
    title('IMU Jerk X vs. Time');
    saveas(gcf, ['plots/jerk/jerk_', name, '.png']);
end

