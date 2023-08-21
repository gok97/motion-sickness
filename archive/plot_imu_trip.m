for n = 6:8
    if n == 9 
        continue; 
    end
    files = dir(sprintf('csv/imu_raw_trip%d_*_trim.csv', n));
    figure('Visible', 'off');
    x0=10;
    y0=10;
    width=600;
    height=800;
    set(gcf,'position',[x0,y0,width,height])
    ylim([-3 4]);
    xlim([-6 inf]);
    hold on;
    grid on;
    for i = 1:numel(files)
        data = readmatrix(['csv/' files(i).name], 'Delimiter', ',');
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
    title(sprintf('Trip %d', n));
    set(findall(gcf,'-property','FontSize'),'FontSize',24)
    saveas(gcf, sprintf('plots/trips/acc_trip%d.png', n));
end

%%
for n = 6:8
    if n == 9 
        continue; 
    end
    files = dir(sprintf('csv/imu_raw_trip%d_*_trim.csv', n));
    figure('Visible', 'off');
    x0=10;
    y0=10;
    width=600;
    height=800;
    set(gcf,'position',[x0,y0,width,height])
    ylim([-6 4]);
    xlim([-2 inf]);
    hold on;
    grid on;
    for i = 1:numel(files)
        data = readmatrix(['csv/' files(i).name], 'Delimiter', ',');
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
    ylabel('Jerk X');
    title(sprintf('Trip %d', n));
    set(findall(gcf,'-property','FontSize'),'FontSize',24)
    saveas(gcf, sprintf('plots/trips/jerk_trip%d.png', n));
end