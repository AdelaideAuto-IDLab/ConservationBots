function path_figure()
    
    % select filename
%     filename = uigetfile('*.mat');
filename = '100-Combine-20targets-Renyi_reward-Void_true-Term_det-20000-ENV_Moderate-20T--05-12-15_04';
n = 1;
    
    if ~isempty(filename)
        raw = load(filename);
    else
       return; 
    end
    
    ntarget = raw.ntarget;
    
    if n > length(raw.action_history)
        error('Selected run not exist');
    end
    
    %% extract truth and estimate when localized
    truth = nan(3, ntarget);
    est = nan(3, ntarget);
    for i = 1:ntarget
        truth(:, i) = squeeze(raw.truth(:, i, raw.found_time(n, i)));
        est(:, i) = raw.est_history{n}{i}(:, raw.found_time(n, i));
    end
    
    % convert to GPS coordinate
    gps_base = [raw.config.area_config.DEM.ref.lat; raw.config.area_config.DEM.ref.lon];
    truth_GPS = GPS_Coord.Cart2GPS(gps_base, truth(1:2, :));
    est_GPS = GPS_Coord.Cart2GPS(gps_base, est(1:2, :));
    uav_gps = GPS_Coord.Cart2GPS(gps_base, raw.uav_path{n}(1:2,:));
    
    %% plot with DEM
    % down sample DEM to speed up plotting
    DEM = raw.config.area_config.DEM.DATA;
    DEM = downsample(DEM, 10);
    DEM = downsample(DEM', 10);
    
    figure('Position', [90,300,1600, 600], 'DefaultLegendFontSize',13,'DefaultLegendFontSizeMode','manual');
    subplot(1,2,1);
    hold on;
    surfl([10:10:2000], [10:10:2000], DEM, 'light'); shading interp;
    alpha 0.8;
    colormap(jet);
    colorbar('Location','eastoutside');
    
    xlabel('East (m)');
    ylabel('North (m)');
    zlabel('Elevation (m)');
%     % plot target location
    ha = scatter3(truth(1,:), truth(2,:), truth(3,:), 50, '*', 'k', 'LineWidth', 2);
%     hb = scatter3(est(1,:), est(2,:), est(3,:), 30, 's', 'MarkerEdgeColor', 0.651*ones(1,3), 'LineWidth', 3);
%     

    % add target number
    for t = 1:ntarget
        text(truth(1,t), truth(2,t), truth(3,t)+ 20, [' ', num2str(t)], 'FontSize', 15, 'Color', 'w');
    end
%     
%     caxis([236, 240]);
%     zlim([236, 240]);
    legend([ha], 'Ground Truth');
    view(45, 45);
    grid on;
    ax = gca;
    ax.FontSize = 16;
    
    %% plot 2D plot with UAV path
    subplot(1,2,2);
    hold on;
    h1 = plot(uav_gps(2,:), uav_gps(1,:), 'HandleVisibility','on', 'LineWidth', 3);
    h2 = scatter(truth_GPS(2,:), truth_GPS(1,:), 42, '*', 'r', 'LineWidth', 3);  % truth
    h3 = scatter(est_GPS(2,:), est_GPS(1,:), 42, 's', 'MarkerEdgeColor', 0.851*ones(1,3), 'LineWidth', 3);      % est 
    for t = 1:ntarget
        text(truth_GPS(2,t), truth_GPS(1,t), [' ', num2str(t)], 'FontSize', 15, 'Color', 'w');
    end
    plot_google_map('MapScale', 1, 'Maptype', 'hybrid');
    grid on;
    xlabel('Longitude');
    ylabel('Latitude');
    legend([h1, h2, h3], 'UAV Trajectory', 'Ground Truth', 'Estimated');
    
    ax = gca;
    ax.FontSize = 16;

end