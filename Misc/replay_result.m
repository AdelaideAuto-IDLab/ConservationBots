function replay_result(varargin)
    p = inputParser;
    addParameter(p, 'N', 1);
    addParameter(p, 'Plot3D', false, @islogical);
    parse(p, varargin{:});

    filename = uigetfile('*.mat');
    
    % read file
    raw_data = load(filename);
    mc = length(raw_data.action_history);   % number of mc runs
    ntarget = raw_data.ntarget;
    
    selected_run = max(min(p.Results.N, mc), 1);
    

    % plotting 
    figure();
    hold on;
    h_target = gobjects(ntarget, 1);
    
    if p.Results.Plot3D         % 3D plot
        h_uav = plot3(nan, nan, nan, 'k', 'LineWidth', 1.5);
        for n = 1:ntarget
            h_target(n) = scatter3(nan, nan, nan, 50, 'Marker', 's', 'LineWidth', 1.5, 'MarkerEdgeColor', 'k'); 
        end
        
        % plot terrain
        DEM = raw_data.config.area_config.DEM.DATA;
        DEM = downsample(DEM, 10);
        DEM = downsample(DEM',10);
        
        surfl((10:10:raw_data.config.area_config.DEM.X_Max), ...
              (10:10:raw_data.config.area_config.DEM.Y_Max), ...
              DEM, 'light'); 
       shading interp;
       alpha 0.8;
       colormap(jet);
       colorbar('Location', 'eastoutside');
       view(45, 45);
        
       ylabel('North (m)');
       xlabel('East (m)');
       zlabel('Elevation (m)');
       
       xlim([raw_data.config.area_config.DEM.X_Min, raw_data.config.area_config.DEM.X_Max]);
       ylim([raw_data.config.area_config.DEM.Y_Min, raw_data.config.area_config.DEM.Y_Max]);
       zlim([raw_data.config.area_config.DEM.Alt_range(1), raw_data.config.area_config.DEM.Alt_range(2) + 100]);
       
       % start animation
       for t = 1:max(raw_data.found_time(selected_run,:))
           % update uav
           set(h_uav, 'XData', raw_data.uav_path{selected_run}(1, 1:t), ...
                      'YData', raw_data.uav_path{selected_run}(2, 1:t), ...
                      'ZData', raw_data.uav_path{selected_run}(3, 1:t) ...
              );
           % update target
           for n = 1:ntarget
               set(h_target(n), 'XData', squeeze(raw_data.truth(1,n,t)), ...
                                'YData', squeeze(raw_data.truth(2,n,t)), ...
                                'ZData', squeeze(raw_data.truth(3,n,t))  ...
                  );
           end
           
           drawnow;
           pause(0.01);
       end
        
    else                        % 2D plot
        h_uav = plot(nan, nan, 'k', 'LineWidth', 1.5);
        for n = 1:ntarget
            h_target(n) = scatter(nan, nan, 50, 'Marker', 's', 'LineWidth', 1.5, 'MarkerEdgeColor', 'k'); 
        end
        
       ylabel('North (m)');
       xlabel('East (m)');
       zlabel('Elevation (m)');
       
       xlim([raw_data.config.area_config.DEM.X_Min, raw_data.config.area_config.DEM.X_Max]);
       ylim([raw_data.config.area_config.DEM.Y_Min, raw_data.config.area_config.DEM.Y_Max]);
       
       % start animation
       for t = 1:max(raw_data.found_time(selected_run,:))
           % update uav
           set(h_uav, 'XData', raw_data.uav_path{selected_run}(1, 1:t), ...
                      'YData', raw_data.uav_path{selected_run}(2, 1:t)  ...
              );
           % update target
           for n = 1:ntarget
               set(h_target(n), 'XData', squeeze(raw_data.truth(1,n,t)), ...
                                'YData', squeeze(raw_data.truth(2,n,t))  ...
                  );
           end
           
           drawnow;
           pause(0.01);
       end
       
       
    end
    
    

end