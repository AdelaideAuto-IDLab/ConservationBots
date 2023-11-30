classdef Result_plot_gps < handle
    properties     
        fig_handle;
        app;                    
        config;
        particle_handle;        % particles handle
        uav_path_handle;        % uav path handle
        uav_path_return_handle; % uav return path handle
        target_handle;          % target location handle
        target_path_handle;     % target path handle
        est_handle;             % estimated location handle
        void_handle;            % void radius handle
        boundary_handle;        % area boundary handle
        
        UAV_shape_handle;
        
        target_text_handle;
        
        fig_rmse;               % RMSE figure handle
        ax_rmse;
        error_xy_handle;
        
        fig_termination_val;    % termination progress
        ax_termination;
        termination_handle;     % termination plot handle
        termination_th_handle;  % termination threshold handle
        
    end
    
    properties (Access = private)
        % available color list
        color = [0.000000000000000,  0.450980392156863,  0.741176470588235;...
                 0.850980392156863   0.329411764705882   0.101960784313725;...
                 0.929411764705882   0.690196078431373   0.129411764705882;...
                 0.490196078431373   0.180392156862745   0.560784313725490;...
                 0.470588235294118   0.670588235294118   0.188235294117647;...
                 0.301960784313725   0.749019607843137   0.929411764705882;...
                 0.639215686274510   0.078431372549020   0.180392156862745;...
                 1.000000000000000   1.000000000000000   0.066666666666667;...
                 0.074509803921569   0.623529411764706   1.000000000000000;...
                 1.000000000000000   0.411764705882353   0.160784313725490;...
                 0.392156862745098   0.831372549019608   0.074509803921569;...
                 0.717647058823529   0.274509803921569   1.000000000000000;...
                 0.058823529411765   1.000000000000000   1.000000000000000;...
                 1.000000000000000   0.074509803921569   0.650980392156863;...
                 1                   0                   0                ;...
                 1                   1                   0                ;...
                 1                   0                   1                ;...
                 0                   1                   1                ;...
                 0                   1                   0                ;...
                 0                   0                   1                ]; 
         
         active_particle_size = 1;
         marker_size = 50;
         linewidth = 1.5;
         text_size = 13;
         
         ID_list = []
         
         void_xp;
         void_yp;
    end

    
    
    methods 
        function obj = Result_plot_gps(max_target, varargin)
            p = inputParser;
            addParameter(p, 'Plot3D', false, @islogical);
            addParameter(p, 'DEM', []);    % DEM matrix
            addParameter(p, 'Plot_Void', false, @islogical);
            addParameter(p, 'Void_r', 0);  % void radius
            addParameter(p, 'Area', []);    % are boundary
            addParameter(p, 'PlotTermination', true, @islogical);
            addParameter(p, 'Term_th', 2e4);    % termination threshold
            addParameter(p, 'Keyboard_Control', false, @islogical); % allow control through
            addParameter(p, 'ID_list', []); % target id list
            addParameter(p, 'gps_ref', []);
            addParameter(p, 'SeperatePlot', false, @islogical);
            addParameter(p, 'ShowSatellite', false, @islogical);
            
            parse(p, varargin{:});
            
            obj.ID_list = p.Results.ID_list;
            obj.config.gps_ref = p.Results.gps_ref;
            obj.config.SeperatePlot = p.Results.SeperatePlot;
            obj.config.show_satellite_map = p.Results.ShowSatellite;
            
            % normal figure
            obj.fig_handle = figure('Position', [100, 100, 1280, 720]);
            if ~obj.config.SeperatePlot
                tiledlayout(4, 7);
                obj.app.UIAxes_Main = nexttile(1, [4,4]);
                obj.app.UIAxes_RMSE = nexttile(5, [2,3]);
                obj.app.UIAxes_Term = nexttile(19, [2,3]);
                hold(obj.app.UIAxes_RMSE);
            else
               obj.app.UIAxes_Main = axes(obj.fig_handle); 
               obj.fig_rmse = figure();
               obj.ax_rmse = axes(obj.fig_rmse);
               hold(obj.ax_rmse);
                set(obj.ax_rmse,'fontname','Arial', 'Fontsize', 14)
            end
            
            % setup figure
            hold(obj.app.UIAxes_Main);  
            
            obj.config.Plot3D = p.Results.Plot3D;
            obj.config.DEM = p.Results.DEM;
            obj.config.Plot_Void = p.Results.Plot_Void;
            obj.config.Void_r = p.Results.Void_r;
            obj.config.max_target = max_target;
            obj.config.PlotTermination = p.Results.PlotTermination;
            obj.config.Term_th = p.Results.Term_th;
            gps_area = GPS_Coord.Cart2GPS(obj.config.gps_ref, p.Results.Area);
            obj.config.area = gps_area;
            

            % enable keyboard control if applicable
            if (p.Results.Keyboard_Control)
                set(obj.fig_handle, 'WindowKeyPressFcn', @keyboard_control_handler);
            end
            
            if ~obj.config.Plot3D
                % termination plot
                if obj.config.PlotTermination && ~obj.config.SeperatePlot
                    hold(obj.app.UIAxes_Term);
                    grid(obj.app.UIAxes_Term, 'on');
                    set(obj.app.UIAxes_Term, 'YScale', 'log');
                    obj.termination_handle = gobjects(obj.config.max_target, 1);
                    for i = 1:obj.config.max_target
                        obj.termination_handle(i) = plot(nan, nan, 'LineWidth', 1.5, 'Color', obj.color(i, :),'Parent', obj.app.UIAxes_Term);
                    end
                    obj.termination_th_handle = plot([nan, nan], [obj.config.Term_th, obj.config.Term_th], 'LineWidth', 2, 'Color', 'r', 'Parent', obj.app.UIAxes_Term);
                    xlabel(obj.app.UIAxes_Term, 'Time (s)');
                    ylabel(obj.app.UIAxes_Term, 'Termination Threshold');
                    title(obj.app.UIAxes_Term, 'Termination vs time');
                end
                
                obj.UAV_shape_handle = UAV_shape(obj.app.UIAxes_Main, 'k', 10*360/(2*pi*6371000));
                obj.particle_handle = gobjects(obj.config.max_target, 1);
                obj.target_handle = gobjects(obj.config.max_target, 1);
                obj.est_handle = gobjects(obj.config.max_target, 1);
                obj.target_text_handle = gobjects(obj.config.max_target, 1);
                obj.uav_path_handle = plot(nan, nan, 'k', 'LineWidth', obj.linewidth, 'Parent', obj.app.UIAxes_Main);
                obj.uav_path_return_handle = line(nan, nan, 'Color', 'k', 'LineStyle', '--', 'LineWidth', obj.linewidth, 'Parent', obj.app.UIAxes_Main);

                % plot boundary
                obj.boundary_handle = plot(polyshape(flipud(obj.config.area)'),  'EdgeColor', 'k', 'LineWidth', 2, 'FaceAlpha', 0.2, 'Parent', obj.app.UIAxes_Main);
                
                
                if obj.config.Plot_Void
                    obj.void_handle = plot(nan,nan, 'g', 'LineStyle', '--', 'Parent', obj.app.UIAxes_Main);
                    ang=0:0.01:2*pi;
                    obj.void_xp = obj.config.Void_r * cos(ang) * 360/(2*pi*6371000);
                    obj.void_yp = obj.config.Void_r * sin(ang) * 360/(2*pi*6371000);
                end

                % initialize plot
                for i = 1:obj.config.max_target
                    obj.particle_handle(i) = scatter(nan, nan, obj.active_particle_size, obj.color(i, :), 'Marker', '*', 'Parent', obj.app.UIAxes_Main);
                    obj.target_handle(i) = scatter(nan, nan, obj.marker_size, 'Marker', 's', 'LineWidth', obj.linewidth, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', obj.color(i, :), 'Parent', obj.app.UIAxes_Main);
                    obj.target_path_handle(i) = plot(nan, nan, 'LineStyle', '-', 'Color', obj.color(i, :), 'LineWidth', obj.linewidth, 'Parent', obj.app.UIAxes_Main); 
                    obj.est_handle(i) = scatter(nan, nan, obj.marker_size , 'Marker', '^', 'LineWidth', obj.linewidth, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', obj.color(i, :), 'Parent', obj.app.UIAxes_Main);
                    if isempty(obj.ID_list)
                        obj.target_text_handle(i) = text(nan, nan, ['\leftarrow Tag ', num2str(i)], 'FontSize', obj.text_size, 'FontWeight', 'bold', 'Parent', obj.app.UIAxes_Main);
                    else
                        obj.target_text_handle(i) = text(nan, nan, ['\leftarrow Tag ', num2str(obj.ID_list(i))], 'FontSize', obj.text_size, 'FontWeight', 'bold', 'Parent', obj.app.UIAxes_Main);
                    end
                end
                
                % main figure
                axis(obj.app.UIAxes_Main, 'equal');
                xlim(obj.app.UIAxes_Main, [min(obj.config.area(2,:))-0.001, max(obj.config.area(2,:))+0.001]);
                ylim(obj.app.UIAxes_Main, [min(obj.config.area(1,:))-0.001, max(obj.config.area(1,:))+0.001]);
                xlabel('Longitude');
                ylabel('Latitude');
                set(obj.app.UIAxes_Main,'fontname','Arial', 'Fontsize', 14)
                
                if ~obj.config.SeperatePlot
                    selected_axes = obj.app.UIAxes_RMSE;
                else
                    selected_axes = obj.ax_rmse;
                end
                    
                % RMSE figure
                obj.error_xy_handle = gobjects(obj.config.max_target, 1);
                for i = 1:obj.config.max_target
                    obj.error_xy_handle(i) = plot(nan, nan, 'LineStyle', '-', 'Color', obj.color(i, :), 'LineWidth', obj.linewidth, 'Parent', selected_axes); 
                end
                title(selected_axes, 'RMSE'); xlabel(selected_axes, 'Time (s)'); ylabel(selected_axes, 'RMSE (m)'); grid(selected_axes, 'on');
                

                if obj.config.show_satellite_map
                    plot_google_map('Axis', obj.app.UIAxes_Main, 'Maptype', 'satellite');
                end
            else
                % TODO: implement 3D plotting
                error('Plot in 3D not implemented.\n'); 
            end
        end
        
        % update everything
        function update_plot(obj, density, uav, target_truth, est, selected_target, current_speed)
            obj.update_density(density);
            obj.update_est(est);
            obj.update_uav(uav);
            obj.update_target(target_truth);
            obj.update_text(target_truth);
            
            if obj.config.Plot_Void
                obj.update_void(uav);
            end
            
            % update title & speed
            if nargin == 6
                title(obj.app.UIAxes_Main, sprintf('Selected target: %i', selected_target));
            elseif nargin == 7
                if ~isfloat(current_speed)
                    current_speed = nan;
                end
                title(obj.app.UIAxes_Main, sprintf('Selected target: %i, Speed: %.1f', selected_target, current_speed));
            end
                       
            pause(0.1);
            drawnow;
        end
        
        function update_error_plot(obj, rmse_error, t)
            if ~obj.config.Plot3D 
                for n = 1:length(rmse_error)
                    set(obj.error_xy_handle(n), 'XData', (1:t), 'YData', sqrt(rmse_error{n}(1, 1:t).^2 + rmse_error{n}(2, 1:t).^2));
                end
            else
                error('Plot in 3D not implemented. \n');
            end
        end
        
        
        
        % update particles
        function update_density(obj, density)
             if ~obj.config.Plot3D
                 for n = 1:length(density)
                    gps_point = GPS_Coord.Cart2GPS(obj.config.gps_ref, density{n}.particles(1:2, :));
                    set(obj.particle_handle(n), 'XData', gps_point(2,:),...
                                                'YData', gps_point(1,:));
                 end
             else
                 error('Plot in 3D not implemented.\n'); 
             end
        end
        
        % update estimated location
        function update_est(obj, est)
             if ~obj.config.Plot3D
                 gps_point = GPS_Coord.Cart2GPS(obj.config.gps_ref, est(1:2, :));
                 for n = 1:size(est, 2)
                    set(obj.est_handle(n), 'XData', gps_point(2, n), 'YData', gps_point(1,n));
                 end
             else
                 error('Plot in 3D not implemented.\n'); 
             end
        end
        
        % update uav location & void circle
        function update_uav(obj, uav)
            if ~obj.config.Plot3D
                gps_point = GPS_Coord.Cart2GPS(obj.config.gps_ref, uav(1:2, :));
                uav_gps = [flipud(gps_point(:, end)); uav(3:4,end)];
                % update uav shape
                obj.UAV_shape_handle.update_pos(uav_gps);
                % update uav location
                set(obj.uav_path_handle, 'XData', gps_point(2,:), 'YData', gps_point(1,:));
                % update void circle
                if obj.config.Plot_Void
                 set(obj.void_handle, 'XData', obj.void_xp + uav_gps(2, end), ...
                                      'YData', obj.void_yp + uav_gps(1, end));
                end
            else
                error('Plot in 3D not implemented.\n'); 
            end
            
        end
    
        % update target location & path
        function update_target(obj, target_truth)
            if ~isempty(target_truth)
                if ~obj.config.Plot3D
                    % update target location & target path
                    for n = 1:size(target_truth, 2)
                        gps_point = GPS_Coord.Cart2GPS(obj.config.gps_ref, squeeze(target_truth(1:2, n, 1:end)));
                        set(obj.target_handle(n), 'XData', gps_point(2, end),...
                                               'YData', gps_point(1, end));
                        set(obj.target_path_handle(n), 'XData', gps_point(2, 1:end),...
                                                    'YData', gps_point(1, 1:end));
                    end
                else
                    error('Plot in 3D not implemented.\n');
                end
            end
        end

        function update_target_trial(obj, target_truth)
            if ~isempty(target_truth)
                if ~obj.config.Plot3D
                    % update target location & target path
                    for n = 1:size(target_truth, 2)
                        gps_point = GPS_Coord.Cart2GPS(obj.config.gps_ref, target_truth(1:2, n));
                        set(obj.target_handle(n), 'XData', gps_point(2, n),...
                                               'YData', gps_point(1, n));
                    end
                else
                    error('Plot in 3D not implemented.\n');
                end
            end
        end
        
        % update void circle
        function update_void(obj, uav)
            gps_point = GPS_Coord.Cart2GPS(obj.config.gps_ref, uav(1:2, end));
            set(obj.void_handle, 'XData', obj.void_xp + gps_point(2, end), 'YData', obj.void_yp+gps_point(1, end));
        end
        
        % update target ID text
        function update_text(obj, target_truth)
            if ~obj.config.Plot3D
                % update target location & target path
                for n = 1:size(target_truth, 2)
                    gps_point = GPS_Coord.Cart2GPS(obj.config.gps_ref, squeeze(target_truth(1:2, n, end)));
                    set(obj.target_text_handle(n), 'Position', [flipud(gps_point)', 0]);
                end
            else
                error('Plot in 3D not implemented.\n');
            end
        end
        
        % plot the return path of uav
        function update_uav_return(obj, uav, uav0)
            uav_gps = GPS_Coord.Cart2GPS(obj.config.gps_ref, uav(1:2, end));
            uav0_gps = GPS_Coord.Cart2GPS(obj.config.gps_ref, uav0(1:2, end));
            set(obj.uav_path_return_handle, 'XData', [uav_gps(2), uav0_gps(2)], 'YData', [uav_gps(1), uav0_gps(1)]);
        end
        
        function update_termination_value(obj, term_val)
            if obj.config.PlotTermination && ~obj.config.SeperatePlot
                set(obj.termination_th_handle, 'XData', [1, size(term_val, 2)]);
                for i = 1:obj.config.max_target
                    set(obj.termination_handle(i), 'XData', (1:size(term_val, 2)), 'YData', term_val(i, :));
                end
            end
        end
        
        function f = clone_figure(obj)
            f = figure('Position', [100, 100, 1280, 720]);
            tiledlayout(4, 7);
            ax1 = nexttile(1, [4,4]);
            copyUIAxes(obj.app.UIAxes_Main, ax1);

            ax2 = nexttile(5, [2,3]);
            copyUIAxes(obj.app.UIAxes_RMSE, ax2);

            ax3 = nexttile(19, [2,3]);
            copyUIAxes(obj.app.UIAxes_Term, ax3);
        end
        
        function toggle_truth(obj)
            current_state = obj.target_handle(1).Visible;
            if strcmp(current_state, 'on')
                toggle_state = 'off';
            else
                toggle_state = 'on';
            end
            
            for n = 1:size(obj.target_handle, 1)
                set(obj.target_handle(n), 'Visible', toggle_state);
                set(obj.target_path_handle(n), 'Visible', toggle_state);
                set(obj.target_text_handle(n), 'Visible', toggle_state);
            end
        end
        
    end
    
    
end


function keyboard_control_handler(hObject, eventdata)
    switch eventdata.Key
        case 'm'    % simple move
            [x,y] = ginput(1);
            control.type = Control_Type.Move;
            control.waypoint = [x; y; 0; 0];
            set(hObject, 'UserData', control);
        case 'r'    % rotate
            control.type = Control_Type.Rotate;
            set(hObject, 'UserData', control);
        case 'p'    % terminate
            control.type = Control_Type.Terminate;
            set(hObject, 'UserData', control);
        case 'f'    % toggle truth
            control.type = Control_Type.ToggleTruth;
            set(hObject, 'UserData', control);
    end
end
