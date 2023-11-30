classdef Result_plot < handle
    properties            
        config;
        fig_handle;             % figure handle
        fig_ax;                 % figure axe handle
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
        error_xy_handle;
        
        fig_termination_val;    % termination progress
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
         
         void_xp;
         void_yp;
        
    end

    
    
    methods 
        function obj = Result_plot(max_target, varargin)
            p = inputParser;
            addParameter(p, 'Plot3D', false, @islogical);
            addParameter(p, 'DEM', []);    % DEM matrix
            addParameter(p, 'Plot_Void', false, @islogical);
            addParameter(p, 'Void_r', 0);  % void radius
            addParameter(p, 'Area', []);    % are boundary
            addParameter(p, 'PlotTermination', false, @islogical);
            addParameter(p, 'Term_th', 2e4);    % termination threshold
            
            parse(p, varargin{:});
            
            obj.config.Plot3D = p.Results.Plot3D;
            obj.config.DEM = p.Results.DEM;
            obj.config.Plot_Void = p.Results.Plot_Void;
            obj.config.Void_r = p.Results.Void_r;
            obj.config.max_target = max_target;
            obj.config.area = p.Results.Area;
            obj.config.PlotTermination = p.Results.PlotTermination;
            obj.config.Term_th = p.Results.Term_th;
            
            
            % setup figure
            obj.fig_handle = figure();
            obj.fig_ax = axes(obj.fig_handle);
            hold on;
            obj.fig_rmse = figure();
            hold on;
            

            
            if ~obj.config.Plot3D
                if obj.config.PlotTermination
                    obj.fig_termination_val = figure();
                    hold on;
                    obj.termination_handle = bar(nan, nan);
                    obj.termination_th_handle = line([nan, nan], [obj.config.Term_th, obj.config.Term_th], 'LineWidth', 2, 'Color', 'r');
                    xlabel('Target');
                    ylabel('Termination Threshold');
                end
                
                
                figure(obj.fig_handle);
                ax = gca();
                obj.UAV_shape_handle = UAV_shape(ax, 'k', 10);
                obj.particle_handle = gobjects(obj.config.max_target, 1);
                obj.target_handle = gobjects(obj.config.max_target, 1);
                obj.est_handle = gobjects(obj.config.max_target, 1);
                obj.target_text_handle = gobjects(obj.config.max_target, 1);
                obj.uav_path_handle = plot(nan, nan, 'k', 'LineWidth', obj.linewidth);
                obj.uav_path_return_handle = line(nan, nan, 'Color', 'k', 'LineStyle', '--', 'LineWidth', obj.linewidth);

                % plot boundary
                obj.boundary_handle = plot(polyshape(obj.config.area'),  'EdgeColor', 'k', 'LineWidth', 2, 'FaceAlpha', 0.2);
                
                if obj.config.Plot_Void
                    obj.void_handle = plot(nan,nan, 'g', 'LineStyle', '--');
                    ang=0:0.01:2*pi;
                    obj.void_xp = obj.config.Void_r * cos(ang);
                    obj.void_yp = obj.config.Void_r * sin(ang);
                end

                % initialize plot
                for i = 1:obj.config.max_target
                    obj.particle_handle(i) = scatter(nan, nan, obj.active_particle_size, obj.color(i, :), 'Marker', '*');
                    obj.target_handle(i) = scatter(nan, nan, obj.marker_size, 'Marker', 's', 'LineWidth', obj.linewidth, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', obj.color(i, :));
                    obj.target_path_handle(i) = plot(nan, nan, 'LineStyle', '-', 'Color', obj.color(i, :), 'LineWidth', obj.linewidth); 
                    obj.est_handle(i) = scatter(nan, nan, obj.marker_size , 'Marker', '^', 'LineWidth', obj.linewidth, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', obj.color(i, :));
                    obj.target_text_handle(i) = text(nan, nan, ['\leftarrow Target ', num2str(i)], 'FontSize', obj.text_size, 'FontWeight', 'bold');
                end
                
                axis equal;
                if ~isempty(obj.config.area)
                    xlim([min(obj.config.area(1,:)) - 50, max(obj.config.area(1,:))+50]);
                    ylim([min(obj.config.area(2,:)) - 50, max(obj.config.area(2,:))+50]);
                else
                xlim([-50, size(obj.config.DEM, 2)+50]);
                ylim([-50, size(obj.config.DEM, 1)+50]);
                end
                
                figure(obj.fig_rmse);
                obj.error_xy_handle = gobjects(obj.config.max_target, 1);
                for i = 1:obj.config.max_target
                    obj.error_xy_handle(i) = plot(nan, nan, 'LineStyle', '-', 'Color', obj.color(i, :), 'LineWidth', obj.linewidth); 
                end
                title('XY Error'); xlabel('times (s)'); ylabel('Error (m)'); grid on;
                
            else
                % TODO: implement 3D plotting
                error('Plot in 3D not implemented.\n'); 
            end
        end
        
        % update everything
        function update_plot(obj, density, uav, target_truth, est, selected_target)
            obj.update_density(density);
            obj.update_est(est);
            obj.update_uav(uav);
            obj.update_target(target_truth);
            obj.update_text(target_truth);
            
            if obj.config.Plot_Void
                obj.update_void(uav);
            end
            
            % update title
            title(obj.fig_ax, sprintf('Selected target: %i', selected_target));
                       
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
                    set(obj.particle_handle(n), 'XData', density{n}.particles(1,:),...
                                                'YData', density{n}.particles(2,:));
                 end
             else
                 error('Plot in 3D not implemented.\n'); 
             end
        end
        
        % update estimated location
        function update_est(obj, est)
             if ~obj.config.Plot3D
                 for n = 1:size(est, 2)
                    set(obj.est_handle(n), 'XData', est(1, n), 'YData', est(2,n));
                 end
             else
                 error('Plot in 3D not implemented.\n'); 
             end
        end
        
        % update uav location & void circle
        function update_uav(obj, uav)
            if ~obj.config.Plot3D
                % update uav shape
                obj.UAV_shape_handle.update_pos(uav(:, end));
                % update uav location
                set(obj.uav_path_handle, 'XData', uav(1,:), 'YData', uav(2,:));
                % update void circle
                if obj.config.Plot_Void
                 set(obj.void_handle, 'XData', obj.void_xp + uav(1, end), ...
                                      'YData', obj.void_yp + uav(2, end));
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
                        set(obj.target_handle(n), 'XData', squeeze(target_truth(1, n, end)),...
                                               'YData', squeeze(target_truth(2, n, end)));
                        set(obj.target_path_handle(n), 'XData', squeeze(target_truth(1, n, 1:end)),...
                                                    'YData', squeeze(target_truth(2, n, 1:end)));
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
                        set(obj.target_handle(n), 'XData', target_truth(1, n),...
                                               'YData', target_truth(2, n));
                    end
                else
                    error('Plot in 3D not implemented.\n');
                end
            end
        end
        
        % update void circle
        function update_void(obj, uav)
            set(obj.void_handle, 'XData', obj.void_xp + uav(1, end), 'YData', obj.void_yp+uav(2, end));
        end
        
        % update target ID text
        function update_text(obj, target_truth)
            if ~obj.config.Plot3D
                % update target location & target path
                for n = 1:size(target_truth, 2)
                    set(obj.target_text_handle(n), 'Position', [squeeze(target_truth(1:2, n, end))', 0]);
                end
            else
                error('Plot in 3D not implemented.\n');
            end
        end
        
        % plot the return path of uav
        function update_uav_return(obj, uav, uav0)
            set(obj.uav_path_return_handle, 'XData', [uav(1), uav0(1)], 'YData', [uav(2), uav0(2)]);
        end
        
        function update_termination_value(obj, term_val)
            if obj.config.PlotTermination
                set(obj.termination_th_handle, 'XData', [-1, length(term_val)+1.5]);
                set(obj.termination_handle, 'XData', (1:length(term_val)), 'YData', term_val);
            end
        end
        
        function f = clone_figure(obj)
            % compatibilitym empty function
        end
        
    end
    
    
    methods (Access = private)
        
    end
    
    
    
end
