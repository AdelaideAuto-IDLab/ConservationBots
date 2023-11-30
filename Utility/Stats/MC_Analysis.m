classdef MC_Analysis < handle
    properties
        stats;
        mode;
    end
    
    methods
        function obj = MC_Analysis(data, mode)
            obj.stats = data;
            obj.mode = mode;
            warning('Warning: MC Analysis currently can not handle nan value.');
        end
        
        
        % generate report
        function gen_report(obj)
            %  get truth
            truth = obj.stats{1}.truth;
            config = obj.stats{1}.config;
             
            ntarget = size(obj.stats{1}.error, 2);
            error = nan(3, ntarget);
            % extract mean error
            for n = 1:ntarget
                error(:, n) = mean(cell2mat(cellfun(@(x) abs(x.error(:, n))', obj.stats, 'UniformOutput', false)), 1)';
            end
            
            % MC size
            mc_size = length(obj.stats);
            
            % extract found time
            found_time = cell2mat(cellfun(@(x) x.found_time', obj.stats, 'UniformOutput', false));
             
            % extract travel distance
            travel_distance = cellfun(@(x) x.travel_dist, obj.stats);
            
            % extract uav path
            uav_path = cellfun(@(x) x.uav_path, obj.stats, 'UniformOutput', false);
            
            % extract action history
            action_history = cellfun(@(x) x.action_history, obj.stats, 'UniformOutput', false);
            
            % extract estimation history
            est_history = cellfun(@(x) x.estimation, obj.stats, 'UniformOutput', false);
            
            % extract measurement history
            measurement_history = cellfun(@(x) x.measurement, obj.stats, 'UniformOutput', false);
             
            % extract existence history
            r_history = cellfun(@(x) x.r, obj.stats, 'UniformOutput', false);
            
            % extract measurement status
            measurement_stats = cell2mat(cellfun(@(x) x.measurement_type, obj.stats, 'UniformOutput', false));
            
            % display result
            error_x = error(1,:)';
            error_y = error(2,:)';
            error_z = error(3,:)';
            error_xy = sqrt(error_x.^2 + error_y.^2);
            error_xyz = sqrt(error_xy.^2 + error_z.^2);

            % print error
            result = table((1:ntarget)', mean(found_time, 1)', error_x, error_y, error_z, error_xy, error_xyz);
            result.Properties.VariableNames = {'Target_ID', 'Found_Time', 'Error_X','Error_Y','Error_Z','Error_XY', 'Error_XYZ'};
            disp(result);
            
            fprintf('Localized time: %f \n', mean(max(found_time, [], 2)));
            fprintf('Total mean RMSE: %f \n', mean(error_xy));
            fprintf('Mean travel distance: %f \n', mean(travel_distance));
            fprintf('AoA Measurement count: %i, percentage: %f \n', mean(measurement_stats(:, 1)), mean(measurement_stats(:, 1))/mean(measurement_stats(:, 2)));
            
            
           % save result
            filename = strcat('Results/', num2str(mc_size), '-', string(obj.mode), '-', ...
                              num2str(size(truth,2)), 'targets-H', string(config.planner_config.Lookahead), '-',string(config.planner_config.Reward), ...
                              '_reward-', 'Void_', string(config.planner_config.Use_Void), '-', ...
                              'Term_', string(config.Term_config.mode), '-', string(config.Term_config.threshold(1)), ...
                              '-' , 'ENV_', config.area_config.DEM.ID, '-', num2str(config.uav_config.traject_time), 'T', '-', ...
                               '-', datestr(now,'mm-dd-HH_MM'), '.mat' ...
                              );
           
           obj.stats = [];
           save(filename, ...
                'config',...
                'truth', ...
                'ntarget', ...
                'error', ...
                'found_time', ...
                'travel_distance', ...
                'uav_path', ...
                'action_history', ...
                'est_history', ...
                'r_history', ...
                'measurement_history', ...
                'measurement_stats' ...
                );
        end
        
        
        % plot heat map
        function plot_heatmap(obj, resolution)
            uav_path = cellfun(@(x) x.uav_path, obj.stats, 'UniformOutput', false);
            
            
            n_path = length(uav_path);
            x = obj.stats{1}.config.area_config.DEM.X_Max;
            y = obj.stats{1}.config.area_config.DEM.Y_Max;

            hmap = zeros(round(x/resolution), round(y/resolution));

            for n = 1:n_path
               tmp_path = [round(uav_path{n}(1,:)/resolution); round(uav_path{n}(2,:)/resolution)]; 
               % inte

               for l = 1:length(uav_path{n})
                   if tmp_path(1,l)<x/resolution && tmp_path(2,l) < y/resolution
                    hmap(tmp_path(1,l)+1, tmp_path(2,l)+1) = hmap(tmp_path(1,l)+1, tmp_path(2,l)+1) + 1;
                   end
               end
            end

            % convert to log scale
            hmap = hmap + 1;
            hmap = 10*log(hmap);

            hmap = hmap/max(hmap,[],'All');

            % plot 
            figure();
            imagesc(hmap', 'XData',[0, x], 'YData', [0, y]);
            xlabel('x (m)');
            ylabel('y (m)');
            set(gca,'YDir','normal')
        end
        
        
    end
    
    
end