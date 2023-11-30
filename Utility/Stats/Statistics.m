classdef Statistics < handle 
    % class for storing experiment settings and results
    
    properties
        stats;
        ID_list;        % list of target ID
        save_measurement = false;
    end
    
    
    methods
        % l:                     -- maximum target numbers
        % ID_list                -- list of target ID
        % mode: 'default'
        %       'simulation'     -- also record truth
        function obj = Statistics(max_target, ID_list, max_time, config, mode, save_meas)    
               
            % TO BE ADDED:
            % DEM?
            %   
            obj.stats.config = config;
            obj.ID_list = ID_list;
            obj.stats.uav_path = [];  % uav path history
            obj.stats.action_history = cell(max_time, 1);    % action history 
            obj.stats.estimation = cell(max_target, 1);      % estimation history
            obj.stats.measurement = cell(max_time, 1);     % measurement history
            obj.stats.real_time = nan(max_target, 1);      
            obj.stats.found_time = nan(max_target, 1);       % time when target is found
            obj.stats.r = nan(max_target, max_time);         % existence probability
            obj.stats.error = nan(3, max_target);            % error in x, y and z
            obj.stats.travel_dist = 0;                       % total travel distance
            obj.stats.measurement_type = [0, 0];             % number of different measurement [bearing, total]
            obj.stats.error_history = cell(max_target, 1);
            
            if strcmp(mode, 'simulation')
                obj.stats.truth = [];
            end
            
            obj.save_measurement = save_meas;
            
            
            % initialize cell array
            for n = 1:max_target
                obj.stats.estimation{n} = nan(3, max_time);
                obj.stats.error_history{n} = nan(obj.stats.config.sim_target_config.nx, max_time);
            end
        
        end
        
        function cleanup(obj, t, naction)
            if ~isempty(obj.stats.uav_path)
                obj.stats.uav_path = obj.stats.uav_path(:, 1:t);
            end
            obj.stats.action_history = obj.stats.action_history(1:naction);
            for n = 1:length(obj.stats.estimation)
                obj.stats.estimation{n} = obj.stats.estimation{n}(:, 1:t);
                obj.stats.error_history{n} = obj.stats.error_history{n}(:, 1:t);
            end
            obj.stats.r = obj.stats.r(:, 1:t);
            obj.stats.measurement = obj.stats.measurement(1:t);
        end
        
        
        
        % record data
        function [est] = record(obj, z, density, found_report, time_idx, real_time)
            % save measurement 
            obj.record_meas(z, time_idx);
            
            % save estimation
            est = obj.get_estimate(density);
            for i = 1:length(density)
             obj.stats.estimation{i}(:, time_idx) = est(:, i);
            end
             
            if ~isempty(obj.stats.truth)
                if size(obj.stats.truth, 3) == 1
                    current_truth = obj.stats.truth;
                else
                    current_truth = squeeze(obj.stats.truth(:, :, time_idx));
                end

                for i = 1:length(density)
                    obj.stats.error_history{i}(:, time_idx) = est(:, i) - current_truth(:, i, 1);
                end
            end
             
             % save existence
             obj.stats.r(:, time_idx) = obj.extract_existence(density);
             
             % save found_time
             if ~isempty(found_report)
                for n = 1:size(found_report, 1)
                    obj.stats.found_time(obj.ID_list == found_report(n, 1)) = time_idx;
                end
             end
            
        end
        
        % record measurement (RSSI)
        function record_meas(obj, z, time_idx)
            if obj.save_measurement
                obj.stats.measurement{time_idx} = z;
            end
        end
        
        % save selected action
        function record_action(obj, uav, action, action_index, time_idx, real_time)
             % uav:                 current uav location
             % action:              array
             % action_index:        nth action
             % time_idx             time index
             % real_time            real time when action is decided
             obj.stats.action_history{action_index}.action = action;
             obj.stats.action_history{action_index}.idx = time_idx;
             obj.stats.action_history{action_index}.idx = time_idx;
             obj.stats.travel_dist = obj.stats.travel_dist + sqrt(sum(action.waypoints(1:2,end) - uav(1:2)).^2);
             if nargin > 5
                 obj.stats.action_history{action_index}.real_time = real_time;
             end
             
             % count bearing and total action
             if action.type == Action_Type.AoA || action.type == Action_Type.Combine
                 obj.stats.measurement_type(1) = obj.stats.measurement_type(1) + 1;
             end
             obj.stats.measurement_type(2) = obj.stats.measurement_type(2) + 1;
        end
        
        % save target ground truth
        function record_truth(obj, truth)
            obj.stats.truth = truth;
        end
        
        % save uav path
        function record_uav_path(obj, uav)
            obj.stats.uav_path = uav;
        end
        
        % export stored data only
        function stat = export(obj)
            % trim useless data to reduce disk space and memory usage
            max_t = max(obj.stats.found_time);
            obj.stats.action_history = obj.stats.action_history(~cellfun('isempty', obj.stats.action_history));
            obj.stats.r = obj.stats.r(:, 1:max_t);
            
            for n = 1:length(obj.stats.estimation)
                obj.stats.estimation{n} = obj.stats.estimation{n}(:, 1:max_t);
            end
            
            stat = obj.stats; 
        end
        
        % calculate error
        function get_error(obj)
            if ~isempty(obj.stats.truth)
                for n = 1:length(obj.stats.found_time)
                    if ~isnan(obj.stats.found_time(n))
                        if ndims(obj.stats.truth) == 3
                            % calculate error
                            obj.stats.error(:, n) = obj.stats.estimation{n}(:, obj.stats.found_time(n)) - ...
                                                    squeeze(obj.stats.truth(:, n, obj.stats.found_time(n)));
                        elseif ismatrix(obj.stats.truth)
                            obj.stats.error(:, n) = obj.stats.estimation{n}(:, obj.stats.found_time(n)) - ...
                                                    squeeze(obj.stats.truth(:, n));
                        end
                    end
                end
            end  
        end
        
        % report tracking result
        function report_result(obj)
            if ~isempty(obj.stats.truth)  
                error_x = obj.stats.error(1,:)';
                error_y = obj.stats.error(2,:)';
                error_z = obj.stats.error(3,:)';
                error_xy = sqrt(error_x.^2 + error_y.^2);
                error_xyz = sqrt(error_xy.^2 + error_z.^2);
                
                % print error
                result = table(obj.ID_list, obj.stats.found_time, error_x, error_y, error_z, error_xy, error_xyz);
                result.Properties.VariableNames = {'Target_ID', 'Localized_Time', 'Error_X','Error_Y','Error_Z','Error_XY', 'Error_XYZ'};
                
                disp(result);
                fprintf('Localized time: %f \n', max(obj.stats.found_time));
                fprintf('Total mean RMSE: %f \n', mean(error_xy));
                fprintf('Travel distance: %f \n', obj.stats.travel_dist);
                fprintf('AoA count: %f, percentage: %f \n', obj.stats.measurement_type(1), obj.stats.measurement_type(1)/obj.stats.measurement_type(2));
            end       
        end
        
        
        % handle unlocalized target by using the lastest estimate location
        % use when time runs out
        function record_unlocalized(obj, t)
             % find unlocalized target
             obj.stats.found_time(isnan(obj.stats.found_time)) = t;
        end
    end
    
    
    
    % contain function for analysing results
    methods (Access = private)
        %% ======================================================
        
        
        
        %% ================ Utils functions ======================
        % extract existence probability (if it exist)
        function r = extract_existence(obj, density)
            if isfield(density{1}, 'r')
                r = cellfun(@(x) x.r, density);
            else
                r = nan(length(density), 1);
            end
        end
        
        % get current estimatie
        function est = get_estimate(obj, density)
            est = cell2mat(cellfun(@(x) mean(x.particles, 2), density, 'UniformOutput', false)');
        end
        
    end
    
    
    
end