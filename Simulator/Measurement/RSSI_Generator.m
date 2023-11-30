classdef RSSI_Generator < handle
    properties 
        sensor_config;       % sensor config struct
        target_config;       % target config struct
        DEM;                 % DEM data
    end     
    
    methods
        function obj = RSSI_Generator(sensor_conf, target_conf, DEM)
            obj.sensor_config = sensor_conf;
            obj.target_config = target_conf;
            obj.DEM = DEM;          
        end
        
        function z = get_RSSI(obj, x, uav)
            n_target = size(x, 2);
            target_id = (1:n_target);
            RSSI = nan(1, n_target);

            valid_target = ~isnan(x(1,:));  % valid target index (non-nan)
            
            RSSI(valid_target) = obj.get_RSSI_private(x(:, valid_target), uav) + ...
                                 obj.sensor_config.Sigma * randn(1, sum(valid_target));

            % simulate miss detection based on detection probability
            miss_detect = (rand(1, n_target) > obj.sensor_config.Pd);
            RSSI(miss_detect) = nan;

            z = table(target_id', obj.target_config.Frequency', RSSI', repmat(uav', n_target, 1));
            z.Properties.VariableNames = {'Target_ID', 'Frequency', 'RSSI', 'UAV'};
        end  
        
        function Pr = get_RSSI_private(obj, x, uav, varargin)
            % Instantiate inputParser
            p = inputParser;
            % Setup parsing schema
            addParameter(p, 'Relative_Dielectric', 15);
            addParameter(p, 'Conductivity', 5e-3);
            addParameter(p, 'Frequency', 150e6);
            parse(p, varargin{:});% Parse inputs

            ntarget = size(x, 2);

            % calculate target height above ground
            I = sub2ind(size(obj.DEM), round(x(1,:)), round(x(2,:)));
            target_height = x(3,:) - obj.DEM(I);

            %% calculate vegetation loss
            % empirical model from ITU-R P.833-9
            % only calculate vegetation loss if vegetation(tree) height is positive and
            % larger than target height

            tree_loss = zeros(1, ntarget);
            tree_loss_I = (obj.target_config.Tree_Height > target_height);
            height_diff = uav(3)-x(3, tree_loss_I);
            theta = atan2d(abs(height_diff), sqrt(sum((uav(1:2) - x(1:2, tree_loss_I)).^2)));

            d = (obj.target_config.Tree_Height(tree_loss_I) - target_height(tree_loss_I)) /sind(theta);    % vegetation depth

            tree_loss(tree_loss_I) = 0.25*(p.Results.Frequency/1e6).^(0.39) .* d.^(0.25) .* theta.^(0.05); 

            switch obj.sensor_config.Antenna
                case Antenna_Type.H
                    [gain, ~, r] = Get_Antenna_Gain(x, uav, obj.sensor_config.Antenna_model);
                case Antenna_Type.Array
                    [gain, ~, r] = Get_Antenna_Gain_phase_array(x, uav);
                case Antenna_Type.Isotropic
                    [~,~,r] = Get_Antenna_Gain_phase_array(x, uav);
                    gain = 0;
            end


            %% calculate uav height to ground
            uav_x_idx = round(uav(1));
            uav_y_idx = round(uav(2));
            if uav_x_idx < 1; uav_x_idx = 1; end
            if uav_x_idx > size(obj.DEM, 2); uav_x_idx = size(obj.DEM, 2); end
            if uav_y_idx < 1; uav_y_idx = 1; end
            if uav_y_idx > size(obj.DEM, 2); uav_y_idx = size(obj.DEM, 1); end
            Height_Rx = uav(3,:) - obj.DEM(uav_x_idx, uav_y_idx);

            terrain_loss = zeros(1, ntarget);
            %% calculate terrain loss
            for i = 1:ntarget
                % calculate terrain loss
                terrain_loss(i) = obj.terrain_loss(x(:, i), uav, target_height(i), Height_Rx, p.Results.Frequency/1e9);
            end

            Pr = obj.target_config.P0 - obj.target_config.Path_Loss.*10.*log10(r) + gain - tree_loss - terrain_loss; 
        end
        
        function [t_loss, is_smooth, VISPROFILE, dist, h] = terrain_loss(obj, x, uav, TX_height, Rx_height, freq)
            %% ITU terrain model

            % [INPUT]
            % x: target state: [x, y, z]
            % uav: uav state: [x, y, z, hdg]
            % freq: tag frequency in GHz
            % DEM: DEM maxtrix

            % [OUTPUT]
            % t_loss: terrain loss
            % is_smooth: whether terrain is considered as smooth based on Rayleigh criterion


            % check if target is within LOS
            DEM_size = size(obj.DEM,1);
            convert_c = DEM_size/rad2deg(DEM_size/6371000); % conversion constant, convert dist to degree
            refvec = [convert_c, rad2deg(DEM_size/6371000), 0];  
            [~,VISPROFILE,dist,h] = los2(obj.DEM, refvec, uav(1)/convert_c, uav(2)/convert_c, x(1)/convert_c, x(2)/convert_c, Rx_height, TX_height);

            if length(dist) == 1
                dist = dist*ones(2,1);
            end

            % find the minimum antenna height above the terrain
            LOS = linspace(uav(3), x(3), length(h))';
            height_difference = LOS - h;

            [min_h, min_I] = min(height_difference(1:end-round(0.05*length(h))));

            % find distance of obstruction from two terminals
            d = max(dist)/1000;
            d1 = dist(min_I)/1000;
            d2 = d - dist(min_I)/1000;

            % calculate terrain loss
            F1 = 17.3*sqrt(d1*d2/(freq*d));
            t_loss = -20*min_h/F1 +10;

            % ITU model not valid for loss less than 6 dB
            if t_loss < 6
                t_loss = 0;
            end


            % check terrain smoothness
            f_lambda = 3e8/(freq*1e9);    % wavelength
            dh = max(h) - min(h);
            if dh < f_lambda*max(dist)/(8*(x(3)+uav(3)))
                is_smooth = true;
            else
                is_smooth = false;
            end
        end
        
        function z = get_RSSI_debug(obj, x, uav)
            n_target = size(x, 2);
            target_id = (1:n_target);
            RSSI = nan(1, n_target);

            valid_target = ~isnan(x(1,:));  % valid target index (non-nan)
            
            RSSI(valid_target) = obj.get_RSSI_debug_private(x(:, valid_target), uav) + ...
                                 obj.sensor_config.Sigma * randn(1, sum(valid_target));

             % simulate miss detection based on detection probability
            miss_detect = (rand(1, n_target) > obj.sensor_config.Pd);
            target_id = target_id(~miss_detect)';
            Frequency = obj.target_config.Frequency(~miss_detect)';
            RSSI = RSSI(~miss_detect)';
            n_remain = sum(~miss_detect);
            

            z = table(target_id, Frequency, RSSI, ones(n_remain, 1), repmat(uav', n_remain, 1));
            z.Properties.VariableNames = {'Target_ID', 'Frequency', 'RSSI', 'time', 'UAV'};
        end 
        
        function Pr = get_RSSI_debug_private(obj, x, uav, varargin)
            % Instantiate inputParser
            p = inputParser;
            % Setup parsing schema
            addParameter(p, 'Relative_Dielectric', 15);
            addParameter(p, 'Conductivity', 5e-3);
            addParameter(p, 'Frequency', 150e6);
            parse(p, varargin{:});% Parse inputs

            switch obj.sensor_config.Antenna
                case Antenna_Type.H
                    [gain, ~, r] = Get_Antenna_Gain(x, uav, obj.sensor_config.Antenna_model);
                case Antenna_Type.Array
                    [gain, ~, r] = Get_Antenna_Gain_phase_array(x, uav);
                case Antenna_Type.Isotropic
                    [~,~,r] = Get_Antenna_Gain_phase_array(x, uav);
                    gain = 0;
            end

            Pr = obj.target_config.P0 - obj.target_config.Path_Loss.*10.*log10(r) + gain; 
        end     
        
        
        % get RSSI measurement in SITL&field_trial format
        function z = get_RSSI_SITL(obj, x, uav, realtime)
            n_target = size(x, 2);
            target_id = (1:n_target);
            RSSI = nan(1, n_target);

            valid_target = ~isnan(x(1,:));  % valid target index (non-nan)
            
            RSSI(valid_target) = obj.get_RSSI_private(x(:, valid_target), uav) + ...
                                 obj.sensor_config.Sigma * randn(1, sum(valid_target));

            % simulate miss detection based on detection probability
            miss_detect = (rand(1, n_target) > obj.sensor_config.Pd);
            RSSI(miss_detect) = nan;

            z = table(target_id', obj.target_config.Frequency', RSSI', realtime*ones(n_target, 1), repmat(uav', n_target, 1));
            z.Properties.VariableNames = {'Target_ID', 'Frequency', 'RSSI', 'time', 'UAV'};
        end
        
        % DEBUG: get RSSI measurement in SITL&field_trial format
        function z = get_RSSI_SITL_debug(obj, x, uav, realtime)
            n_target = size(x, 2);
            target_id = (1:n_target);
            RSSI = nan(1, n_target);

            valid_target = ~isnan(x(1,:));  % valid target index (non-nan)
            
            RSSI(valid_target) = obj.get_RSSI_debug_private(x(:, valid_target), uav) + ...
                                 obj.sensor_config.Sigma * randn(1, sum(valid_target));

            % simulate miss detection based on detection probability
            miss_detect = (rand(1, n_target) > obj.sensor_config.Pd);
            target_id = target_id(~miss_detect)';
            Frequency = obj.target_config.Frequency(~miss_detect)';
            RSSI = RSSI(~miss_detect)';
            n_remain = sum(~miss_detect);
            

            z = table(target_id, Frequency, RSSI, realtime*ones(n_remain, 1), repmat(uav', n_remain, 1));
            z.Properties.VariableNames = {'Target_ID', 'Frequency', 'RSSI', 'time', 'UAV'};
        end
    end
    
end