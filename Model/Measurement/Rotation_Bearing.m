classdef Rotation_Bearing < handle
    properties
        config;
        record;
        start_time;
        end_time;
    end
    
    properties (Access = private)
         
    end
    
    
    methods
        % constructor
        function obj = Rotation_Bearing(config)
            obj.config = config;
            obj.record = cell(config.Samples, 1);
            obj.start_time = config.Move_time;
            obj.end_time = config.Move_time + config.Samples;
        end
        
        % start recording if action is bearing measurement action
        function check_start_condition(obj, action, t)
             if action.type == Action_Type.AoA || action.type == Action_Type.Combine
                 obj.start_time = t;
                 obj.end_time = t + obj.config.Samples;
             end
        end
        
        function check_start_condition2(obj, t)
             obj.start_time = t;
             obj.end_time = t + obj.config.Samples;
        end
        
        
        
        % record data
        function record_data(obj, z, t)
            if t > obj.start_time && t <= obj.end_time
                obj.record{t - obj.start_time} = z;
            end
        end
        
        
        % calculate bearing measurement
        function [z, ref_i, raw_meas, meas, uav] = get_bearing(obj)
            % get target number and target id
            Target_ID = [];
            Frequency = [];
            for n = 1:length(obj.record)
                if ~isempty(obj.record{n})
                    Target_ID = union(Target_ID, obj.record{n}.Target_ID);
                    Frequency = union(Frequency, obj.record{n}.Frequency);
                end
            end
            ntarget = min(length(Target_ID), max(Target_ID));
            bearing = nan(ntarget, 1); 
            corr_coef = nan(ntarget, 1);
            meas = cell(ntarget, 1);
            raw_meas = obj.record;
            uav = cell(ntarget, 1);
            ref_i = cell(ntarget, 1);
            
            Target_ID = sort(Target_ID);

            for n = 1:ntarget
                % extract measurement from record
                meas{n} = nan(length(obj.record), 1);
                uav{n} = nan(4, length(obj.record));
                for i = 1:length(obj.record)
                     if ~isempty(obj.record{i})
                         idx = find(obj.record{i}.Target_ID == Target_ID(n));
                         if ~isempty(idx)
                             [meas{n}(i), max_i] = max(obj.record{i}.RSSI(idx));
                             uav{n}(:, i) = obj.record{i}.UAV(max_i, :)';
                         end
                     end
                end

                [bearing(n), corr_coef(n), ref_i{n}] = obj.cal_bearing(meas{n}, uav{n});
            end

            % construct table
            z = table(Target_ID, Frequency, bearing, corr_coef);
            z.Properties.VariableNames = {'Target_ID', 'Frequency', 'AoA', 'Coef'};
        end
        
        % calculate bearing measurement
        function [z, ref_i, meas] = get_bearing_debug(obj, raw_meas)
            % get target number and target id
            Target_ID = [];
            Frequency = [];
            for n = 1:length(raw_meas)
                if ~isempty(raw_meas{n})
                    Target_ID = union(Target_ID, raw_meas{n}.Target_ID);
                    Frequency = union(Frequency, raw_meas{n}.Frequency);
                end
            end
            ntarget = min(length(Target_ID), max(Target_ID));
            bearing = nan(ntarget, 1); 
            corr_coef = nan(ntarget, 1);
            meas = cell(ntarget, 1);
            uav = cell(ntarget, 1);
            ref_i = cell(ntarget, 1);
            
            Target_ID = sort(Target_ID);

            for n = 1:ntarget
                % extract measurement from record
                meas{n} = nan(length(raw_meas), 1);
                uav{n} = nan(4, length(raw_meas));
                for i = 1:length(raw_meas)
                     if ~isempty(raw_meas{i})
                         idx = find(raw_meas{i}.Target_ID == Target_ID(n));
                         if ~isempty(idx)
                              [meas{n}(i), max_i] = max(raw_meas{i}.RSSI(idx));
                              uav{n}(:, i) = raw_meas{i}.UAV(max_i, :)';
                         end
                     end
                end

%                 calculate bearing
                [bearing(n), corr_coef(n), ref_i{n}] = obj.cal_bearing(meas{n}, uav{n});
            end

            % construct table
            z = table(Target_ID, Frequency, bearing, corr_coef);
            z.Properties.VariableNames = {'Target_ID', 'Frequency', 'AoA', 'Coef'};
        end
        
        
        % get the maximum RSSI during bearing measurement
        function meas = get_max(obj)
            meas = max(cell2mat(cellfun(@(x) x.RSSI, obj.record, 'UniformOutput', false)'), [], 2); 
        end
        
        
    end
    
    methods %(Access = private)
        % generate reference antenna gain pattern
        function gain = gen_ref_gain(obj, uav, x)
            default_r = 250;    % assume target distance
            if isempty(x)
                ref_target = [default_r*sin(uav(4)), default_r*cos(uav(4)), uav(3)]'; 
            else
               d = sqrt(sum((uav(1:2) - xk(1:2)).^2));
               ref_target = [d*sin(uav(4)), d*cos(uav(4)), uav(3)]';
            end 
            ref_target(1:2) = ref_target(1:2) + uav(1:2);
            
            ref_uav = uav;
            ref_uav(4) = 0;
            
            switch obj.config.Antenna
                case Antenna_Type.H
                    gain = Get_Antenna_Gain(ref_target, ref_uav, obj.config.Antenna_model);
                case Antenna_Type.Array
                    gain = Get_Antenna_Gain_phase_array(ref_target, ref_uav);
                case Antenna_Type.NoBackH
                    gain = Get_Antenna_Gain(ref_target, ref_uav, obj.config.Antenna_model);
                otherwise
                    error('Error: selected antenna not implemented.');
            end
        end
        
        % calculate bearing (single target)
        function [bearing, p, ref_i] = cal_bearing(obj, z, uav)
%             % interpolate data
            theta_i = deg2rad((1:360));
            meas_i = zeros(360, 1);
            slot_counter = zeros(360, 1);
            tmp_theta = wrapTo2Pi(uav(4,:));
            [tmp_theta, sorted_idx] = sort(tmp_theta);
            tmp_theta = rad2deg(tmp_theta);
            tmp_meas = z(sorted_idx);
            tmp_meas(isinf(tmp_meas)) = nan;
            tmp_meas = normalize(tmp_meas, 'range');
            
            for n = 1:length(tmp_theta)
                 idx = round(tmp_theta(n));
                 if idx == 0
                     idx = 360;
                 end
                 
                 if ~isnan(idx) && ~isnan(tmp_meas(n))
                     if meas_i(idx)==0
                         meas_i(idx) = tmp_meas(n);
                         slot_counter(idx) = slot_counter(idx) + 1;
                     else
                         meas_i(idx) = (meas_i(idx)*slot_counter(idx) + tmp_meas(n))/ (slot_counter(idx)+1);
                         slot_counter(idx) = slot_counter(idx) + 1;
                     end
                 end
            end

            theta_i = [zeros(3, 360); theta_i];
            % calculate ref_gain
            ref_i = zeros(size(meas_i));
            for i = 1:360
                ref_i(i) = obj.gen_ref_gain(theta_i(:, i), []);
            end
            ref_i = normalize(ref_i, 'range');
            
%             % PCA?
%             valid_idx = (meas_i ~= 0);
%             tmp_meas = meas_i(valid_idx)';
%             tmp_theta = theta_i(4,valid_idx);
%             tmp_M = [tmp_meas .*cos(tmp_theta); tmp_meas .*sin(tmp_theta)];
%             [U,~,~] = svd(tmp_M);
%             bearing = atan2(U(2,1), U(1,1));
%             p = nan;

%             % Peak value heading bearing
%             [~,idx2] = max(meas_i);
%             p_max = nan;
%             bearing_2 = theta_i(4, idx2);

            % Correlation based method
            [idx, p] = obj.angle_corr(meas_i, ref_i, Correlator_Type.Pearson);
            bearing = wrapTo2Pi(theta_i(4, idx));
            
            [idx2, p2] = obj.angle_corr(meas_i, ref_i, Correlator_Type.Dot);
            bearing2 = wrapTo2Pi(theta_i(4, idx2));
            
            if isnan(p) || isnan(p2)
                bearing = nan;
                p = nan;
            else
                % select the second correlator to avoid 180 deg err in
                % measurement
                if abs(angdiff(bearing, bearing2)) > pi/2
                    bearing = bearing - pi;
                end
            end
        end
        
        function [bearing, p, ref_i] = cal_bearing_low_sample(obj, z, uav)     
            meas_i = normalize(z, 'range');
            meas_i(isnan(meas_i)) = 0;   
            
            theta_i = unwrap(uav(4,:));
            % calculate ref_gain
            ref_i = nan(size(meas_i));
            
            % calculate bearing
            [~, idx] = max(meas_i);
            p = nan;
            % antenna pattern assume NW coordinate (0 degree at +y)
            bearing = wrapTo2Pi(theta_i(1, idx)); 
        end
            
        % bearing gain pattern correlator
        function [max_arg, p] = angle_corr(obj, z, pattern, correlator_type)
            
            switch correlator_type
                case Correlator_Type.Pearson
                    correlator = @obj.pearson_corr;
                case Correlator_Type.Robust
                    correlator = @obj.robust_corr;
                case Correlator_Type.MAD
                    correlator = @obj.mad_corr;
                case Correlator_Type.Sn
                    correlator = @obj.Sn_corr;
                case Correlator_Type.Dot
                    correlator = @obj.simple_dot;
            end
                    
            if size(z,1) < size(pattern, 1)
                error('dimension mismatch');
            else
                data_length = size(z,1);
                corr_coef = zeros(data_length, 1);
                for i =1:data_length
                    corr_coef(i, 1) = correlator(circshift(pattern, i-1), z);
                end
                [p, max_arg] = max(corr_coef);
            end
        end
        
        function r = pearson_corr(obj, pattern, data)
            idx = (data ~= 0);
            x = pattern(idx);
            y = data(idx);  
            if isempty(x) || isempty(y)
                r = nan;
            else
                r = corr(x, y);
            end
        end
        
        function [r_robust] = robust_corr(obj, pattern, data)
            % Implementation of
            % "Calculating a robust correlation coefficient and quantifying its uncertainty"
            idx = (data ~= 0);
            n_sample = sum(idx);
            valid_pattern = pattern(idx);
            valid_data = data(idx);
            alpha = min(1 + n_sample/13, 15);
            leaveout = 1;    
            if length(valid_data) > 1
                R_actual = corr(valid_pattern, valid_data);

                n_choice = nchoosek(n_sample, leaveout);
                choice_idx = nchoosek(1:n_sample, leaveout);
                R_loot = zeros(1, n_choice);
                weight = zeros(1, n_choice);
                set_idx = boolean(ones(n_sample, 1));
                for i = 1:n_choice
                    set_idx(choice_idx(i)) = 0;
                    R_loot(i) = corr(valid_pattern(set_idx), valid_data(set_idx));
                    weight(i) = abs(R_loot(i) - R_actual).^alpha;
                    set_idx(choice_idx(i)) = 1;
                end

                r_robust = sum(weight .* R_loot)/sum(weight);      
            elseif length(valid_data) == 1
                r_robust = valid_data * valid_pattern;
            else
                r_robust = nan;
            end
        end
        
        function r = mad_corr(obj, pattern, data)
            idx = (data ~= 0);
            x = pattern(idx);
            y = data(idx);  
            
            med_x = median(x);
            med_y = median(y);
            mad_x = mad(x);
            mad_y = mad(y);
            
            u = (x - med_x)/(sqrt(2)*mad_x) + (y - med_y)/(sqrt(2)*mad_y);
            v = (x - med_x)/(sqrt(2)*mad_x) - (y - med_y)/(sqrt(2)*mad_y);
            
            mad_u = mad(u);
            mad_v = mad(v);
            r = (mad_u^2 - mad_v^2)/(mad_u^2 + mad_v^2);
        end
        
        function r = Sn_corr(obj, pattern, data)
            idx = (data ~= 0);
            x = pattern(idx);
            y = data(idx); 
            
            med_x = median(x);
            med_y = median(y);
            Sn_x = obj.Sn(x);
            Sn_y = obj.Sn(y);
            
            u = (x - med_x)/(sqrt(2)*Sn_x) + (y - med_y)/(sqrt(2)*Sn_y);
            v = (x - med_x)/(sqrt(2)*Sn_x) - (y - med_y)/(sqrt(2)*Sn_y);
            
            Sn_u = obj.Sn(u);
            Sn_v = obj.Sn(v);
            
            r = (Sn_u^2 - Sn_v^2)/(Sn_u^2 + Sn_v^2);
        end
        
        function r = simple_dot(obj, pattern, data)
            idx = (data ~= 0);
            x = pattern(idx);
            y = data(idx);  
            r = dot(x, y)/length(x);
        end
        
        function d = Sn(obj, data)
            tmp = nan(size(data));
            for i = 1:length(data)
                tmp(i) = median(abs(data(i) - data));
            end
            d = 1.1926*median(tmp);
        end
        
        
    end
    
end
    
