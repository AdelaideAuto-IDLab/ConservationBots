classdef Planner_greedy_POMDP < Planner
   properties 
      ID = 'Greed_POMDP';   % planner type
      config;           % planner configuration
      Meas_Model;       % measurement model
      Sys_Model;        % system model
      Action_Model;     % control action generator
      
      Meas_Model_2;     % secondary measurement model (AoA)
      
      Type;             % planner type
   end
   
   properties (Access = private)
       selected_density;        % index of selected density for planning 
   end
    
   
   methods 
       function obj = Planner_greedy_POMDP(config, meas_model, sys_model, action_model, secondary_meas_model)
           obj.config = config;
           obj.Meas_Model = meas_model;
           obj.Sys_Model = sys_model;
           obj.Action_Model = action_model;
           
           switch meas_model.Type
               case Measurement_Type.RSSI
                   obj.Type = Planner_Type.RSSI;
               case Measurement_Type.AoA
                   obj.Type = Planner_Type.AoA;
               case Measurement_Type.AoA_2P
                   obj.Type = Planner_Type.AoA_2P;
               otherwise
                   obj.Type = Planner_Type.Undefined;
           end
           
           if nargin == 5
               obj.Meas_Model_2 = secondary_meas_model;
               if obj.Meas_Model_2.Type == Measurement_Type.AoA
                   obj.Meas_Model_2 = secondary_meas_model;
                   obj.Type = Planner_Type.Combine;
               else
                   error('Error: The second measurement model must be bearing model');
               end
           end
                  
       end    
       
       
       function [action, selected_target] = plan(obj, uav, density)
           % density: cell array
           % actions: cell array      
           
           %% decide which target to plan for
           switch obj.config.Plan_target
               case 'Closest'
                   [obj.selected_density] = obj.get_closest_target(uav, density);
                   % calculate heading offset to make sure at least one
                   % action point to target estimated location
                   tmp_x = mean(density{obj.selected_density}.particles, 2);
                   heading_offset = atan2(tmp_x(1) - uav(1), tmp_x(2) - uav(2));     
               case 'All'
                   % select unlocalized targets
                   [obj.selected_density] = find(obj.get_unlocalized_target(density));
                   heading_offset = 0;
               otherwise
                   error('Error: Plan target:''[%s] not implemented.\n', obj.config.Plan_target);
           end
           selected_target = obj.selected_density;
           
           
           %% reduce particle number
           dst = obj.down_sample(density);
           
           %% decide lookahead steps
           if obj.config.Variable_lookahead
               lookahead = obj.get_variable_lookahead_step(uav, density);
           else
               lookahead = obj.config.Lookahead;
           end
           
           %% decide which planning mode to use
           action = obj.plan_private(uav, dst, lookahead, heading_offset);

       end
       
       
       % planning using PIMS 
       function [action] = plan_private(obj, uav, density, H, heading_offset)
           % generate control action sets
           headings = heading_offset+linspace(0, 2*pi*(1-1/obj.config.Heading_number), obj.config.Heading_number);
           
           switch obj.Type
               case Planner_Type.AoA
                   action_set =  obj.Action_Model.get_composit_waypoints_set(uav, headings);
                   reward = obj.filter_function(density, H, action_set, Action_Type.AoA);
                   
                   action.type = Action_Type.AoA;
               case Planner_Type.RSSI
                   action_set = obj.Action_Model.get_action_set_md(uav, headings);
                   reward = obj.filter_function(density, H, action_set, Action_Type.RSSI);
                   
                   action.type = Action_Type.RSSI;
               case Planner_Type.Combine
                   action_set_RSSI = obj.Action_Model.get_action_set_md(uav, headings);
                   reward_RSSI = obj.filter_function(density, H, action_set_RSSI, Action_Type.RSSI);
                   action_set_comb = obj.Action_Model.get_composit_waypoints_set(uav, headings);
                   reward_comb = obj.filter_function(density, H, action_set_comb, Action_Type.Combine);
                   
                   if max(reward_comb) > max(reward_RSSI)
                       action_set = action_set_comb;
                       reward = reward_comb;
                       action.type = Action_Type.Combine;
                   else
                       action_set = action_set_RSSI;
                       reward = reward_RSSI;
                       action.type = Action_Type.RSSI;
                   end                                   
                      
               case Planner_Type.AoA_2P
                   action_set = obj.Action_Model.get_action_set_rotation_md(uav, headings);
                   reward = obj.filter_function(density, H, action_set, Action_Type.AoA_2P);
                   action.type = Action_Type.AoA_2P;
                   
               otherwise
                   error('Error: Selected Planning Mode not implemented.');
           end

           [~, i] = max(reward);
%            action.waypoints = action_set{i}.waypoint;
           action.waypoints = action_set{i};

       end
          
       
       % filtering wrapper
       function [reward] = filter_function(obj, density, H, action_set, action_type)
           if obj.config.Mode == Planning_Meas_Mode.MC
               switch action_type
                   case Action_Type.RSSI
                       filtering = @obj.filtering_MC_RSSI;
                   case Action_Type.AoA
                       filtering = @obj.filtering_MC_AoA;
                   case Action_Type.Combine
                       filtering = @obj.filtering_MC_Combine;
                   otherwise
                       error('Error: invalid action type.');
               end
           elseif obj.config.Mode == Planning_Meas_Mode.PIMS
               switch action_type
                   case Action_Type.RSSI
                       filtering = @obj.filtering_PIMS_RSSI;
                   case Action_Type.AoA
                       filtering = @obj.filtering_PIMS_AoA;
                   case Action_Type.Combine
                       filtering = @obj.filtering_PIMS_Combine;
                   case Action_Type.AoA_2P
                       filtering = @obj.filtering_PIMS_AoA_2P;
                   otherwise
                       error('Error: invalid action type.');
               end
           end
           
           reward = zeros(length(action_set), 1); % reward for each action
           for a = 1:length(action_set)
               % copy original density
               density_copy = density(obj.selected_density);
               selected_action = action_set{a};
               % check global density void constraint
               if ~(~obj.config.Use_Void ||(obj.config.Use_Void && obj.check_void(selected_action, density)))
                   reward(a) = reward(a) - 100;
               end
               
               for h = 1:H
                   if h == 1
                       [density_copy, reward_list] = filtering(selected_action, density_copy);
                       if ~obj.config.Use_Void ||(obj.config.Use_Void && obj.check_void(selected_action, density_copy))
                           reward(a) = sum(reward_list);
                       else
                           reward(a) = reward(a) - 100;
                       end
                           
                   else
                       % generate next action
                       if ~isempty(selected_action)
                           selected_action = obj.Action_Model.get_action_md(selected_action(:, end), selected_action(4, end), obj.Action_Model.Uav_config.vmax);
                       end
                       if ~isempty(selected_action)
                           [density_copy, reward_list] = filtering(selected_action, density_copy);
                           reward(a) = reward(a) + obj.config.Discount^h * sum(reward_list);
                       else
                           % [TODO], action cause UAV to move out of bounds 
                           % select the action move uav closer into bounds
                       end
                   end
               end
           end           
           
       end
       
       
       
   end
    
   
   
   
   methods (Access = private)
       
       % calculate the variable lookahead step base on distance to target
       function H = get_variable_lookahead_step(obj, uav, density)
           d = obj.get_distance(uav, density(obj.selected_density));
           H = max(1, floor(d/(obj.Action_Model.Uav_config.vmax * obj.config.T)));
       end
       
       % find the closest and unlocalized target
       % return estimated state, index and heading
       function [i] = get_closest_target(obj, uav, density)
            % get list of unlocalized target
            available_target = obj.get_unlocalized_target(density);
            % calculate distance to density
            d = obj.get_distance(uav, density);
            avail_d = d(available_target);
            [~, ix] = min(avail_d);

            idx = find(available_target);
            i = idx(ix);  
        end
        
       % find unlocalized target
       function available_list = get_unlocalized_target(obj, density)
            available_list = ~cellfun(@(x) x.localized, density);
        end
       
       % reduce the particle number
       function [dst] = down_sample(obj, density)
           
            dst = cell(length(density), 1);
            for n = 1:length(density)
                % using multinomial resampling to reduce sample number
                sample_index = Multinomial_resampling(density{n}.weight, obj.config.Planning_sample);
                dst{n}.particles = density{n}.particles(:, sample_index);
                dst{n}.weight = 1/obj.config.Planning_sample * ones(obj.config.Planning_sample, 1);
                dst{n}.N = obj.config.Planning_sample;
                dst{n}.ID = density{n}.ID;
                if obj.config.Density_type == Density_Type.Bernoulli
                    dst{n}.r = density{n}.r;
                end 
            end
           
       end
       
       
       % partical filtering in planning, RSSI only action
       % uav: waypoints
       function [dst, reward] = filtering_PIMS_RSSI(obj, uav, density)
           dst = density;
           reward = nan(length(density), 1);
           
           % get PIMS
           PIMS_state = cell2mat(cellfun(@(x) mean(x.particles, 2), dst, 'UniformOutput', false)');
           for l = 1:size(uav, 2)
               % predict PIMS state
               PIMS_state= obj.Sys_Model.sys(PIMS_state, true); 
               for n = 1:length(density)
                   
                   % prediction
                   dst{n}.particles = obj.Sys_Model.sys(dst{n}.particles, true);
                   PIMS = obj.Meas_Model.meas_model(PIMS_state(:, n), uav(:, l), dst{n}.ID);     
                   
                   if PIMS > obj.Meas_Model.config.Sensitivity
                       % update
                       dst{n}.weight = dst{n}.weight .* obj.Meas_Model.likelihood(PIMS, dst{n}.particles, uav(:, l), dst{n}.ID);

                       % normalized weight
                       dst{n}.weight = dst{n}.weight/sum(dst{n}.weight);    
                   end
               end
               
               
           end
           
           for n = 1:length(density)
               % calculate reward
               reward(n) = obj.reward_function([], dst{n});   
               
               % resampling
               sample_index = Multinomial_resampling(dst{n}.weight, dst{n}.N);
               dst{n}.particles = dst{n}.particles(:, sample_index);
               dst{n}.weight = 1/dst{n}.N * ones(dst{n}.N, 1);     
           end
       end
       
       
       % partical filtering in planning, AoA only action
       % uav: waypoints
       function [dst, reward] = filtering_PIMS_AoA(obj, uav, density)
           dst = density;
           reward = nan(length(density), 1);
           
           % get PIMS
           PIMS_state = cell2mat(cellfun(@(x) mean(x.particles, 2), dst, 'UniformOutput', false)');
           for l = 1:size(uav, 2)
               % predict PIMS state
               PIMS_state= obj.Sys_Model.sys(PIMS_state, true);
               for n = 1:length(density)
                   % prediction
                   dst{n}.particles = obj.Sys_Model.sys(dst{n}.particles, true);
               end             
           end
           
           for n = 1:length(density)
               PIMS = obj.Meas_Model.meas_model(PIMS_state(:, n), uav(:, l), dst{n}.ID); 
               % AoA update
               dst{n}.weight = dst{n}.weight .* obj.Meas_Model.likelihood(PIMS, dst{n}.particles, uav(:, l), dst{n}.ID);
               % normalized weight
               dst{n}.weight = dst{n}.weight/sum(dst{n}.weight);    
               
               
               % calculate reward
               reward(n) = obj.reward_function([], dst{n});   
               
               % resampling
               sample_index = Multinomial_resampling(dst{n}.weight, dst{n}.N);
               dst{n}.particles = dst{n}.particles(:, sample_index);
               dst{n}.weight = 1/dst{n}.N * ones(dst{n}.N, 1);     
           end
       end
            
       % partical filtering in planning, combine action
       % uav: waypoints
       function [dst, reward] = filtering_PIMS_Combine(obj, uav, density)
           dst = density;
           reward = nan(length(density), 1);
           
           % get PIMS
           PIMS_state = cell2mat(cellfun(@(x) mean(x.particles, 2), dst, 'UniformOutput', false)');
           for l = 1:size(uav, 2)-obj.config.Rotation_time
               % predict PIMS state
               PIMS_state= obj.Sys_Model.sys(PIMS_state, true);
               for n = 1:length(density)
                   
                   % prediction
                   dst{n}.particles = obj.Sys_Model.sys(dst{n}.particles, true);
                   PIMS = obj.Meas_Model.meas_model(PIMS_state(:, n), uav(:, l), dst{n}.ID);
                   
                   if PIMS > obj.Meas_Model.config.Sensitivity
                       % update
                       dst{n}.weight = dst{n}.weight .* obj.Meas_Model.likelihood(PIMS, dst{n}.particles, uav(:, l), dst{n}.ID);
                       % normalized weight
                       dst{n}.weight = dst{n}.weight/sum(dst{n}.weight);   
                   end
               end
               
               
           end
           
           for n = 1:length(density)
               PIMS = obj.Meas_Model_2.meas_model(PIMS_state(:, n), uav(:, l), dst{n}.ID); 
               % AoA update
               dst{n}.weight = dst{n}.weight .* obj.Meas_Model_2.likelihood(PIMS, dst{n}.particles, uav(:, l), dst{n}.ID); 

               % normalized weight
               dst{n}.weight = dst{n}.weight/sum(dst{n}.weight);   
              

               % calculate reward
               reward(n) = obj.reward_function([], dst{n});   
               
               % resampling
               sample_index = Multinomial_resampling(dst{n}.weight, dst{n}.N);
               dst{n}.particles = dst{n}.particles(:, sample_index);
               dst{n}.weight = 1/dst{n}.N * ones(dst{n}.N, 1);     
           end
       end
           
       function [dst, reward] = filtering_PIMS_AoA_2P(obj, uav, density)
           dst = density;
           reward = nan(length(density), 1);
           
           % get PIMS
           PIMS_state = cell2mat(cellfun(@(x) mean(x.particles, 2), dst, 'UniformOutput', false)');
           PIMS_state_prev = PIMS_state;
           dst_prev = dst;
           uav_prev = uav(:, 1);
           for l = 1:size(uav, 2)
               % predict PIMS state
               PIMS_state = obj.Sys_Model.sys(PIMS_state, true); 
               for n = 1:length(density)           
                   % prediction
                   dst{n}.particles = obj.Sys_Model.sys(dst{n}.particles, true);
                   PIMS_set = {PIMS_state_prev; PIMS_state};
                   PIMS = obj.Meas_Model.meas_model(PIMS_set, [uav_prev, uav(:, l)], dst{n}.ID);     
                   
                   
                   % update
                   dst{n}.weight = dst{n}.weight .* obj.Meas_Model.likelihood(PIMS, {dst_prev{n}.particles; dst{n}.particles}, [uav_prev, uav(:, l)], dst{n}.ID);

                   dst{n}.weight(isnan(dst{n}.weight)) = 1e-100;
                   dst{n}.weight(dst{n}.weight <= 0) = 1e-100;

                   % normalized weight
                   dst{n}.weight = dst{n}.weight/sum(dst{n}.weight);   
               end
               
               uav_prev = uav(:,l);
               dst_prev = dst;
               PIMS_state_prev = PIMS_state;         
           end
           
           for n = 1:length(density)
               % calculate reward
               reward(n) = obj.reward_function([], dst{n});   
               
               % resampling
               sample_index = Multinomial_resampling(dst{n}.weight, dst{n}.N);
               dst{n}.particles = dst{n}.particles(:, sample_index);
               dst{n}.weight = 1/dst{n}.N * ones(dst{n}.N, 1);     
           end
       end
            
       
       % MC filtering
       function [dst, reward] = filtering_MC_RSSI(obj, uav, density)
           dst = density;
           reward = nan(length(density), 1);
           
           MC_idx = randi([1, dst{1}.N], obj.config.MC_sample, 1);
           for n = 1:length(density)
               for l = 1:size(uav, 2)
                   % prediction
                   dst{n}.particles = obj.Sys_Model.sys(dst{n}.particles, true); 
               end
               tmp_weight = zeros(size(dst{n}.weight));
               for s = 1:obj.config.MC_sample
                   MC_z = obj.Meas_Model.meas_model(dst{n}.particles(:, MC_idx(s)), uav(:, l), dst{n}.ID);     

                   if MC_z > obj.Meas_Model.config.Sensitivity
                       % update
                       tmp_weight = tmp_weight + 1/obj.config.MC_sample *  obj.Meas_Model.likelihood(MC_z, dst{n}.particles, uav(:, l), dst{n}.ID);
                   end
               end
               tmp_weight = tmp_weight .* dst{n}.weight;
               % normalized weight
               dst{n}.weight = tmp_weight/sum(tmp_weight);   
           end
           
           for n = 1:length(density)
               % calculate reward
               reward(n) = obj.reward_function([], dst{n});   
               
               % resampling
               sample_index = Multinomial_resampling(dst{n}.weight, dst{n}.N);
               dst{n}.particles = dst{n}.particles(:, sample_index);
               dst{n}.weight = 1/dst{n}.N * ones(dst{n}.N, 1);     
           end
       end
       
       function [dst, reward] = filtering_MC_AoA(obj, uav, density)
           dst = density;
           reward = nan(length(density), 1);
           
           MC_idx = randi([1, dst{1}.N], obj.config.MC_sample, 1);
           for l = 1:size(uav, 2)
               for n = 1:length(density)
                   % prediction
                   dst{n}.particles = obj.Sys_Model.sys(dst{n}.particles, true);
               end             
           end
           
           for n = 1:length(density)
               tmp_weight = zeros(size(dst{n}.weight));
               for s = 1:obj.config.MC_sample
                   MC_z = obj.Meas_Model.meas_model(dst{n}.particles(:, MC_idx(s)), uav(:, l), dst{n}.ID);  
                   % AoA update
                   tmp_weight = tmp_weight + 1/obj.config.MC_sample * obj.Meas_Model.likelihood(MC_z, dst{n}.particles, uav(:, l), dst{n}.ID);   
               end
               tmp_weight = tmp_weight .* dst{n}.weight;
               % normalized weight
               dst{n}.weight = tmp_weight/sum(tmp_weight); 
           end
           
           for n = 1:length(density)
               % calculate reward
               reward(n) = obj.reward_function([], dst{n});   
               
               % resampling
               sample_index = Multinomial_resampling(dst{n}.weight, dst{n}.N);
               dst{n}.particles = dst{n}.particles(:, sample_index);
               dst{n}.weight = 1/dst{n}.N * ones(dst{n}.N, 1);   
           end
       end
       
       function [dst, reward] = filtering_MC_Combine(obj, uav, density)
           dst = density;
           reward = nan(length(density), 1);
           
           MC_idx = randi([1, dst{1}.N], obj.config.MC_sample, 1);
           for l = 1:size(uav, 2)-obj.config.Rotation_time
               for n = 1:length(density)
                   % prediction
                   dst{n}.particles = obj.Sys_Model.sys(dst{n}.particles, true);
                   tmp_weight = zeros(size(dst{n}.weight));
                   
                   for s = 1:obj.config.MC_sample
                       MC_z = obj.Meas_Model.meas_model(dst{n}.particles(:, MC_idx(s)), uav(:, l), dst{n}.ID);  
                   
                       if MC_z > obj.Meas_Model.config.Sensitivity
                           % update
                           tmp_weight = tmp_weight + 1/obj.config.MC_sample * obj.Meas_Model.likelihood(MC_z, dst{n}.particles, uav(:, l), dst{n}.ID);
                       end
                   end  
                   dst{n}.weight = dst{n}.weight .* tmp_weight;
               end
           end
  
           for n = 1:length(density)
               tmp_weight = zeros(size(dst{n}.weight));
               for s = 1:obj.config.MC_sample
                   MC_z = obj.Meas_Model_2.meas_model(dst{n}.particles(:, MC_idx(s)), uav(:, l), dst{n}.ID); 
                   % AoA update
                   tmp_weight = tmp_weight + 1/obj.config.MC_sample *  obj.Meas_Model_2.likelihood(MC_z, dst{n}.particles, uav(:, l), dst{n}.ID); 
               end
               tmp_weight = dst{n}.weight .*tmp_weight;
               % normalized weight
               dst{n}.weight = tmp_weight/sum(tmp_weight);   
           end
           
           for n = 1:length(density)
               % calculate reward
               reward(n) = obj.reward_function([], dst{n});   
               
               % resampling
               sample_index = Multinomial_resampling(dst{n}.weight, dst{n}.N);
               dst{n}.particles = dst{n}.particles(:, sample_index);
               dst{n}.weight = 1/dst{n}.N * ones(dst{n}.N, 1);  
           end
       end
       
       % calculate the reward 
       function reward = reward_function(obj, current_density, future_density)
           switch obj.config.Reward
               case Reward_Type.Renyi
                   reward = obj.renyi_divergence(current_density, future_density);
               case Reward_Type.Shannon
                   reward = obj.shannon_entropy(current_density, future_density);
               case Reward_Type.Cauchy
                   reward = obj.cauchy_inequality(current_density, future_density);
               case Reward_Type.Circle
                   reward = obj.circle_reward(current_density, future_density);
               otherwise
                   error('Error: Selected reward not implemented.\n');
           end
       end 
       
       %% ===================== Reward functions ==========================
              
       % Renyi divergence reward
       function reward = renyi_divergence(obj, current_density, future_density)
           if isempty(current_density)  % assume uniform weight if no current density is provided
               reward = 1/(obj.config.alpha-1) * log(future_density.N^(obj.config.alpha-1) ...
                                             * sum(future_density.weight.^obj.config.alpha)); 
           else
               reward = 1/(obj.config.alpha-1) * log(dot(current_density.weight.^(1-obj.config.alpha), ...
                                                      (future_density.weight.^obj.config.alpha))); 
           end
           
       end 
       
       % shannon reward function
       function reward = shannon_entropy(obj, current_density, future_density)
           if isempty(current_density)  % assume uniform weight if no current density is provided
               reward = dot(future_density.weight, log(future_density.weight)) + log(length(future_density.weight));
           else
               reward = dot(future_density.weight, log(future_density.weight)) + log(length(future_density.weight)) ...
                        - dot(current_density.weight, log(current_density.weight)) - + log(length(current_density.weight));
           end
       end
       
       % Cauchy-Schwarz inequality
       function reward = cauchy_inequality(obj, current_density, future_density)
           if isempty(current_density)  % assume uniform weight if no current density is provided
               reward = -log((1/future_density.N)/sqrt(1/future_density.N * sum(future_density.weight.^2)));
           else
               reward = -log(dot(current_density.weight, future_density.weight)/ sqrt(dot(current_density.weight.^2, future_density.weight.^2)));
           end
       end
       
       % Task based circle reward
       function reward = circle_reward(obj, current_density, future_density)
           % resampling
           sample_index = Multinomial_resampling(future_density.weight, future_density.N);
           particles = future_density.particles(:, sample_index);
           center = mean(particles(1:2, :), 2);
           reward = mean(sqrt(sum((particles(1:2, :) - center).^2)) <= obj.config.Circle_radius);
       end
       
       
       
       %% =================== Other Utilities ============================
       % calculate distance to selected density
       function d = get_distance(obj, uav, density)
           % calculate mean state
           mean_x = cell2mat(cellfun(@(x) mean(x.particles(1:2,:), 2), density, 'UniformOutput', false)');
           d = sqrt(sum((mean_x - uav(1:2)).^2));
       end
       
       
       % check if void  constrains are met
       function valid_void = check_void(obj, waypoints, density)
            void_pr = Void_Probability(density, waypoints, obj.config.Void_r);
            valid_void = (void_pr >= obj.config.Void_th);
       end
       
       
   end
          
end