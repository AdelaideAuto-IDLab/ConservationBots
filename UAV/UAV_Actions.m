classdef UAV_Actions < handle
    properties
      Uav_config; 
      Area_config;    
    end
    
    properties
        heading_base; 
    end


    methods
        function obj = UAV_Actions(uav_config, area_config)
            obj.Uav_config = uav_config;
            obj.Area_config = area_config;
            
            %% Note: uav must has equal accelration and deaccelration
            if abs(obj.Uav_config.accel) ~= abs(obj.Uav_config.decel)
                error('Config Error: non-equal accelration and deaccelration.\n');
            end
            
            if obj.Uav_config.traject_time < pi/obj.Uav_config.turn_rate
                error('Config Error: Unable to turn 180 degree in given trajectory time.\n');
            end
            
        end
        
        % calculate next series of waypoint given current uav state and heading
        % find the maximum distance
        % Note: theta is absolute angle
        % output empty set if action violated boundary
        function waypoints = get_action_md(obj, uav, theta, vmax)
            waypoints = obj.move_straight(uav, theta, obj.Uav_config.traject_time, vmax);
        end
        
        % get set of actions given vector of heading
        function action_set = get_action_set_md(obj, uav, theta)
            action_set = cell(1*length(theta), 1);
            for i = 1:length(theta)
               action_set{i} = obj.get_action_md(uav, theta(i), 1*obj.Uav_config.vmax); 
%                action_set{2*i} = obj.get_action_md(uav, theta(i), 0.66*obj.Uav_config.vmax); 
%                action_set{3*i} = obj.get_action_md(uav, theta(i), 0.33*obj.Uav_config.vmax); 
            end     
            
            % remove empty cells
            action_set = action_set(~cellfun('isempty', action_set));
            
            if sum(~cellfun(@isempty, action_set)) == 0
                error('Error: no valid actions.\n');
            end
        end
              
        % calculate waypoints given starting and stopping points
        function waypoints = get_action_pp(obj, uav, end_point)
            %% check if given waypoint is reachable within [traject_time]
            theta = atan2(end_point(1) - uav(1), end_point(2) - uav(2));
            relative_coord = end_point(1:2) - uav(1:2);
            max_d = obj.check_max_distance(uav, theta, obj.Uav_config.vmax);
            end_d = sqrt(sum(relative_coord.^2));
            
            travel_distance = min(end_d, max_d);
            
            
            %% calculate waypoints
            turn_angle = angdiff(uav(4), theta);
            t_turn = abs(turn_angle)/obj.Uav_config.turn_rate;  
            
            % remaining time to move
            remain_time = obj.Uav_config.traject_time - t_turn;   
            
            % calculate acceleration time
            % solve: a*dt^2 - a*T*dt + d = 0
            % a: acceleration, dt: accel time
            % T: remain_time, d: travel_distance
            a = obj.Uav_config.accel;
            T = obj.Uav_config.traject_time;
            
            t_accel = ((a*T) - sqrt((a*T).^2 - 4*a*travel_distance))/(2*a);
            
            % constant velocity time
            t_cv = remain_time - 2*t_accel;
            
            % calculate waypoints
            waypoints = obj.sampling_waypoint(uav, turn_angle, t_turn, t_accel, t_cv, obj.Uav_config.traject_time);
            
            % [TODO]: check boundary condition
            
        end
        
        % get rotation waypoints
        function waypoints = get_action_rotation(obj, uav, omega)
            waypoints =  repmat(uav, 1, obj.Uav_config.rotation_time);
            
            % limit omega to achievable turn angle
            maximum_angle = obj.Uav_config.turn_rate * obj.Uav_config.rotation_time;
            omega = sign(omega) * min(maximum_angle, abs(omega));
            
            headings = linspace(uav(4), uav(4) + omega, obj.Uav_config.rotation_time+1);
            headings = headings(2:end);
            
            waypoints(4,:) = wrapTo2Pi(headings);  
        end
        
        % get composit waypoints (move & rotation)
        function waypoints = get_composit_waypoints(obj, uav, theta, vmax)
              waypoint1 = obj.move_straight(uav, theta, obj.Uav_config.traject_time - obj.Uav_config.rotation_time, vmax);
              if ~isempty(waypoint1)
                  waypoint2 = obj.get_action_rotation(waypoint1(:, end), 2*pi);
                  waypoints = [waypoint1, waypoint2];
              else
                  waypoints = [];
              end
        end
        
        % get set of composit waypoints
        function action_set = get_composit_waypoints_set(obj, uav, theta)
            action_set = cell(1*length(theta), 1);
            for i = 1:length(theta)
               action_set{i} = obj.get_composit_waypoints(uav, theta(i), 1*obj.Uav_config.vmax); 
            end   
            
            % remove empty cells
            action_set = action_set(~cellfun('isempty', action_set));
            
            if sum(~cellfun(@isempty, action_set)) == 0
                error('Error: no valid actions.\n');
            end
        end
            
    end
    
    
    methods (Access=private)
        % check the maximum distance the uav can travel in direction theta
        function max_d = check_max_distance(obj, uav, theta, vmax)
            % calculate rotation time
            turn_angle = angdiff(uav(4), theta);
            t_turn = abs(turn_angle)/obj.Uav_config.turn_rate;        
            
            % maximum acceleration time
            ta_max = vmax/obj.Uav_config.accel;
            
            % remaining time to move
            remain_time = obj.Uav_config.traject_time - t_turn;         
            % accerelation/deaccelration time
            t_accel = min(ta_max, obj.Uav_config.traject_time/2);
            % constant velocity time
            t_cv = remain_time - 2*t_accel;
            
            max_d = obj.Uav_config.accel*t_accel + obj.Uav_config.accel*t_accel*t_cv;
        end
           
        % calculate UAV moving in straight line waypoints
        function waypoints = move_straight(obj, uav, theta, move_time, vmax)
            % calculate rotation time
            turn_angle = angdiff(uav(4), theta);
            t_turn = abs(turn_angle)/obj.Uav_config.turn_rate;   
            
            % maximum acceleration time
            ta_max = vmax/obj.Uav_config.accel;
            
            % remaining time to move
            remain_time = move_time - t_turn;         
            % accerelation/deaccelration time
            t_accel = min(ta_max, move_time/2);
            % constant velocity time
            t_cv = remain_time - 2*t_accel;
            
            % calculate waypoints
            waypoints = obj.sampling_waypoint(uav, turn_angle, t_turn, t_accel, t_cv, move_time);
            
            % check boundary condition
            inbound = inpolygon(waypoints(1,:), waypoints(2,:), obj.Area_config.area(1, :), obj.Area_config.area(2,:));
%             invalid_idx = find(~inbound, 1, 'first');
            if sum(~inbound) > 0
                waypoints = [];
            end
            
%             if ~isempty(invalid_idx) && invalid_idx ~= 1
%                 waypoints(:, invalid_idx:end) = repmat(waypoints(:, invalid_idx-1), 1, length(inbound)-invalid_idx+1);
%             else
%                 waypoints = [];
%             end
        end
        
        % sampling uav waypoints at discrete time
        function waypoints = sampling_waypoint(obj, uav, turn_angle, t_turn, t_accel, t_cv, move_time)
            % initialization
            waypoints = repmat(uav, 1, move_time);
            
            % construct time_stamp
            time_stamp = cumsum([0, t_turn, t_accel, t_cv, t_accel])'; 
            
            % calculate travel distance at each time stamp
            d = cumsum([0, 0, 0.5*obj.Uav_config.accel*t_accel, obj.Uav_config.accel*t_accel*t_cv, 0.5*obj.Uav_config.accel*t_accel])';
            
            % sample distance at discrete time steps
            d_i = interp1q(time_stamp, d, (0:move_time)');
            d_i = d_i(2:end)';  % discard the first data (starting point)
            
            
            %% update waypoints
            % update headings
            if t_turn ~= 0
                waypoints(4, 1:floor(t_turn)) = waypoints(4, 1:floor(t_turn)) + ...
                                         sign(turn_angle)*obj.Uav_config.turn_rate * (1:floor(t_turn));
                waypoints(4, ceil(t_turn):end) = uav(4) + turn_angle;
            end
            
            % limit heading to 0, 2*pi
            waypoints(4,:) = wrapTo2Pi(waypoints(4, :));
            
            % update distance
            waypoints(1,:) = waypoints(1,:) + d_i .* sin(uav(4) + turn_angle);
            waypoints(2,:) = waypoints(2,:) + d_i .* cos(uav(4) + turn_angle);     
        end
        
    end
    
    
end