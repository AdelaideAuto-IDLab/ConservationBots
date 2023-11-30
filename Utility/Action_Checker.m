classdef Action_Checker < handle
    properties
         config;
         action = [];
         distance_threshold = 3;
         angle_threshold = deg2rad(5);
         speed_threshold = 0.3;
    end
    
    properties 
        rotate_start_t = nan;
    end
    
    
    methods
        function obj = Action_Checker(uav_config)
            obj.config = uav_config;
        end
        
        function register_action(obj, action)
            obj.action = action; 
        end
         
        
        function [finish] = move_finish(obj, uav, gps)
            if nargin == 3
                if ~isempty(gps)
                    speed = sqrt(gps.vx^2 + gps.vy^2);
                else
                    speed = 0;
                end
            elseif nargin == 2
                speed = 0;
            end
            
            if ~isempty(obj.action)
                switch obj.action.type
                    case Action_Type.RSSI
                         d = sqrt(sum((uav(1:2) - obj.action.waypoints(1:2, end)).^2));
                         if d <= obj.distance_threshold && speed < obj.speed_threshold
                             finish = true;
                         else
                             finish = false;
                         end
                    case Action_Type.AoA
                         t = obj.config.traject_time - obj.config.rotation_time;    % move time
                         d = sqrt(sum((uav(1:2) - obj.action.waypoints(1:2, t)).^2));
                         if d <= obj.distance_threshold && speed < obj.speed_threshold
                             finish = true;
                         else
                             finish = false;
                         end
                    case Action_Type.Combine
                         t = obj.config.traject_time - obj.config.rotation_time;    % move time
                         d = sqrt(sum((uav(1:2) - obj.action.waypoints(1:2, t)).^2));
                         if d <= obj.distance_threshold && speed < obj.speed_threshold
                             finish = true;
                         else
                             finish = false;
                         end
                    otherwise
                        error('Error: Unknown action type.');
                end
            else
                finish = false;
            end
            
            
        end
        
        function rotate_start_time(obj, t)
            obj.rotate_start_t = t;
        end
        
        
        function [finish] = rotate_finish(obj, uav, t)
            if ~isempty(obj.action)
                switch obj.action.type
                    case Action_Type.RSSI
                        finish = false;
                    case Action_Type.AoA 
                         if ~isnan(obj.rotate_start_t)
                             finish = (t - obj.rotate_start_t)>1.1*obj.config.rotation_time || ((t - obj.rotate_start_t)> 0.9*obj.config.rotation_time && abs(angdiff(uav(4), obj.action.waypoints(4,end))) <= obj.angle_threshold);
                         else
                             finish = abs(angdiff(uav(4), obj.action.waypoints(4,end))) <= obj.angle_threshold;
                         end 
                    case Action_Type.Combine
                         if ~isnan(obj.rotate_start_t)
                             finish = (t - obj.rotate_start_t)>1.1*obj.config.rotation_time || ((t - obj.rotate_start_t)> 0.9*obj.config.rotation_time && abs(angdiff(uav(4), obj.action.waypoints(4,end))) <= obj.angle_threshold);
                         else
                             finish = abs(angdiff(uav(4), obj.action.waypoints(4,end))) <= obj.angle_threshold;
                         end 
                    otherwise
                        error('Error: Unknown action type.');
                end 
            else
                finish = false;
            end
            
        end
        
    end
    
    methods (Access = private)
    end
    
end