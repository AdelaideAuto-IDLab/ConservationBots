classdef UAV_Comm < handle
    %% UAV communication module
    properties
        tcp_connection;
        Mode;
        DEBUG;
        config;
        
        Reader;     % gps & pulse reader
    end
	
	properties (Access  = private)
		% properties for achieving rotation
		rotate_start = false;
		rotate_step = 0;
		init_heading;
	end
    
    methods
        function obj = UAV_Comm(config_filename, varargin)
            p = inputParser;
            addParameter(p, 'Debug', false, @islogical);
            addParameter(p, 'Config', []);
            parse(p, varargin{:});
            
            % read config file
            config = read_json(config_filename);
            obj.config = config;
            
            obj.DEBUG = p.Results.Debug;    % debug message
            
            
            % initialize gps & pulse reader
            obj.Reader = GPS_Pulse_Reader(config.TCP.log_file, config.TCP.pulse_log, config.TCP.gps_log, ...
                                          'SNR_threshold', config.Threshold.SNR, ...
                                          'RSSI_threshold', config.Threshold.RSSI);
            
            % estabalish TCP connection
            obj.tcp_connection=tcpclient('127.0.0.1',config.TCP.port);
            fopen(obj.tcp_connection);
        end
        
        % move UAV to specified GPS coordinate
        function move(obj, location)
            % construct message
            msg.command = 16;   % MAV_CMD_NAV_WAYPOINT
            msg.param1 = 0;
            msg.param2 = 0;
            msg.param3 = 0;
            msg.param4 = 0;
            msg.x = location.lat*1e7;
            msg.y = location.lon*1e7;
            msg.z = location.relative_alt;
            json_send_msg = obj.generate_generic_mission(msg);
            obj.write_to_tcp(json_send_msg);
            
            if obj.DEBUG
                fprintf('%s\n', json_send_msg);
            end
        end
        
         % move UAV to specified relative to home location
        function move_local(obj, location)  
            % location format:
            % .x
            % .y
            % .z
            % .vx
            % .vy
            % .vz
            % .yaw
            % .yaw_rate
            json_send_msg = obj.set_position_target_local_ned(location);
            obj.write_to_tcp(json_send_msg);
            
            if obj.DEBUG
                fprintf('%s\n', json_send_msg);
            end
        end
        
        function move_global(obj, location)
            msg.lat = location.lat;
            msg.lon = location.lon;
            msg.vx = 0;
            msg.vy = 0;
            msg.vz = 0;
            msg.yaw = deg2rad(location.hdg);
            msg.yaw_rate = 0;
            msg.relative_alt = location.relative_alt;
            
            json_send_msg = obj.set_position_target_global_int(msg);
            obj.write_to_tcp(json_send_msg);
            
            if obj.DEBUG
                fprintf('%s\n', json_send_msg);
            end
        end
        
        function move_rotate(obj, location, yaw_rate)
            send_msg.SET_POSITION_TARGET_GLOBAL_INT.time_boot_ms = 0;        % time since boot
            send_msg.SET_POSITION_TARGET_GLOBAL_INT.target_system = 0;       % broadcast
            send_msg.SET_POSITION_TARGET_GLOBAL_INT.target_component = 0;    % broadcast
            send_msg.SET_POSITION_TARGET_GLOBAL_INT.coordinate_frame = 3;% MAV_FRAME_GLOBAL_RELATIVE_ALT;    %
            send_msg.SET_POSITION_TARGET_GLOBAL_INT.type_mask = 0b010111111000;%POS_MASK 1528;    %
            send_msg.SET_POSITION_TARGET_GLOBAL_INT.lat_int = round(location.lat*1e7); % latitude
            send_msg.SET_POSITION_TARGET_GLOBAL_INT.lon_int = round(location.lon*1e7); % lontitude
            send_msg.SET_POSITION_TARGET_GLOBAL_INT.alt = location.relative_alt;       % altitude
            send_msg.SET_POSITION_TARGET_GLOBAL_INT.vx = 0;               
            send_msg.SET_POSITION_TARGET_GLOBAL_INT.vy = 0;
            send_msg.SET_POSITION_TARGET_GLOBAL_INT.vz = 0;
            send_msg.SET_POSITION_TARGET_GLOBAL_INT.afx = 0;
            send_msg.SET_POSITION_TARGET_GLOBAL_INT.afy = 0;
            send_msg.SET_POSITION_TARGET_GLOBAL_INT.afz = 0;
            send_msg.SET_POSITION_TARGET_GLOBAL_INT.yaw = 0;
            send_msg.SET_POSITION_TARGET_GLOBAL_INT.yaw_rate = yaw_rate;
            json_msg = jsonencode(send_msg);
            
            obj.write_to_tcp(json_msg);
            
            if obj.DEBUG
                fprintf('%s\n', json_msg);
            end
        end
            
        function request_gps_sitl(obj, rate)
            msg.command = 511; % MAV_CMD_SET_MESSAGE_INTERVAL
            msg.confirmation = 0;
            msg.param1 = 33;   % GLOBAL_POSITION_INT
            msg.param2 = round(1/rate *1e6);   % interval in us
            msg.param3 = 0;
            msg.param4 = 0;
            msg.param5 = 0;
            msg.param6 = 0;
            msg.param7 = 0;        % target message stream. 0: default, 1: requestor, 2: broadcast
            json_send_msg = obj.generate_generic_commandlong(msg);
            obj.write_to_tcp(json_send_msg);

            if obj.DEBUG
                fprintf('%s\n', json_send_msg);
            end
        end
        
        % turn UAV
        function yaw(obj, theta)
            msg.command = 115; % MAV_CMD_CONDITION_YAW 
            msg.confirmation = 0;
            msg.param1 = theta;
            msg.param2 = 0;
            msg.param3 = 0;
            msg.param4 = 0;
            msg.param5 = 0;
            msg.param6 = 0;
            msg.param7 = 0;
            json_send_msg = obj.generate_generic_commandlong(msg);
            obj.write_to_tcp(json_send_msg);    
            
            if obj.DEBUG
                fprintf('%s\n', json_send_msg);
            end
        end
        
        function RTL(obj)
            msg.command = 20; % MAV_CMD_NAV_RETURN_TO_LAUNCH
            msg.confirmation = 0;
            msg.param1 = 0;
            msg.param2 = 0;
            msg.param3 = 0;
            msg.param4 = 0;
            msg.param5 = 0;
            msg.param6 = 0;
            msg.param7 = 0;
            json_send_msg = obj.generate_generic_commandlong(msg);
            obj.write_to_tcp(json_send_msg);    
            
            if obj.DEBUG
                fprintf('%s\n', json_send_msg);
            end
        end
        
        function land(obj)
            msg.command = 21; % MAV_CMD_NAV_LAND
            msg.confirmation = 0;
            msg.param1 = 0;
            msg.param2 = 0;
            msg.param3 = 0;
            msg.param4 = 0;
            msg.param5 = 0;
            msg.param6 = 0;
            msg.param7 = 0;
            json_send_msg = obj.generate_generic_commandlong(msg);
            obj.write_to_tcp(json_send_msg);    
            
            if obj.DEBUG
                fprintf('%s\n', json_send_msg);
            end
        end
        
        function takeoff(obj, altitude)
            msg.command = 22; % MAV_CMD_NAV_TAKEOFF
            msg.confirmation = 0;
            msg.param1 = 0;
            msg.param2 = 0;
            msg.param3 = 0;
            msg.param4 = 0;
            msg.param5 = 0;
            msg.param6 = 0;
            msg.param7 = min(altitude, 120);
            json_send_msg = obj.generate_generic_commandlong(msg);
            obj.write_to_tcp(json_send_msg);    
            
            if obj.DEBUG
                fprintf('%s\n', json_send_msg);
            end
        end      
        
        function do_change_speed(obj, speed)
            msg.command = 178; % MAV_CMD_DO_CHANGE_SPEED
            msg.confirmation = 0;
            msg.param1 = 0;
            msg.param2 = min(max(0.1, speed), 20);
            msg.param3 = 0;
            msg.param4 = 0;
            msg.param5 = 0;
            msg.param6 = 0;
            msg.param7 = 0;
            json_send_msg = obj.generate_generic_commandlong(msg);
            obj.write_to_tcp(json_send_msg);    
            
            if obj.DEBUG
                fprintf('%s\n', json_send_msg);
            end
        end
                
        % close all connections
        function close_tcp(obj)
            fclose(obj.tcp_connection);
        end    
        
        % set yaw rate (in centidegree)
        function set_yaw_rate(obj, yaw_rate)
            % yaw rate limit to [5 ~ 180] degree/s
            yaw_rate = min(max(500, yaw_rate), 18000);
            Param_ID = 'ATC_SLEW_YAW'; 
            Param_ID(length(Param_ID)+1) = char(0);    % null terminated param id
            json_send_msg = obj.generate_param_set(Param_ID, single(yaw_rate), single(MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32));
            obj.write_to_tcp(json_send_msg);    
            if obj.DEBUG
                fprintf('%s\n', json_send_msg);
            end
        end
        
        % set WPNAV_SPEED
        function set_speed(obj, speed)
            speed = 100*min(max(0.1, speed), 20);  % Unit: cm/s
            Param_ID = 'WPNAV_SPEED';
            Param_ID(length(Param_ID)+1) = char(0);    % null terminated param id
            json_send_msg = obj.generate_param_set(Param_ID, single(speed), single(MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32));
            obj.write_to_tcp(json_send_msg);    
            if obj.DEBUG
                fprintf('%s\n', json_send_msg);
            end
        end
          
        % get home position
        function home = get_home_pos(obj)
            home = obj.Reader.get_home_pos(); 
        end
        
        % get gps
        function gps = get_gps(obj)
            gps = obj.Reader.get_gps();
        end
        
        % get pulses
        function pulses = get_pulses(obj)
            pulses = obj.Reader.get_pulses();
        end
        
        % guided turns 
        function turns(obj, n)
            msg.command = 18;   % MAV_CMD_NAV_LOITER_TURNS
            msg.param1 = n;     % number of turns
            msg.param2 = 0;
            msg.param3 = 0;
            msg.param4 = 0;
            msg.x = 0;          % lat, 0 = use current location
            msg.y = 0;          % lon, 0 = use current location
            msg.z = 0;          % alt, 0 = use current location
            send_msg = obj.generate_generic_mission(msg);
            obj.write_to_tcp(send_msg);
            
            if obj.DEBUG
                fprintf('%s\n', send_msg);
            end
        end
        
        function complete = turn_once(obj)
            complete = false;
            if ~obj.rotate_start
                % get initial heading
                current_gps = obj.get_gps(); 
                while isempty(current_gps)
                    current_gps = obj.get_gps(); 
                    pause(0.05);
                end
                current_gps = current_gps(end, :);
                obj.init_heading = current_gps.hdg;

                obj.turns(1);

                obj.rotate_start = true;
            else
				current_gps = obj.get_gps();
                while isempty(current_gps)
                    current_gps = obj.get_gps();
                    pause(0.05);
                end
                
                if abs(angdiffd(obj.init_heading, current_gps.hdg)) <= 5.0
                    obj.rotate_start = false;
                    complete = true;
                end
            end
        end
		
		% rotate UAV for 360 degree (3 steps of 120 degree turns)
		% return true once rotation is finished
		function complete = rotate(obj) 
			complete = false;
			if ~obj.rotate_start
				obj.rotate_start = true;
				% get initial heading
				current_gps = obj.get_gps(); 
                while isempty(current_gps)
                    current_gps = obj.get_gps(); 
                end
                current_gps = current_gps(end, :);
				obj.init_heading = current_gps.hdg;
				
				% update stage
				obj.rotate_step = 1;
				
				% perform first 120 degree rotation
				obj.yaw(mod(obj.init_heading + 120*obj.rotate_step, 360));
                
                fprintf('Yaw Angle: %i\n',mod(obj.init_heading + 120*obj.rotate_step, 360)); 
                
			else
				% chekc whether previous rotation is finished
				current_gps = obj.get_gps();
                while isempty(current_gps)
                    current_gps = obj.get_gps();
                    pause(0.05);
                end
                
                  if angdiffd(obj.init_heading + 120*obj.rotate_step, current_gps.hdg) >-5
					if obj.rotate_step ~= 3
						% perform next rotation 
						obj.rotate_step = obj.rotate_step + 1;
						obj.yaw(mod(obj.init_heading + 120*obj.rotate_step, 360));
                        
                        fprintf('Yaw Angle: %i\n',mod(obj.init_heading + 120*obj.rotate_step, 360)); 
					else
						% rotation finshed
						complete = true;
						obj.rotate_start = false;
                        obj.rotate_step = 0;
					end		
				end
			end
        end
        
        % reset internal state of rotation ocunter
        function rotate_reset(obj)
            obj.rotate_start = false;
            obj.rotate_step = 0;
        end
		
    end
    
    
    methods %(Access=private)
        % generate generic json message
        function json_msg = generate_generic_commandlong(obj, msg)
            send_msg.COMMAND_LONG.target_system = 0;    % broadcast 
            send_msg.COMMAND_LONG.target_component = 0; % broadcast 
            send_msg.COMMAND_LONG.confirmation = msg.confirmation;
            send_msg.COMMAND_LONG.command = msg.command; 
            send_msg.COMMAND_LONG.param1 = msg.param1;
            send_msg.COMMAND_LONG.param2 = msg.param2;
            send_msg.COMMAND_LONG.param3 = msg.param3;
            send_msg.COMMAND_LONG.param4 = msg.param4;
            send_msg.COMMAND_LONG.param5 = msg.param5;
            send_msg.COMMAND_LONG.param6 = msg.param6;
            send_msg.COMMAND_LONG.param7 = msg.param7;
            json_msg = jsonencode(send_msg);
        end
        
        % generate generic mission item
        function json_msg = generate_generic_mission(obj, msg)    % MISSION_ITEMS_INT 
            send_msg.MISSION_ITEM_INT.target_system = 0;     % broadcast 
            send_msg.MISSION_ITEM_INT.target_component = 0;  % broadcast 
            send_msg.MISSION_ITEM_INT.seq = 0;
            send_msg.MISSION_ITEM_INT.frame = 3;%MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
            send_msg.MISSION_ITEM_INT.command = msg.command; 
            send_msg.MISSION_ITEM_INT.current = 2;
            send_msg.MISSION_ITEM_INT.autocontinue = 0;
            send_msg.MISSION_ITEM_INT.param1 = msg.param1;
            send_msg.MISSION_ITEM_INT.param2 = msg.param2;
            send_msg.MISSION_ITEM_INT.param3 = msg.param3;
            send_msg.MISSION_ITEM_INT.param4 = msg.param4;
            send_msg.MISSION_ITEM_INT.x = msg.x;
            send_msg.MISSION_ITEM_INT.y = msg.y;
            send_msg.MISSION_ITEM_INT.z = msg.z; 
            json_msg = jsonencode(send_msg);
        end
        
        function json_msg = set_position_target_global_int(obj, msg)    % SET_POSITION_TARGET_GLOBAL_INT
            send_msg.SET_POSITION_TARGET_GLOBAL_INT.time_boot_ms = 0;        % time since boot
            send_msg.SET_POSITION_TARGET_GLOBAL_INT.target_system = 0;       % broadcast
            send_msg.SET_POSITION_TARGET_GLOBAL_INT.target_component = 0;    % broadcast
            send_msg.SET_POSITION_TARGET_GLOBAL_INT.coordinate_frame = 3;% MAV_FRAME_GLOBAL_RELATIVE_ALT;    %
            send_msg.SET_POSITION_TARGET_GLOBAL_INT.type_mask = 0b100111111000;%POS_MASK 3576;    %
            send_msg.SET_POSITION_TARGET_GLOBAL_INT.lat_int = round(msg.lat*1e7); % latitude
            send_msg.SET_POSITION_TARGET_GLOBAL_INT.lon_int = round(msg.lon*1e7); % lontitude
            send_msg.SET_POSITION_TARGET_GLOBAL_INT.alt = msg.relative_alt;       % altitude
            send_msg.SET_POSITION_TARGET_GLOBAL_INT.vx = msg.vx;               
            send_msg.SET_POSITION_TARGET_GLOBAL_INT.vy = msg.vy;
            send_msg.SET_POSITION_TARGET_GLOBAL_INT.vz = msg.vz;
            send_msg.SET_POSITION_TARGET_GLOBAL_INT.afx = 0;
            send_msg.SET_POSITION_TARGET_GLOBAL_INT.afy = 0;
            send_msg.SET_POSITION_TARGET_GLOBAL_INT.afz = 0;
            send_msg.SET_POSITION_TARGET_GLOBAL_INT.yaw = msg.yaw;
            send_msg.SET_POSITION_TARGET_GLOBAL_INT.yaw_rate = msg.yaw_rate;
            json_msg = jsonencode(send_msg);
        end
        
        function json_msg = set_position_target_local_ned(obj, msg)    % SET_POSITION_TARGET_LOCAL_NED
            send_msg.SET_POSITION_TARGET_LOCAL_NED.time_boot_ms = 0;        % time since boot
            send_msg.SET_POSITION_TARGET_LOCAL_NED.target_system = 0;       % broadcast
            send_msg.SET_POSITION_TARGET_LOCAL_NED.target_component = 0;    % broadcast
            send_msg.SET_POSITION_TARGET_LOCAL_NED.coordinate_frame = 1; %MAV_FRAME_LOCAL_NED;    %
            send_msg.SET_POSITION_TARGET_LOCAL_NED.type_mask = 2552;%NAV_TYPE_MASK.POS_MASK;    %
            send_msg.SET_POSITION_TARGET_LOCAL_NED.x = msg.x; 
            send_msg.SET_POSITION_TARGET_LOCAL_NED.y = msg.y; 
            send_msg.SET_POSITION_TARGET_LOCAL_NED.z = -msg.z;  
            send_msg.SET_POSITION_TARGET_LOCAL_NED.vx = msg.vx;               
            send_msg.SET_POSITION_TARGET_LOCAL_NED.vy = msg.vy;
            send_msg.SET_POSITION_TARGET_LOCAL_NED.vz = msg.vz;
            send_msg.SET_POSITION_TARGET_LOCAL_NED.afx = 0;
            send_msg.SET_POSITION_TARGET_LOCAL_NED.afy = 0;
            send_msg.SET_POSITION_TARGET_LOCAL_NED.afz = 0;
            send_msg.SET_POSITION_TARGET_LOCAL_NED.yaw = msg.yaw;
            send_msg.SET_POSITION_TARGET_LOCAL_NED.yaw_rate = msg.yaw_rate;
            json_msg = jsonencode(send_msg);
        end
        
        % update param
        function json_msg = generate_param_set(obj, param_id, param_value, param_type)
            send_msg.PARAM_SET.target_system = 0;    % broadcast
            send_msg.PARAM_SET.target_component = 0;  % broadcast 
            send_msg.PARAM_SET.param_id = param_id;
            send_msg.PARAM_SET.param_value = param_value;
            send_msg.PARAM_SET.param_type = param_type;
            json_msg = jsonencode(send_msg);  
        end
        
        % write json message via TCP
        function write_to_tcp(obj, json_input)
            json_out = jsonencode(jsondecode(json_input));
            json_bytes = unicode2native(json_out);
            json_length = uint8(mod(length(json_out),256));
            json_length_2 = uint8(floor(length(json_out)/256));
            json_command_bytes = [json_length,json_length_2,json_bytes];
            
            fwrite(obj.tcp_connection,json_command_bytes,'uint8'); 
        end
          
        
    end
    
end