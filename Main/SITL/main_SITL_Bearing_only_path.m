%% Main problem for SITL
% clc; clear; close all;


%% Global configuration
Total_time = 1000;                      % maximun simulation time

plot_result = true;                     % whether to plot particles
plot_interval = 1;                      % plotting interval
ntarget = 1;                            % number of targets

uav_altitude = 10;                      % desire uav operation altitude (AGL)

reward = Reward_Type.Shannon;
Use_Void = false;
rotation_time = 20;
%% Useful intermedia variable
uav = nan(4, Total_time);
selected_action = [];               % planner output
selected_target = nan;              % planner selected target
action_index = 0;                   % the nth action
all_target_found = false;           % true if all targets are found 

%% Initialization
addpath(genpath(pwd));

% generate module settings
config = Experiment_config_wrapper_Victor(ntarget, reward, Use_Void, rotation_time);


% =========== [Actio checking] ====================
Action_check = Action_Checker(config.uav_config);

%% ============== Plotting setting ==================
if plot_result
    plot_tool = Result_plot( ...
        config.target_config.Ntarget,...
        'Plot3D',           false,...
        'Plot_Void',        config.planner_config.Use_Void, ...
        'Void_r',           config.planner_config.Void_r, ...
        'DEM',              config.area_config.DEM.DATA ...
    );
end



move_time = config.planner_config.T - config.uav_config.rotation_time;

% generate path
path = Generate_waypoint_pattern(config.area_config.area, rotation_time+10, 20, 'WE', Action_Type.Combine, 20);


%% ====================== SITL initialization ============================
comm = UAV_Comm('filter_config_udoo.json', 'Debug', false);
comm.request_gps_sitl(5);

%% ======================= Main Program =============================
main_start_time = tic;
proc_time = 1;

% get home position & initialize state
pause(0.5);
home = comm.get_home_pos();
% initialize initial position
uav0 = GPS_Coord.GPS2Cart_struct(config.area_config.DEM.ref, home);
warning('The elevation in uav state is relative elevation, not AMSL.');


% ======================== Main Loop ================================
t = 0;
waypoint_count = 1;
move_fin = true;
rotate_fin = false;
rotate_started = false;
while (t < Total_time) && ~all_target_found
    fprintf('Epos: %i/%i.\n', t, Total_time);
    loop_time = tic;
    t = t+1;    % increment simulation time
     
    % ============= update uav location ========================
    current_gps = comm.get_gps();
    if ~isempty(current_gps)
        uav(:,t) = GPS_Coord.GPS2Cart_struct(config.area_config.DEM.ref, current_gps);
        uav(3,t) = uav(3,t) + config.area_config.DEM.DATA(round(uav(1,t)), round(uav(2,t)));
    end
    
    % check action
    move_fin = Action_check.move_finish(uav(:, t));

    if  move_fin && selected_action.type == Action_Type.Combine
        rotate_fin = Action_check.rotate_finish(uav(:, t)) & rotate_started;
    else
        rotate_fin = true;
    end
    
    
    if move_fin && rotate_fin && selected_action.type == Action_Type.Combine
        rotate_started = false;
    end
    
    % ====================== planning ==========================
    if (move_fin && rotate_fin) || waypoint_count == 1
        selected_action = path{waypoint_count};
        waypoint_count = waypoint_count + 1;
        
        % issue the move command
        if selected_action.type == Action_Type.Combine || selected_action.type == Action_Type.AoA
            waypoint = GPS_Coord.Cart2GPS_struct(config.area_config.DEM.ref, selected_action.waypoints(:, move_time));
        elseif selected_action.type == Action_Type.RSSI
            waypoint = GPS_Coord.Cart2GPS_struct(config.area_config.DEM.ref, selected_action.waypoints(:, end));
        end
        
        % update waypoint alt to relative altitude
        waypoint.relative_alt = uav_altitude;
        comm.move(waypoint);
        
        % reset rotation state
        comm.rotate_reset();
        
       % register action
       Action_check.register_action(selected_action);
       move_fin = false;
       rotate_fin = false;
    end
    
    % issue rotate command 
    if move_fin && ~rotate_fin && selected_action.type == Action_Type.Combine
        rotate_started = true;
        comm.turn_once();
    end
    

    % ==================== Time ticking ====================
    proc_time = toc(main_start_time);
    
    
    % ====================== update plot =========================
    if (plot_result && mod(t, plot_interval) == 0) || t == 1
        plot_tool.update_uav(uav(:, 1:t));
    end
    
    
    % wait until looptime reach 1 second
    while toc(loop_time) < 1
        pause(0.05);
    end
    
end


% finishing
% ========================================================
if plot_result
    % plot return path
    plot_tool.update_uav_return(uav(:, t), uav0);
end




% send UAV back to home position
% adjust heading
home.hdg = rad2deg(wrapTo2Pi(atan2(uav0(1) - uav(1,t), uav0(2) - uav(2,t))));
comm.move(home);

% close tcp connection to mavlink-proxy
comm.close_tcp();

