%% Main problem for SITL
clear; close all;


%% Global configuration
Total_time = 2000;                      % maximun simulation time

plot_result = true;                     % whether to plot particles
plot_interval = 1;                      % plotting interval
ntarget = 1;                            % number of targets

uav_altitude = 50;                      % desire uav operation altitude (AGL)

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

% initialize each modules
% =========== [System modules] ===============
target_model = Target_Model(config.target_config);
AoA_model = AoA_Model(config.AoA_sensor_config);
UAV_Action = UAV_Actions(config.uav_config, config.area_config);

Terminate = Termination_Condition(config.Term_config);
AoA_Filter = Bernoulli_Filter(config.AoA_filter_config, AoA_model, target_model);
Planner = Planner_greedy_POMDP(config.planner_config, AoA_model, target_model, UAV_Action);

AoA_Generator = Rotation_Bearing(config.AoA_config);

% =========== [Data recording] ====================
recorder = Statistics(ntarget, (1:ntarget)', Total_time, config, 'Simulation', true);
pattern_record = cell(100, 1);
raw_meas_record = cell(100, 1);

% =========== [Actio checking] ====================
Action_check = Action_Checker(config.uav_config);

%% ============== Plotting setting ==================
if plot_result
    plot_tool = Result_plot( ...
        config.target_config.Ntarget,...
        'Plot3D',           false,...
        'Plot_Void',        config.planner_config.Use_Void, ...
        'Void_r',           config.planner_config.Void_r, ...
        'DEM',              config.area_config.DEM.DATA, ...
        'Area',             config.area_config.area, ...
        'PlotTermination',  true, ...
        'Term_th',          config.Term_config.threshold ...
    );
end

% calculate truth
truth = nan(3, ntarget, 1);
for n = 1:ntarget
   truth(1:2, n, 1) = GPS_Coord.GPS2Cart([config.area_config.DEM.ref.lat; config.area_config.DEM.ref.lon], truth_gps(:, n));
   truth(3, n, 1) = config.area_config.DEM.DATA(round(truth(1, n, 1)), round(truth(2,n,1)));
end


% initialize filter density
density = cell(config.target_config.Ntarget, 1);
for i = 1:config.target_config.Ntarget
    density{i} = Bernoulli(gen_birth_particles_polygon(config.area_config.DEM.DATA, 10000, config.area_config.area, config.area_config.Alt_range), 0.01, i);
end


move_time = config.planner_config.T - config.uav_config.rotation_time;

% generate path
path = Generate_waypoint_pattern(config.area_config.area, rotation_time+10, 20, 'WE', Action_Type.Combine, 20);


%% ====================== SITL initialization ============================
comm = UAV_Comm('filter_config_udoo.json', 'Debug', true);


%% ======================= Main Program =============================
main_start_time = tic;
proc_time = 1;

% get home position & initialize state
pause(0.5);
home = comm.get_home_pos();
% initialize initial position
uav0 = GPS_Coord.GPS2Cart_struct(config.area_config.DEM.ref, home);
uav0(3) = uav0(3) + config.area_config.DEM.DATA(round(uav0(1)), round(uav0(2)));
warning('The elevation in uav state is relative elevation, not AMSL.');


% ======================== Main Loop ================================
t = 0;
meas_count = 0;
planning_cycle = 0; % planning cycle count
move_fin = false;
rotate_fin = false;
while (t < Total_time) && action_index < length(path)
    fprintf('Epos: %i/%i.\n', t, Total_time);
    loop_time = tic;
    t = t+1;    % increment simulation time
     
    % ============= update uav location ========================
    current_gps = comm.get_gps();
    if ~isempty(current_gps)
        uav(:,t) = GPS_Coord.GPS2Cart_struct(config.area_config.DEM.ref, current_gps(end, :));
        uav(3,t) = uav(3,t) + config.area_config.DEM.DATA(round(uav(1,t)), round(uav(2,t)));
    end

    
    % check action
     previous_move_fin = move_fin;
     move_fin = Action_check.move_finish(uav(:, t), t);

     if previous_move_fin && move_fin
         rotate_fin = Action_check.rotate_finish(uav(:, t), t);
     else
         rotate_fin = false;
     end
    
    % ============= generate measurement =======================
    pulses = comm.get_pulses();
    disp(pulses);
    z = convert_pulses(pulses, config.area_config.DEM.ref);
  
    
    % record data for bearing
    AoA_Generator.record_data(z, t);
    
    % ===================== filtering ==========================
    % filter prediction
    for n = 1:length(density)
        density{n} = AoA_Filter.predict(density{n});
    end
    
    if move_fin && rotate_fin
        % get bearing measurement
        [bearing, pattern, raw_meas] = AoA_Generator.get_bearing();
        meas_count = meas_count + 1;
        pattern_record{meas_count} = pattern;
        raw_meas_record{meas_count} = raw_meas;
        for n = 1:length(density)
            % extract ID & RSSI from measurement table z
            meas = table2array(bearing(bearing.Target_ID == n, :));
            density{n} = AoA_Filter.filtering(density{n}, meas, uav(:, t));
        end
    end
    
    % ====================== planning ==========================
    if (move_fin && rotate_fin) || action_index == 0
        selected_action = path{action_index};
        action_index = action_index + 1;  
       
        
        % init bearing action
        AoA_Generator.check_start_condition(selected_action, t);
        
        % save selected action
        recorder.record_action(uav(:,t), selected_action, action_index, t, 0);
        
        % issue the move command
        waypoint = GPS_Coord.Cart2GPS_struct(config.area_config.DEM.ref, selected_action.waypoints(:, move_time));
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
    if move_fin && ~rotate_fin
        comm.rotate();
    end
    

    
    % =============== check termination codition ================
    [density, all_target_found, found_report] = Terminate.termination_check(density, t);
        
    
    % ====================== Collecting data ====================
    [est] = recorder.record(z, density, found_report, t);
    
    
    % ==================== Time ticking ====================
    proc_time = toc(main_start_time);
    
    
    % ====================== update plot =========================
    if (plot_result && mod(t, plot_interval) == 0) || (t == 1 && plot_result)
        plot_tool.update_plot(density, uav(:, 1:t), truth, est, selected_target);
        plot_tool.update_error_plot(recorder.stats.error_history, t);
        plot_tool.update_termination_value(term_val);
    end
    
    
    % wait until looptime reach 1 second
    while toc(loop_time) < 1
        pause(0.05);
    end
    
end

% send UAV back to home position
% adjust heading
home.hdg = rad2deg(wrapTo2Pi(atan2(uav0(1) - uav(1,t), uav0(2) - uav(2,t))));
comm.move(home);

% close tcp connection to mavlink-proxy
comm.close_tcp();


% save uav path
recorder.record_uav_path(uav);
% cleanup recorded data
recorder.cleanup(t, action_index);

%% save result
filename = strcat('Victor_results/', 'AoA_fixedpath_test-', datestr(now,'mm-dd-HH_MM'), '.mat');
save(filename);


% finishing
% ========================================================
if plot_result
    % plot return path
    plot_tool.update_uav_return(uav(:, t), uav0);
end

%% ====================================================================

% % report error
% recorder.get_error();
% recorder.report_result();