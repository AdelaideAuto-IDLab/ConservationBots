%% Main problem for SITL
clear; close all;


%% Global configuration
start_date = datetime();
Total_time = 2000;                      % maximun simulation time

plot_result = true;                     % whether to plot particles
plot_interval = 1;                      % plotting interval
ntarget = 4;                            % number of targets

term_val = nan(ntarget, Total_time);

ID_list = [1,2,3,4];      % 4 tag detector
truth_gps_total = [-35.455104, 138.516823; -35.455142, 138.514649; -35.455736, 138.51928; -35.45726, 138.517211]'; 
wanted_ID_list = [1,2,3,4];

truth_gps = nan(2, length(wanted_ID_list));
for n = 1:length(wanted_ID_list)
    truth_gps(:, n) = truth_gps_total(:, wanted_ID_list(n)==ID_list);
end

uav_altitude = 50;                      % desire uav operation altitude (AGL)

reward = Reward_Type.Renyi;
Use_Void = true;
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
target_id_list = config.target_id_list;

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
recorder = Statistics(ntarget, config.target_id_list', Total_time, config, 'simulation', true);
max_bearing_length = 100;
pattern_record = cell(max_bearing_length, 1);
raw_meas_record = cell(max_bearing_length, 1);
cycle_record = cell(max_bearing_length, 1);
bearing_record = cell(max_bearing_length, 1);
bearing_truth = nan(ntarget, max_bearing_length);
bearing_start_end = nan(2, max_bearing_length);

% =========== [Actio checking] ====================
Action_check = Action_Checker(config.uav_config);

%% ============== Plotting setting ==================
if plot_result
    plot_tool = Result_plot_experiment( ...
        config.target_config.Ntarget,...
        'Plot3D',           false,...
        'Plot_Void',        config.planner_config.Use_Void, ...
        'Void_r',           config.planner_config.Void_r, ...
        'DEM',              config.area_config.DEM.DATA, ...
        'Area',             config.area_config.area, ...
        'PlotTermination',  true, ...
        'Term_th',          config.Term_config.threshold(1), ...
        'Keyboard_Control', true ...
    );
end

try
% calculate truth
truth = nan(3, ntarget, 1);
for n = 1:ntarget
   truth(1:2, n, 1) = GPS_Coord.GPS2Cart([config.area_config.DEM.ref.lat; config.area_config.DEM.ref.lon], truth_gps(:, config.target_id_list(n)));
   truth(3, n, 1) = config.area_config.DEM.DATA(round(truth(1, n, 1)), round(truth(2,n,1)));
end
recorder.stats.truth = truth;
catch e
end

% initialize filter density
density = cell(config.target_config.Ntarget, 1);
for i = 1:config.target_config.Ntarget
    density{i} = Bernoulli(gen_birth_particles_polygon(config.area_config.DEM.DATA, 10000, config.area_config.area, config.area_config.Alt_range), 0.01, config.target_id_list(i));
end


move_time = config.planner_config.T - config.uav_config.rotation_time;



%% ====================== SITL initialization ============================
comm = UAV_Comm('filter_config_udoo.json', 'Debug', true);


%% ======================= Main Program =============================


% get home position & initialize state
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
rotate_started = false;

pause(0.5);
main_start_time = tic;
proc_time = 1;
while (t < Total_time) && ~all_target_found
    fprintf('Epochs: %i/%i.\n', t, Total_time);
    loop_time = tic;
    t = t+1;    % increment simulation time
     
    % ============= update uav location ========================
    current_gps = comm.get_gps();
    if ~isempty(current_gps)
        uav(:,t) = GPS_Coord.GPS2Cart_struct(config.area_config.DEM.ref, current_gps(end, :));
        uav(3,t) = uav(3,t) + config.area_config.DEM.DATA(round(uav(1,t)), round(uav(2,t)));
    end

    
     % check action
     move_fin = Action_check.move_finish(uav(:, t), current_gps);
     
     if move_fin
         if rotate_started
            rotate_fin = Action_check.rotate_finish(uav(:, t), t);
         end
     else
         rotate_fin = false;
     end
    
    % ============= generate measurement =======================
    pulses = comm.get_pulses();
    disp(pulses);
    z = convert_pulses(pulses, config.area_config.DEM.ref);
    z = remove_invalid_id(z, config.target_id_list);
    
    % record data for bearing
    AoA_Generator.record_data(z, t);
    
    % ===================== filtering ==========================
    % filter prediction
    for n = 1:length(density)
        density{n} = AoA_Filter.predict(density{n});
    end
    
    if move_fin && rotate_fin
        rotate_started = false;
        
        % get bearing measurement
        [bearing, pattern, raw_meas, cycle_m] = AoA_Generator.get_bearing();
        Action_check.rotate_start_t = nan;
        disp(bearing);
        meas_count = meas_count + 1;
        pattern_record{meas_count} = pattern;
        raw_meas_record{meas_count} = raw_meas;
        cycle_record{meas_count} = cycle_m;
        bearing_record{meas_count} = bearing;
        bearing_start_end(2, meas_count) = t;
        for n = 1:length(density)
        % extract ID & RSSI from measurement table z
            meas = table2array(bearing(bearing.Target_ID == config.target_id_list(n), :));
            if ~isempty(meas) && ~density{n}.localized
                density{n} = AoA_Filter.filtering(density{n}, meas(1, [1,3]), uav(:, t));
            end
            
            bearing_truth(n, meas_count) = atan2(squeeze(truth(1, n, 1)) - uav(1,t), squeeze(truth(2, n, 1)) - uav(2, t));
        end
    end
    
    
    % =============== check termination codition ================
    [density, all_target_found, found_report, term_val(:, t)] = Terminate.termination_check(density, t);
    
    % ====================== planning ==========================
    if ((move_fin && rotate_fin) || planning_cycle == 0) && ~all_target_found
        planning_cycle = planning_cycle + 1;
        planning_time = tic;
        action_index = action_index + 1;
        [selected_action, selected_target] = Planner.plan(uav(:, t), density);
        fprintf('Planner finish in %f s.\n', toc(planning_time));
        fprintf('Planning Cycle: %i \n', planning_cycle);
        fprintf('Selected Action Type: %s .\n', selected_action.type);
        
        % save selected action
        recorder.record_action(uav(:,t), selected_action, action_index, t, toc(planning_time));
        
        % issue the move command
        waypoint = GPS_Coord.Cart2GPS_struct(config.area_config.DEM.ref, selected_action.waypoints(:, move_time));
        % update waypoint alt to relative altitude
        waypoint.relative_alt = uav_altitude;
        comm.move(waypoint);
        
        % register action
        Action_check.register_action(selected_action);
        move_fin = false;
        rotate_fin = false;
    end
    
    % issue rotate command 
    if move_fin && ~rotate_started
        if (isnan(bearing_start_end(1, meas_count+1)))
            bearing_start_end(1, meas_count+1) = t+1;
        end
        
        % init bearing action
        AoA_Generator.check_start_condition(selected_action, t+1);
        
        Action_check.rotate_start_time(t+1);
        comm.turns(1);
        pause(0.1);
        comm.turns(1);
        rotate_started = true;
    end

    % ====================== Collecting data ====================
    [est] = recorder.record(z, density, found_report, t);
    
    
    % ==================== Time ticking ====================
    proc_time = toc(main_start_time);
    
    
    % ====================== update plot =========================
    if (plot_result && mod(t, plot_interval) == 0) || (t == 1 && plot_result)
        plot_tool.update_plot(density, uav(:, 1:t), truth, est, selected_target);
        plot_tool.update_error_plot(recorder.stats.error_history, t);
        plot_tool.update_termination_value(term_val(:, 1:t));
        drawnow();
    end
    
    % handle figure keyboard event
    if ~isempty(plot_tool.fig_handle.UserData)
        switch plot_tool.fig_handle.UserData.type
            case Control_Type.Terminate
                all_target_found = true;
            case Control_Type.ToggleTruth
                plot_tool.toggle_truth();
        end
        plot_tool.fig_handle.UserData = [];
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

pattern_record = pattern_record(1:meas_count);
raw_meas_record = raw_meas_record(1:meas_count);
cycle_record = cycle_record(1:meas_count);
bearing_record = bearing_record(1:meas_count);
bearing_truth = bearing_truth(:, 1:meas_count);
bearing_start_end = bearing_start_end(:, 1:meas_count);


%% save result
save_prompt = input('Save Result? (Y/N)\n', 's');
if strcmp(save_prompt, 'Y')
    filename = input('Enter Filename: \n', 's');
    if ~isempty(filename)
        filename = strcat('Victor_farm_hilly/', filename, '-', datestr(now, 'mm-dd-HH_MM'), '.mat');
%         save(filename, '-regexp', '^(?!(plot_tool)$).');
        save(filename);
    else
        error('Invalid filename, file NOT saved.\n');
    end
end



% finishing
% ========================================================
if plot_result
    % plot return path
    plot_tool.update_uav_return(uav(:, t), uav0);
end

% Show final GPS coordinate
fprintf('Final Estimated Coordinate:\n');
disp(GPS_Coord.Cart2GPS([config.area_config.DEM.ref.lat; config.area_config.DEM.ref.lon], est(1:2, :)));

% Show GPS coordinate when target first found
fprintf('First found coordinate:\n');
for n = 1:ntarget
   if ~isnan(recorder.stats.found_time(n))
        fprintf('Target %i:\n', target_id_list(n));
        disp(GPS_Coord.Cart2GPS([config.area_config.DEM.ref.lat; config.area_config.DEM.ref.lon], recorder.stats.estimation{n}(1:2, recorder.stats.found_time(n))));
   end
end


%% ====================================================================

% report error
recorder.record_unlocalized(t);
recorder.get_error();
recorder.report_result();