%% Main problem for SITL
clear; close all;


%% Global configuration
Total_time = 800;                      % maximun simulation time
% ID_list = [4,6,7,8];      % 4 tag detector
% % truth_gps_total = [-35.455221, 138.516699; -35.455236, 138.514994; -35.455959, 138.519316; -35.457301, 138.517274]';
% truth_gps_total = [-34.534156, 139.580463; -34.532685, 139.581054; -34.537712, 139.579234; -34.528899, 139.579922]';
% wanted_ID_list = [4, 6,7,8];

ID_list = [4,7];      % 4 tag detector
wanted_ID_list = [4,7];
truth_gps_total = [-34.534156, 139.580463;  -34.537712, 139.579234]';
non_localized_list = boolean(ones(1, length(ID_list)));

truth_gps = nan(2, length(wanted_ID_list));
for n = 1:length(wanted_ID_list)
    truth_gps(:, n) = truth_gps_total(:, wanted_ID_list(n)==ID_list);
end

ntarget = length(wanted_ID_list);

plot_result = true;                     % whether to plot particles
plot_interval = 1;                      % plotting interval
planning_rssi_var = 4;                  % rssi noise variance used in planning

uav_altitude = 50;                      % desire uav operation altitude (AGL)

reward = Reward_Type.Renyi;
Use_Void = true;
rotation_time = 20;
term_val = nan(ntarget, Total_time);

%% Useful intermedia variable
uav = nan(4, Total_time);
selected_action = [];               % planner output
selected_target = nan;              % planner selected target
all_target_found = false;           % true if all targets are found 
measurement_subset = cell(1, Total_time);
planning_time_step = nan(1, Total_time);
planning_cycle = 0; % planning cycle count

%% Initialization
addpath(genpath(pwd));

% generate module settings
% config = Experiment_config_wrapper_Victor(ntarget, reward, Use_Void, rotation_time);
config = Experiment_config_wrapper_Swan_Reach(ntarget, Reward_Type.Renyi, Use_Void, rotation_time);
target_id_list = config.target_id_list;

% initialize each modules
% =========== [System modules] ===============
target_model = Target_Model(config.target_config);
RSSI_model = RSSI_Model(config.RSSI_sensor_config);
AoA_model = AoA_Model(config.AoA_sensor_config);
UAV_Action = UAV_Actions(config.uav_config, config.area_config);

Terminate = Termination_Condition(config.Term_config);
AoA_Filter = Bernoulli_Filter(config.AoA_filter_config, AoA_model, target_model);
RSSI_Filter = Bernoulli_Filter(config.RSSI_filter_config, RSSI_model, target_model);
Planner = Planner_greedy_POMDP(config.planner_config, RSSI_model, target_model, UAV_Action, AoA_model);
%% Change planning rssi noise variance
Planner.Meas_Model.config.Sigma_RSSI = planning_rssi_var;

AoA_Generator = Rotation_Bearing(config.AoA_config);

% =========== [Data recording] ====================
recorder = Statistics(ntarget, config.target_id_list', Total_time, config, 'simulation', true);
raw_meas_record = cell(100, 1);
bearing_record = cell(100, 1);
bearing_truth = nan(ntarget, Total_time);
bearing_start_end = nan(2, 100);
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
        'ID_list',          config.target_id_list, ...
        'Term_th',          config.Term_config.threshold(1), ...
        'Keyboard_Control', true ...
    );
end

try
% calculate truth
truth = nan(3, ntarget, 1);
for n = 1:ntarget
   truth(1:2, n, 1) = GPS_Coord.GPS2Cart([config.area_config.DEM.ref.lat; config.area_config.DEM.ref.lon], truth_gps(:, n));
   truth(3, n, 1) = config.area_config.DEM.DATA(round(truth(1, n, 1)), round(truth(2,n,1)));
end
% save truth
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
starting_altitude = config.area_config.DEM.DATA(round(uav0(1)), round(uav0(2)));
warning('The elevation in uav state is relative elevation, not AMSL.');


% ======================== Main Loop ================================
t = 0;
meas_count = 0;
move_fin = false;
rotate_fin = false;
rotate_started = false;


main_start_time = tic;
proc_time = 1;
start_date = datetime();
while (t < Total_time) && ~all_target_found
    fprintf('Epos: %i/%i.\n', t, Total_time);
    loop_time = tic;
    t = t+1;    % increment simulation time
     
    % ============= update uav location ========================
    current_gps = comm.get_gps();
    if ~isempty(current_gps)
        uav(:,t) = GPS_Coord.GPS2Cart_struct(config.area_config.DEM.ref, current_gps(end, :));
%         uav(3,t) = uav(3,t) + config.area_config.DEM.DATA(round(uav(1,t)), round(uav(2,t)));
        uav(3, t) = uav(3,t) + starting_altitude;
    end
    
    % check action
    move_fin = Action_check.move_finish(uav(:, t), current_gps);

    if move_fin
        if selected_action.type == Action_Type.RSSI
            rotate_fin = true;
        elseif selected_action.type == Action_Type.Combine
            rotate_fin = Action_check.rotate_finish(uav(:, t), t) & rotate_started;
        end
    end
    
    % ============= generate measurement =======================
    pulses = comm.get_pulses();
    if ~isempty(pulses)
        disp(pulses);
        z = convert_pulses(pulses, config.area_config.DEM.ref);
        z = remove_invalid_id(z, config.target_id_list);
    else
        z = [];
    end
    measurement_subset{1,t} = remove_invalid_id(z, wanted_ID_list);
    
    % record data for bearing
    AoA_Generator.record_data(z, t);
    
    % ===================== filtering ==========================
    % RSSI filtering
    for n = 1:length(density)
        % extract ID & RSSI from measurement table z
        [meas, meas_uav] = filter_pulse_by_id(z, n, config.target_id_list);

        if ~isempty(meas_uav)
            for m = 1:size(meas_uav, 2)
%                 meas_uav(3, m) = -0.3+ meas_uav(3, m) + config.area_config.DEM.DATA(round(meas_uav(1, m)), round(meas_uav(2, m)));
                meas_uav(3, m) = -0.3 + meas_uav(3, m) + starting_altitude;
            end
            
            if ~density{n}.localized
                density{n} = RSSI_Filter.filtering(density{n}, meas, meas_uav);
            end
        else
            density{n} = RSSI_Filter.predict(density{n});
        end
    end
    
    if move_fin && rotate_fin && selected_action.type == Action_Type.Combine
        rotate_started = false;
        
        % get bearing measurement
        [bearing, ~, raw_meas, ~] = AoA_Generator.get_bearing();
        Action_check.rotate_start_t = nan;
        z = {z, bearing};
        display(bearing);
        meas_count = meas_count + 1;
        raw_meas_record{meas_count} = raw_meas;
        bearing_record{meas_count} = bearing;
        bearing_start_end(2, meas_count) = t;
        for n = 1:length(density)
            % extract ID & RSSI from measurement table 
            meas = table2array(bearing(bearing.Target_ID == config.target_id_list(n), :));
            if ~isempty(meas) && ~density{n}.localized
                density{n} = AoA_Filter.filtering(density{n}, meas(1, [1,3]), uav(:, t));
            end
            
            bearing_truth(n, meas_count) = atan2(squeeze(truth(1, n, 1)) - uav(1,t), squeeze(truth(2, n, 1)) - uav(2, t));
        end
    end
    
    % =============== check termination codition ================
    [density, all_target_found, found_report, term_val(:, t)] = Terminate.termination_check(density, t);
    
%     % update non localized list
%     for n = 1:ntarget
%         if density{n}.localized
%             non_localized_list(n) = false;
%         end
%     end
    
    % ====================== planning ==========================
    if ((move_fin && rotate_fin) || planning_cycle == 0) && ~all_target_found
        planning_cycle = planning_cycle + 1;
        planning_time_step(1, planning_cycle) = t;
        planning_time = tic;
        
        if planning_cycle == 1  % first planning action: rotation/normal planning
%             [selected_action, selected_target] = Planner.plan(uav(:, t), density);
            selected_action.waypoints = repmat(uav(:,t), 1, config.uav_config.traject_time);
            selected_action.type = Action_Type.Combine;
            selected_target = nan;
        else
            detect_count = zeros(ntarget, 1);
            for d = planning_time_step(1, planning_cycle-1):t
                if istable(measurement_subset{d})
                    for Itar = 1:ntarget
                        Idx = find(measurement_subset{d}.Target_ID == wanted_ID_list(Itar));
                        if ~isempty(Idx)
                            detect_count(Itar, 1) = detect_count(Itar, 1)+1;
                        end
                    end
                end
            end
            if max(detect_count(non_localized_list)/(t-planning_time_step(1, planning_cycle-1))) < 0.05
                % rotate if detection count is low
                selected_action.waypoints = repmat(uav(:,t), 1, config.uav_config.traject_time);
                selected_action.type = Action_Type.Combine;
                selected_target = nan;
                fprintf('Low detections, doing Bearing\n');
            else
                % else do normal planning
                [selected_action, selected_target] = Planner.plan(uav(:, t), density);
            end
        end
        fprintf('Planner finish in %f s.\n', toc(planning_time));
        fprintf('Planning Cycle: %i \n', planning_cycle);
        fprintf('Selected Action Type: %s .\n', selected_action.type);
       
        
        % save selected action
        recorder.record_action(uav(:,t), selected_action, planning_cycle, t, toc(planning_time));
        
        % issue the move command
        if selected_action.type == Action_Type.Combine
            waypoint = GPS_Coord.Cart2GPS_struct(config.area_config.DEM.ref, selected_action.waypoints(:, move_time));
        elseif selected_action.type == Action_Type.RSSI
            waypoint = GPS_Coord.Cart2GPS_struct(config.area_config.DEM.ref, selected_action.waypoints(:, end));
        end
        waypoint.relative_alt = uav_altitude;
        comm.move(waypoint);
        
        % register action
        Action_check.register_action(selected_action);
        move_fin = false;
        rotate_fin = false;
    end
    
    % issue rotate command 
    if move_fin && ~rotate_started && selected_action.type == Action_Type.Combine 
        if (isnan(bearing_start_end(1, meas_count+1)))
            bearing_start_end(1, meas_count+1) = t+1;
        end
        
        % init bearing action
        AoA_Generator.check_start_condition(selected_action, t+1);
        
        Action_check.rotate_start_time(t+1);
        fprintf('Rotate Started.\n');
        
        comm.turns(1);
        rotate_started = true;
    end
    

    % ====================== Collecting data ====================
    [est] = recorder.record(z, density, found_report, t);
    % ==================== Time ticking ====================
    proc_time = toc(main_start_time);
    
    
    % ====================== update plot =========================
    if (plot_result && mod(t, plot_interval) == 0) || t == 1
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

% clean up
measurement_subset = measurement_subset(1:t);
planning_time_step = planning_time_step(1, 1:planning_cycle);  

% send UAV back to home position
% adjust heading
home.hdg = rad2deg(wrapTo2Pi(atan2(uav0(1) - uav(1,t), uav0(2) - uav(2,t))));
comm.move(home);

% close tcp connection to mavlink-proxy
comm.close_tcp();


% save uav path
recorder.record_uav_path(uav);
% cleanup recorded data
recorder.cleanup(t, planning_cycle);

raw_meas_record = raw_meas_record(1:meas_count);
bearing_record = bearing_record(1:meas_count);
bearing_truth = bearing_truth(:, 1:meas_count);
bearing_start_end = bearing_start_end(:, 1:meas_count);


%% save result
save_prompt = input('Save Result? (Y/N)\n', 's');
if strcmp(save_prompt, 'Y')
    filename = input('Enter Filename: \n', 's');
    if ~isempty(filename)
        filename = strcat('Swan_Reach_test2/', filename, '-', datestr(now, 'mm-dd-HH_MM'), '.mat');
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

% % report error
recorder.record_unlocalized(t);
recorder.get_error();
recorder.report_result();