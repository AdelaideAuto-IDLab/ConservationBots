%% Main problem for SITL
clear; close all;


%% Global configuration
Total_time = 800;                      % maximun simulation time

plot_result = true;                     % whether to plot particles
plot_interval = 1;                      % plotting interval
ntarget = 1;                            % number of targets
planning_rssi_var = 4;                  % rssi noise variance used in planning
wanted_ID_list = [1];

term_val = nan(ntarget, Total_time);

uav_altitude = 50;                      % desire uav operation altitude (AGL)

reward = Reward_Type.Renyi;
Use_Void = true;
rotation_time = 20;
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
config = simulation_config_wrapper(ntarget, reward, Use_Void, rotation_time);
% config = Experiment_config_wrapper_Victor(ntarget, reward, Use_Void, rotation_time);
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

% =========== [Simulation modules] ============
target_truth_generator = Target_State_Generator(config.sim_target_config, target_model);
RSSI_generator = RSSI_Generator(config.sim_sensor_config, config.sim_target_radio_config, config.area_config.DEM.DATA);


% =========== [Data recording] ====================
recorder = Statistics(ntarget, (1:ntarget)', Total_time, config, 'Simulation', true);
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
        'Term_th',          config.Term_config.threshold(1) ...
    );
end



% initialize filter density
density = cell(config.target_config.Ntarget, 1);
for i = 1:config.target_config.Ntarget
    density{i} = Bernoulli(gen_birth_particles_polygon(config.area_config.DEM.DATA, 10000, config.area_config.area, config.area_config.DEM.Alt_range), 0.01, i);
end

% % generate target truth
% truth = target_truth_generator.gen_target_state(Total_time, config.area_config.DEM.DATA);
% calculate truth
truth = nan(3, ntarget, Total_time);
for n = 1:ntarget
    for t = 1:Total_time
       truth(1:2, n, t) = config.sim_target_config.Init_State(1:2, n);
       truth(3, n, t) = config.area_config.DEM.DATA(round(truth(1, n, 1)), round(truth(2,n,1)));
    end
end
% save truth
recorder.stats.truth = truth;


% save truth 
recorder.record_truth(truth);
move_time = config.planner_config.T - config.uav_config.rotation_time;

%% ====================== SITL initialization ============================
comm = UAV_Comm('filter_config_udoo.json', 'Debug', true);
% comm.request_gps_sitl(5);


%% ======================= Main Program =============================
main_start_time = tic;
proc_time = 1;

% get home position & initialize state
home = comm.get_home_pos();
% initialize initial position
uav0 = GPS_Coord.GPS2Cart_struct(config.area_config.DEM.ref, home);
uav0(3) = uav0(3) + config.area_config.DEM.DATA(round(uav0(1)), round(uav0(2)));
warning('The elevation in uav state is relative elevation, not AMSL.');
pause(0.5);

% ======================== Main Loop ================================
t = 0;
meas_count = 0;
move_fin = false;
rotate_fin = false;
rotate_started = false;
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
        if selected_action.type == Action_Type.RSSI
            rotate_fin = true;
        elseif selected_action.type == Action_Type.Combine
            rotate_fin = Action_check.rotate_finish(uav(:, t), t) & rotate_started;
        end
    end
    
    % ============= generate measurement =======================
    z = RSSI_generator.get_RSSI_SITL_debug(squeeze(truth(:, 1:ntarget, t)), uav(:, t), 1);
    invalid_list = boolean(zeros(1, ntarget));
    for h = 1:height(z)
        if z.RSSI(h) < config.sim_sensor_config.Sensitivity
            invalid_list(h) = true;
        end
    end
    z(invalid_list,:) = [];
    disp(z);
    measurement_subset{1,t} = remove_invalid_id(z, wanted_ID_list);

    % record data for bearing
    AoA_Generator.record_data(z, t);
    
    % ===================== filtering ==========================
    % RSSI filtering
    for n = 1:length(density)
        % extract ID & RSSI from measurement table z
        [meas, meas_uav] = filter_pulse_by_id(z, config.target_id_list(n), config.target_id_list);

        if ~isempty(meas_uav)
            for m = 1:size(meas_uav, 2)
                meas_uav(3, m) = meas_uav(3, m) + config.area_config.DEM.DATA(round(meas_uav(1, m)), round(meas_uav(2, m)));
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
    
    % ====================== planning ==========================
    if (move_fin && rotate_fin) || planning_cycle == 0
        planning_cycle = planning_cycle + 1;
        planning_time = tic;
        
        planning_time_step(1, planning_cycle) = t;
        
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
            
            if max(detect_count/(t-planning_time_step(1, planning_cycle-1))) < 0.1
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
        
        comm.turns(1);
        rotate_started = true;
    end
    
    % =============== check termination codition ================
    [density, all_target_found, found_report, term_val(:, t)] = Terminate.termination_check(density, t);
    
    % ====================== Collecting data ====================
    [est] = recorder.record(z, density, found_report, t);
    
    
    % ==================== Time ticking ====================
    proc_time = toc(main_start_time);
    
    
    % ====================== update plot =========================
    if (plot_result && mod(t, plot_interval) == 0) || t == 1
        plot_tool.update_plot(density, uav(:, 1:t), truth(:, 1:ntarget, 1:t), est, selected_target);
        plot_tool.update_error_plot(recorder.stats.error_history, t);
        plot_tool.update_termination_value(term_val(:, 1:t));
    end
    
    
    % wait until looptime reach 1 second
    while toc(loop_time) < 1
        pause(0.05);
    end
    
end
planning_time_step = planning_time_step(1, 1:planning_cycle);   

% send UAV back to home position
% adjust heading
home.hdg = rad2deg(wrapTo2Pi(atan2(uav0(1) - uav(1,t), uav0(2) - uav(2,t))));
comm.move(home);


% finishing
% ========================================================
if plot_result
    % plot return path
    plot_tool.update_uav_return(uav(:, t), uav0);
end

% save uav path
recorder.record_uav_path(uav);
%% ====================================================================

% report error
recorder.get_error();
recorder.report_result();



