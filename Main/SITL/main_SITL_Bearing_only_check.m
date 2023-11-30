%% Main problem for SITL
clear; close all;

%% Global configuration
Total_time = 2000;                      % maximun simulation time

plot_result = true;                     % whether to plot particles
plot_interval = 1;                      % plotting interval
ntarget = 4;                            % number of targets

term_val = nan(ntarget, Total_time);

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
% config = Experiment_config_wrapper_Waite(ntarget, reward, Use_Void, rotation_time);
config = simulation_config_wrapper(ntarget, reward, Use_Void, rotation_time);

% initialize each modules
% =========== [System modules] ===============
target_model = Target_Model(config.target_config);
AoA_model = AoA_Model(config.AoA_sensor_config);
UAV_Action = UAV_Actions(config.uav_config, config.area_config);

Terminate = Termination_Condition(config.Term_config);
AoA_Filter = Bernoulli_Filter(config.AoA_filter_config, AoA_model, target_model);
Planner = Planner_greedy_POMDP(config.planner_config, AoA_model, target_model, UAV_Action);

AoA_Generator = Rotation_Bearing(config.AoA_config);
% =========== [Simulation modules] ============
target_truth_generator = Target_State_Generator(config.sim_target_config, target_model);
RSSI_generator = RSSI_Generator(config.sim_sensor_config, config.sim_target_radio_config, config.area_config.DEM.DATA);

% =========== [Data recording] ====================
recorder = Statistics(ntarget, (1:ntarget)', Total_time, config, 'Simulation', false);

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
        'Keyboard_Control', false ...
    );
end


% initialize filter density
density = cell(config.target_config.Ntarget, 1);
for i = 1:config.target_config.Ntarget
    density{i} = Bernoulli(gen_birth_particles_polygon(config.area_config.DEM.DATA, 10000, config.area_config.area, config.area_config.DEM.Alt_range), 0.01, i);
end


% generate target truth
truth = target_truth_generator.gen_target_state(Total_time, config.area_config.DEM.DATA);
% save truth 
recorder.record_truth(truth);
move_time = config.planner_config.T - config.uav_config.rotation_time;



%% ====================== SITL initialization ============================
comm = UAV_Comm('filter_config_udoo.json', 'Debug', false);
comm.request_gps_sitl(5);

%% ======================= Main Program =============================
main_start_time = tic;
proc_time = 1;

% get home position & initialize state
home = comm.get_home_pos();
% initialize initial position
uav0 = GPS_Coord.GPS2Cart_struct(config.area_config.DEM.ref, home);
uav0(3) = uav0(3) + config.area_config.DEM.DATA(round(uav0(1)), round(uav0(2)));
warning('The elevation in uav state is relative elevation, not AMSL.');


% ======================== Main Loop ================================
t = 0;
planning_cycle = 0; % planning cycle count
move_fin = false;
rotate_fin = false;
rotate_started = false;
while (t < Total_time) && ~all_target_found
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
    move_fin = Action_check.move_finish(uav(:, t), current_gps);

    if move_fin && ~rotate_started
        fprintf('Movement Finished.\n');
    elseif move_fin && rotate_started 
        rotate_fin = Action_check.rotate_finish(uav(:, t), t);
    else
        rotate_fin = false;
    end
    
    % ============= generate measurement =======================
    z = RSSI_generator.get_RSSI_SITL_debug(squeeze(truth(:, 1:ntarget, t)), uav(:, t), 1); 
    % record data for bearing
    AoA_Generator.record_data(z, t);
    
    % ===================== filtering ==========================
    % filter prediction
    for n = 1:length(density)
        density{n} = AoA_Filter.predict(density{n});
    end
    
    if move_fin && rotate_fin
        fprintf('Rotate Finished.\n');
        Action_check.rotate_start_t = nan;
        rotate_started = false;
        % get bearing measurement
        [bearing, pattern, raw_meas, cycle_m] = AoA_Generator.get_bearing();
        disp(bearing);
        for n = 1:length(density)
            if ~density{n}.localized
                % extract ID & RSSI from measurement table z
                density{n} = AoA_Filter.filtering(density{n}, table2array(bearing(n, [1,3])), uav(:, t));
            end
        end
    end
    
    % ====================== planning ==========================
    if (move_fin && rotate_fin) || planning_cycle == 0
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
        Action_check.rotate_start_time(t+1);
        % init bearing action
        AoA_Generator.check_start_condition(selected_action, t+1);
        
        fprintf('Rotate Started.\n');
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
        drawnow;
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

% save uav path
recorder.record_uav_path(uav);
%% ====================================================================

% report error
recorder.get_error();
recorder.report_result();

% send UAV back to home position
% adjust heading
home.hdg = rad2deg(wrapTo2Pi(atan2(uav0(1) - uav(1,t), uav0(2) - uav(2,t))));
comm.move(home);

% close tcp connection to mavlink-proxy
comm.close_tcp();


