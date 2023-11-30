%% Replaying using recorded data
% AoA
clear; close all;

% load file
filename = uigetfile('*.mat');
res = load(filename);


%% Global configuration
Total_time = res.Total_time;                      % maximun simulation time

plot_result = true;                               % whether to plot particles
plot_interval = 1;                                % plotting interval
ntarget = res.ntarget;                            % number of targets

uav_altitude = res.uav_altitude;                  % desire uav operation altitude (AGL)

reward = res.reward;
Use_Void = res.Use_Void;
rotation_time = res.rotation_time;

%% Useful intermedia variable
uav = res.uav;
selected_target = 0;
action_index = 0;                   % the nth action
all_target_found = false;           % true if all targets are found 

%% Initialization
addpath(genpath(pwd));

% generate module settings
% config = res.config;
config = Experiment_config_wrapper_Victor(ntarget, reward, Use_Void, rotation_time);

move_time = config.planner_config.T - config.uav_config.rotation_time;

% initialize each modules
% =========== [System modules] ===============
target_model = Target_Model(config.target_config);
AoA_model = AoA_Model(config.AoA_sensor_config);
RSSI_model = RSSI_Model(config.RSSI_sensor_config);
UAV_Action = UAV_Actions(config.uav_config, config.area_config);

Terminate = Termination_Condition(config.Term_config);
AoA_Filter = Bernoulli_Filter(config.AoA_filter_config, AoA_model, target_model);
RSSI_Filter = Bernoulli_Filter(config.RSSI_filter_config, RSSI_model, target_model);
Planner = Planner_greedy_POMDP(config.planner_config, AoA_model, target_model, UAV_Action);
AoA_Generator = Rotation_Bearing(config.AoA_config);
% =========== [Actio checking] ====================
Action_check = Action_Checker(config.uav_config);

rp_recorder = Statistics(ntarget, config.target_id_list', Total_time, config, 'simulation', true);
rp_recorder.stats.truth = res.truth;

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
    %plot_tool.update_target_trial([85, 112, 160.6]');
end


% initialize filter density
density = cell(config.target_config.Ntarget, 1);
for i = 1:config.target_config.Ntarget
    density{i} = Bernoulli(gen_birth_particles_polygon(config.area_config.DEM.DATA, 10000, config.area_config.area, config.area_config.Alt_range), 0.01, res.config.target_id_list(i));
end


%% ======================= Main Program =============================
proc_time = 1;

% get home position & initialize state
home = res.home;
% initialize initial position
uav0 = res.uav0;


% ======================== Main Loop ================================
t = 0;
meas_count = 0;
planning_cycle = 0; % planning cycle count
move_fin = false;
rotate_fin = false;
while t < res.t
    fprintf('Epos: %i/%i.\n', t, Total_time);
    t = t+1;    % increment simulation time
    
    % check action
    previous_move_fin = move_fin;
    move_fin = Action_check.move_finish(uav(:, t), t);

    if previous_move_fin && move_fin
        rotate_fin = Action_check.rotate_finish(uav(:, t), t);
    else
        rotate_fin = false;
    end
    % ============= get measurement =======================
    z = res.recorder.stats.measurement{t}; 
    % record data for bearing
    AoA_Generator.record_data(z, t);
    % ===================== filtering ==========================
    % filter prediction
    for n = 1:length(density)
        density{n} = AoA_Filter.predict(density{n});
    end
    
    % RSSI filtering
    for n = 1:length(density)
        [meas, meas_uav] = filter_pulse_by_id(z, config.target_id_list(n), config.target_id_list);
        if ~isempty(meas_uav)
            meas_uav(3) = meas_uav(3) + config.area_config.DEM.DATA(round(meas_uav(1)), round(meas_uav(2)));
            density{n} = RSSI_Filter.filtering(density{n}, meas, meas_uav);
        end
    end
    
    if move_fin && rotate_fin
        % get bearing measurement
        [bearing, pattern, raw_meas] = AoA_Generator.get_bearing();
        disp(bearing);
        meas_count = meas_count + 1;
        for n = 1:length(density)
        % extract ID & RSSI from measurement table z
            meas = table2array(bearing(bearing.Target_ID == config.target_id_list(n), :));
            if ~isempty(meas)
                density{n} = AoA_Filter.filtering(density{n}, meas(1, [1,3]), uav(:, t));
            end
        end
    end
    
    % ====================== planning ==========================
    if (move_fin && rotate_fin) || planning_cycle == 0
        planning_cycle = planning_cycle + 1;
        action_index = action_index + 1;

        selected_action = res.recorder.stats.action_history{action_index}.action;

        fprintf('Selected Action Type: %s .\n', selected_action.type);

        % init bearing action
        AoA_Generator.check_start_condition(selected_action, t);

        % register action
        Action_check.register_action(selected_action);
        move_fin = false;
        rotate_fin = false;
    end
    
    [~, ~, ~, term_val] = Terminate.termination_check(density, t);
    
    % =============== check termination codition ================
    [est] = rp_recorder.record(z, density, [], t);
     
    % ====================== update plot =========================
    if (plot_result && mod(t, plot_interval) == 0) || t == 1
        plot_tool.update_plot(density, uav(:, 1:t), res.truth, est, selected_target);
        plot_tool.update_error_plot(rp_recorder.stats.error_history, t);
        plot_tool.update_termination_value(term_val);
    end 
    
end


% finishing
% ========================================================
if plot_result
    % plot return path
    plot_tool.update_uav_return(uav(:, t), uav0);
end
%% ====================================================================
