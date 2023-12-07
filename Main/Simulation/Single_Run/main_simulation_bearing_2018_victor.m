%% Main problem for simulation
clear; close all;
addpath(genpath(pwd));

%% Global configuration
Total_time = 15000;                      % maximun simulation time
uav0 = [993, 504, 50, pi/4]';               % uav initial location 
all_target_found = false;               % true if all targets are found 
reward = Reward_Type.Shannon;
Use_Void = false;
rotation_time = 45;

plot_result = true;
plot_interval = 20;

%% Useful intermedia variable
uav = nan(4, Total_time);
selected_target = nan;              % planner selected target
action_index = 0;                   % the nth action


%% Initialization


% generate module settings
ntarget = 4;
config = simulation_config_wrapper_victor(ntarget, reward, Use_Void, rotation_time);

% update RSSI model 
config.RSSI_sensor_config.Antenna = Antenna_Type.Isotropic;
config.RSSI_sensor_config.Likelihood_Type = Likelihood_Type.precise;
config.RSSI_sensor_config.Sigma_RSSI = 12;   % increased nosie to handle non-precise model

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

% =========== [Simulation modules] ============
target_truth_generator = Target_State_Generator(config.sim_target_config, target_model);
RSSI_generator = RSSI_Generator(config.sim_sensor_config, config.sim_target_radio_config, config.area_config.DEM.DATA);

% =========== [Data recording] ====================
recorder = Statistics(ntarget, (1:ntarget)', Total_time, config, 'Simulation', false);


%% Simulation preparation 
% initialize uav height
uav0(3) = uav0(3)+config.area_config.DEM.DATA(round(uav0(1)), round(uav0(2)));


% generate target truth
% truth = target_truth_generator.gen_target_state(Total_time, config.area_config.DEM.DATA);
truth = nan(3, ntarget, Total_time);
for n = 1:ntarget
    for t = 1:Total_time
       truth(1:2, n, t) = config.sim_target_config.Init_State(1:2, n);
       truth(3, n, t) = config.area_config.DEM.DATA(round(truth(1, n, 1)), round(truth(2,n,1))) + 0.5;
    end
end
% save truth
recorder.stats.truth = truth;


% initialize filter density
density = cell(config.target_config.Ntarget, 1);
for i = 1:config.target_config.Ntarget
    density{i} = Bernoulli(gen_birth_particles_uniform(config.area_config.DEM.DATA, 20000), 0.01, i);
end

% generate initial action
selected_action.waypoints = UAV_Action.get_composit_waypoints(uav0, pi/4, 10);
selected_action.type = Action_Type.RSSI;


%% Plotting setting
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



%% ======================== Main Loop ================================
t = 0;
term_val = nan(ntarget, Total_time);
while t <= Total_time && ~all_target_found
    t = t+1;    % increment simulation time
    n_cycle = ceil(t/config.planner_config.T);  % current planning cycle
    
    % ============= update uav location ========================
    if n_cycle == 1
        uav(:, t) = selected_action.waypoints(:, t);
    else
        uav(:, t) = selected_action.waypoints(:, t- (n_cycle-1)*config.planner_config.T);
    end
    
    % ============= generate measurement =======================
    z = RSSI_generator.get_RSSI(squeeze(truth(:, 1:ntarget, t)), uav(:, t)); 
    % record data for bearing
    AoA_Generator.record_data(z, t);
    
    % ===================== filtering ==========================
    % filter prediction
    for n = 1:length(density)
        density{n} = AoA_Filter.predict(density{n});
    end
    
    
    if mod(t, config.planner_config.T) == 0
        % get bearing measurement
        bearing = AoA_Generator.get_bearing();
        % get maximum RSSI
        meas = AoA_Generator.get_max();
        for n = 1:length(density)
            % extract ID & RSSI from measurement table z
            density{n} = AoA_Filter.update(density{n}, table2array(bearing(n, [1,3])), uav(:, t));
            % perform range update
            density{n} = RSSI_Filter.update(density{n}, [n, meas(n)], uav(:,t));
        end
    end
    
    % ====================== planning ==========================
    if mod(t, config.planner_config.T) == 0
        planning_time = tic;
        action_index = action_index + 1;
        [selected_action, selected_target] = Planner.plan(uav(:, t), density);
        fprintf('Planner finish in %f s.\n', toc(planning_time));
        fprintf('Selected Action Type: %s .\n', selected_action.type);
        
        % init bearing action
        AoA_Generator.check_start_condition(selected_action, t + config.uav_config.traject_time - config.uav_config.rotation_time);
        
        % save selected action
        recorder.record_action(uav(:,t), selected_action, action_index, t);
    end
    
    % =============== check termination codition ================
    [density, all_target_found, found_report, term_val(:, t)] = Terminate.termination_check(density, t);
    
    % ====================== Collecting data ====================
    [est] = recorder.record(z, density, found_report, t);
    
    
    % ====================== update plot =========================
    if plot_result && mod(t, plot_interval) == 0
        plot_tool.update_plot(density, uav(:, 1:t), truth(:, :, 1:t), est, selected_target);
        plot_tool.update_error_plot(recorder.stats.error_history, t);
        plot_tool.update_termination_value(term_val(:, 1:t));
    end
end

% Save UAV Path
recorder.record_uav_path(uav);

if plot_result
    % plot return path
    plot_tool.update_uav_return(uav(:, t), uav0);
end

% report result
recorder.get_error();
recorder.report_result();

%% ====================================================================
% save result 



