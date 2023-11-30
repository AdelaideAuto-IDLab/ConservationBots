%% Main problem for simulation
clear; close all;
addpath(genpath(pwd));

%% Global configuration
Total_time = 1000;                      % maximun simulation time
uav0 = [993, 504, 50, pi/4]';          % uav initial location 
Use_Void = true;
rotation_time = 20;

plot_result = true;
plot_interval = 1;
reward = Reward_Type.Renyi;
planning_rssi_var = 4;                  % rssi noise variance used in planning

%% Useful intermedia variable
uav = nan(4, Total_time);
selected_action = [];               % planner output
selected_target = nan;              % planner selected target
action_index = 0;                   % the nth action
all_target_found = false;           % true if all targets are found 

%% Initialization


% generate module settings
ntarget = 4;
config = simulation_config_wrapper(ntarget, reward, Use_Void, rotation_time);
target_id_list = config.target_id_list;

term_val = nan(ntarget, Total_time);

% initialize each modules
% =========== [System modules] ===============
target_model = Target_Model(config.target_config);
RSSI_model = RSSI_Model(config.RSSI_sensor_config);
UAV_Action = UAV_Actions(config.uav_config, config.area_config);

Terminate = Termination_Condition(config.Term_config);
RSSI_Filter = Bernoulli_Filter(config.RSSI_filter_config, RSSI_model, target_model);
Planner = Planner_greedy_POMDP(config.planner_config, RSSI_model, target_model, UAV_Action);
%% Change planning rssi noise variance
Planner.Meas_Model.config.Sigma_RSSI = planning_rssi_var;

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
    density{i} = Bernoulli(gen_birth_particles_polygon(config.area_config.DEM.DATA, 10000, config.area_config.area, config.area_config.Alt_range), 0.01, config.target_id_list(i));
end

% generate initial action
selected_action.waypoints = [];


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
planning_cycle = 0;
t = 0;
while t <= Total_time && ~all_target_found
    t = t+1;    % increment simulation time
    n_cycle = ceil(t/config.planner_config.T);  % current planning cycle
    
    % ============= update uav location ========================
    if planning_cycle == 0
        uav(:, t) = uav0;
    else
        uav(:, t) = selected_action.waypoints(:, t- (planning_cycle-1)*config.planner_config.T);
    end
    
    % ============= generate measurement =======================
    z = RSSI_generator.get_RSSI(squeeze(truth(:, 1:ntarget, t)), uav(:, t)); 
    
    
    % ===================== filtering ==========================
    for n = 1:length(density)
        % extract ID & RSSI from measurement table z
        if ~isempty(z)
            idx = find(z.Target_ID == config.target_id_list(n));
            if ~isempty(idx)
                density{n} = RSSI_Filter.filtering(density{n}, [config.target_id_list(n), z.RSSI(idx)], uav(:,t));
            else
                % predicting
            end
        end
    end
    
    % ====================== planning ==========================
    if mod(t, config.planner_config.T) == 0 || planning_cycle == 0
        planning_cycle = planning_cycle + 1;
        planning_time = tic;
        action_index = action_index + 1;
        [selected_action, selected_target] = Planner.plan(uav(:, t), density);
        fprintf('Planner finish in %f s.\n', toc(planning_time));
        fprintf('Selected Action Type: %s .\n', selected_action.type);
        % save selected action
        recorder.record_action(uav(:,t), selected_action, action_index, t);
    end
    
    % =============== check termination codition ================
    [density, all_target_found, found_report, term_val(:,t)] = Terminate.termination_check(density, t);
    
    % ====================== Collecting data ====================
    [est] = recorder.record(z, density, found_report, t);
    
    
    % ====================== update plot =========================
    if plot_result && mod(t, plot_interval) == 0
        plot_tool.update_plot(density, uav(:, 1:t), truth, est, selected_target);
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

% print result
recorder.get_error();
recorder.report_result();

%% ====================================================================
% save result 



