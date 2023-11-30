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

term_val = nan(ntarget, Total_time);
uav_altitude = res.uav_altitude;                  % desire uav operation altitude (AGL)

reward = res.reward;
Use_Void = res.Use_Void;
rotation_time = res.rotation_time;

%% Useful intermedia variable
uav = res.uav;
all_target_found = false;           % true if all targets are found 

%% Initialization
addpath(genpath(pwd));

% generate module settings
config = res.config;
% config = Experiment_config_wrapper_Victor(ntarget, reward, Use_Void, rotation_time);

% load DEM
config.area_config.DEM = load('Victor_hilly.mat');
move_time = config.planner_config.T - config.uav_config.rotation_time;

% initialize each modules
% =========== [System modules] ===============
target_model = Target_Model(config.target_config);
AoA_model = AoA_Model(config.AoA_sensor_config);
UAV_Action = UAV_Actions(config.uav_config, config.area_config);

Terminate = Termination_Condition(config.Term_config);
AoA_Filter = Bernoulli_Filter(config.AoA_filter_config, AoA_model, target_model);
Planner = Planner_greedy_POMDP(config.planner_config, AoA_model, target_model, UAV_Action);
AoA_Generator = Rotation_Bearing(config.AoA_config);


recorder = Statistics(ntarget, config.target_id_list', Total_time, config, 'simulation', true);
recorder.stats.truth = res.truth;

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
        'Term_th',          config.Term_config.threshold ...
    );
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

bearing_started = false;
bearing_start_t_idx = [];

while t < res.t && ~all_target_found
    fprintf('Epochs: %i/%i.\n', t, Total_time);
    t = t+1;    % increment simulation time
    
    % ===================== filtering ==========================
    % filter prediction
    for n = 1:length(density)
        density{n} = AoA_Filter.predict(density{n});
    end
    
    
    % ============= get measurement =======================
    z = res.recorder.stats.measurement{t}; 
    
    % recalculate bearing from real measured data
    if isempty(bearing_start_t_idx)
        bearing_start_t_idx = find(t == res.bearing_start_end(1,:));
        if ~isempty(bearing_start_t_idx) && res.bearing_start_end(2,bearing_start_t_idx) - res.bearing_start_end(1,bearing_start_t_idx) > 5
            AoA_Generator.start_time = res.bearing_start_end(1,bearing_start_t_idx);
            AoA_Generator.end_time = res.bearing_start_end(2,bearing_start_t_idx);
            bearing_started = true;
        end
    end
    
    if bearing_started && ~isempty(bearing_start_t_idx)
        if t < res.bearing_start_end(2, bearing_start_t_idx)
            AoA_Generator.record_data(z, t);
        elseif t == res.bearing_start_end(2, bearing_start_t_idx)
            bearing_started = false;
            bearing_start_t_idx = [];
            AoA_Generator.start_time = nan;
            [bearing_z, ~, raw_meas, ~] = AoA_Generator.get_bearing();
            disp(bearing_z);
            
            for n = 1:length(density)
            % extract ID & RSSI from measurement table z
                meas = table2array(bearing_z(bearing_z.Target_ID == config.target_id_list(n), :));
                if ~isempty(meas)
                    density{n} = AoA_Filter.filtering(density{n}, meas(1, [1,3]), uav(:, t));
                end
            end
        end
    end
    
    
    [density, all_target_found, found_report, term_val(:, t)] = Terminate.termination_check(density, t);
    
    % =============== check termination codition ================
    [est] = recorder.record(z, density, found_report, t);
     
    % ====================== update plot =========================
    if (plot_result && mod(t, plot_interval) == 0) || t == 1
        plot_tool.update_plot(density, uav(:, 1:t), res.truth, est, 0);
        plot_tool.update_error_plot(recorder.stats.error_history, t);
        plot_tool.update_termination_value(term_val(:, 1:t));
    end 
    
end


% finishing
% ========================================================
if plot_result
    % plot return path
    plot_tool.update_uav_return(uav(:, t), uav0);
end

recorder.cleanup(t, res.action_index);
recorder.record_unlocalized(t);
recorder.get_error();
recorder.report_result();
%% ====================================================================
