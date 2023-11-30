%% Replaying using recorded data
% RSSI
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

try
    reward = res.reward;
    Use_Void = res.Use_Void;
    rotation_time = res.rotation_time;
catch
    reward = Reward_Type.Renyi;
    Use_Void = true;
    rotation_time = 20;
end

%% Useful intermedia variable
uav = res.uav;
selected_target = 0;
action_index = 0;                   % the nth action
all_target_found = false;           % true if all targets are found 

%% Initialization
addpath(genpath(pwd));

% [Use modified config]
% config = Experiment_config_wrapper_Victor(ntarget, reward, Use_Void, rotation_time);
% [Use original config]
config = res.config;

% load DEM
config.area_config.DEM = load('Victor_hilly.mat');

% initialize each modules
% =========== [System modules] ===============
target_model = Target_Model(config.target_config);
RSSI_model = RSSI_Model(config.RSSI_sensor_config);
UAV_Action = UAV_Actions(config.uav_config, config.area_config);

Terminate = Termination_Condition(config.Term_config);
RSSI_Filter = Bernoulli_Filter(config.RSSI_filter_config, RSSI_model, target_model);
Planner = Planner_greedy_POMDP(config.planner_config, RSSI_model, target_model, UAV_Action);

% =========== [Actio checking] ====================
Action_check = Action_Checker(config.uav_config);

recorder = Statistics(ntarget, config.target_id_list', Total_time, config, 'simulation', true);
recorder.stats.truth = res.truth;

RSSI_model.config.target_id_list = res.config.target_id_list;

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
    density{i} = Bernoulli(gen_birth_particles_polygon(config.area_config.DEM.DATA, 10000, config.area_config.area, config.area_config.Alt_range), 0.01, res.config.target_id_list(i));
end


%% ======================= Main Program =============================
proc_time = 1;

% get home position & initialize state
home = res.home;
% initialize initial position
uav0 = res.uav0;
absolute_alt = uav0(3);

% ======================== Main Loop ================================
t = 0;

while t < res.t && ~all_target_found
    fprintf('Epochs: %i/%i.\n', t, Total_time);
    t = t+1;    % increment simulation time

    % ============= get measurement =======================
    z = res.recorder.stats.measurement{t}; 
    if istable(z)
        valid_z = false(3, height(z));
        for h = 1:height(z)
            if z.SNR(h) > 15  && z.RSSI(h) > -75 
                valid_z(h) = true;
            end
        end
        z = z(valid_z, :);
    end
    
    % ===================== filtering ==========================
    for n = 1:length(density)
        if ~density{n}.localized
            [meas, meas_uav] = filter_pulse_by_id(z, n, config.target_id_list);
            
            if ~isempty(meas_uav)
                meas_uav(3) = absolute_alt;
                density{n} = RSSI_Filter.filtering(density{n}, meas, meas_uav);
            end
        end
    end
    
    [density, all_target_found, found_report, term_val(:, t)] = Terminate.termination_check(density, t);
    [est] = recorder.record(z, density, found_report, t);
      
    % ====================== update plot =========================
    if (plot_result && mod(t, plot_interval) == 0) || t == 1
        plot_tool.update_plot(density, uav(:, 1:t), res.truth, est, selected_target);
        plot_tool.update_error_plot(recorder.stats.error_history, t);
        plot_tool.update_termination_value(term_val(:, 1:t));
        drawnow();
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

