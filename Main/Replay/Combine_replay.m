%% Replaying using recorded data
% RSSI
clear; close all;

% load file
filename = uigetfile('*.mat');
res = load(filename);

%% Global configuration
Total_time = res.Total_time;                      % maximun simulation time

plot_result = true;                              % whether to plot particles
plot_gps = true;
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

% generate module settings
% config = Experiment_config_wrapper_Victor(ntarget, reward, Use_Void, rotation_time);
config  = res.config;   % use replay config

% load DEM
config.area_config.DEM = load('Victor_hilly.mat');
% config.area_config.DEM = load('Swan_Reach_DEM.mat');

% initialize each modules;
% =========== [System modules] ===============
target_model = Target_Model(config.target_config);
RSSI_model = RSSI_Model(config.RSSI_sensor_config);
AoA_model = AoA_Model(config.AoA_sensor_config);
UAV_Action = UAV_Actions(config.uav_config, config.area_config);

Terminate = Termination_Condition(config.Term_config);
AoA_Filter = Bernoulli_Filter(config.AoA_filter_config, AoA_model, target_model);
RSSI_Filter = Bernoulli_Filter(config.RSSI_filter_config, RSSI_model, target_model);
Planner = Planner_greedy_POMDP(config.planner_config, RSSI_model, target_model, UAV_Action, AoA_model);

AoA_Generator = Rotation_Bearing(config.AoA_config);

% =========== [Actio checking] ====================
recorder = Statistics(ntarget, config.target_id_list', Total_time, config, 'simulation', true);
recorder.stats.truth = res.recorder.stats.truth;


try
% calculate truth
truth = nan(3, ntarget, Total_time);
for n = 1:ntarget
   truth(:, n, :) = repmat(res.recorder.stats.truth(:, n), 1, Total_time);
end
% save truth
recorder.stats.truth = truth;
catch e
end
recorder.stats.truth = truth;


RSSI_model.config.target_id_list = res.config.target_id_list;
%% ============== Plotting setting ==================
if plot_result
    if plot_gps
        plot_tool = Result_plot_gps( ...
             ntarget,...
            'Plot3D',           false,...
            'Plot_Void',        config.planner_config.Use_Void, ...
            'Void_r',           config.planner_config.Void_r, ...
            'DEM',              config.area_config.DEM.DATA, ...
            'Area',             config.area_config.area,...
            'PlotTermination',  true, ...
            'Term_th',          config.Term_config.threshold(1), ...
            'Keyboard_Control', true, ...
            'gps_ref',          [config.area_config.DEM.ref.lat; config.area_config.DEM.ref.lon], ...
            'SeperatePlot',     true, ...
            'ShowSatellite',    true ...
        );
    else
        plot_tool = Result_plot_experiment( ...
             ntarget,...
            'Plot3D',           false,...
            'Plot_Void',        config.planner_config.Use_Void, ...
            'Void_r',           config.planner_config.Void_r, ...
            'DEM',              config.area_config.DEM.DATA, ...
            'Area',             config.area_config.area, ...
            'PlotTermination',  true, ...
            'Term_th',          config.Term_config.threshold(1), ...
            'Keyboard_Control', false ...
        ); %#ok<*UNRCH>
    end
end

% initialize filter density
density = cell(ntarget, 1);
% res.config.target_id_list = wanted_ID_list;
for i = 1:ntarget
    density{i} = Bernoulli(gen_birth_particles_polygon(config.area_config.DEM.DATA, 10000, config.area_config.area, config.area_config.Alt_range), 0.01, res.config.target_id_list(i));
end


%% ======================= Main Program =============================
% get home position & initialize state
home = res.home;
% initialize initial position
uav0 = res.uav0;

absolute_alt = uav0(3);

% ======================== Main Loop ================================
t = 0;
bearing_started = false;
bearing_start_t_idx = [];

while t < res.t && ~all_target_found
    fprintf('Epochs: %i/%i.\n', t, Total_time);
    t = t+1;    % increment simulation time

    % ============= get measurement =======================
    z = res.recorder.stats.measurement{t}; 
    if ~isempty(z)
        if istable(z)
            filtered_z = z;
        else
            filtered_z = z{1};
            bearing_z = [];
        end
        
        if ~isempty(filtered_z)
            valid_z = false(3, height(filtered_z));
            for h = 1:height(filtered_z)
                if filtered_z.SNR(h) > 10 && filtered_z.RSSI(h) > -75
                    valid_z(h) = true;
                end
            end
            filtered_z = filtered_z(valid_z, :); 
        end
    else
        filtered_z = [];
        bearing_z = [];
    end
    
    
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
            AoA_Generator.record_data(filtered_z, t);
        elseif t == res.bearing_start_end(2, bearing_start_t_idx)
            bearing_started = false;
            bearing_start_t_idx = [];
            AoA_Generator.start_time = nan;
            [bearing_z, ~, raw_meas, ~] = AoA_Generator.get_bearing();
            disp(bearing_z);
        end
    end
    
        
    % ===================== filtering ==========================
    for n = 1:length(density)
        if ~density{n}.localized
            [meas, meas_uav] = filter_pulse_by_id(filtered_z, n, config.target_id_list);
            
            if ~isempty(meas_uav)
                meas_uav(3) = absolute_alt;
                density{n} = RSSI_Filter.filtering(density{n}, meas, meas_uav);
            else
                density{n} = RSSI_Filter.predict(density{n});
            end
        end
    end
    
    if ~istable(z) && ~isempty(bearing_z)
        for n = 1:length(density)
            if ~density{n}.localized
                % extract ID & RSSI from measurement table 
                meas = table2array(bearing_z(bearing_z.Target_ID == config.target_id_list(n), :));
                if ~isempty(meas)
                    density{n} = AoA_Filter.filtering(density{n}, meas(1, [1,3]), uav(:, t));
                end
            end
        end
    end

    [density, all_target_found, found_report, term_val(:, t)] = Terminate.termination_check(density, t); 
    
    [est] = recorder.record(z, density, found_report, t);
      
    % ====================== update plot =========================
    if (plot_result && mod(t, plot_interval) == 0) || t == 1
        plot_tool.update_plot(density, uav(:, 1:t), recorder.stats.truth(:,:, 1:t), est, selected_target);
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

%rp_recorder.cleanup(t, res.action_index);
recorder.record_unlocalized(t);
recorder.get_error();
recorder.report_result();

%% ====================================================================
