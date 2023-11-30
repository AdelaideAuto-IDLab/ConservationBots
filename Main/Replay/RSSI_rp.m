%% Replaying using recorded data
% RSSI
clear; close all;

% load file
filename = uigetfile('*.mat');
try
    res = load(filename);
catch
end


%% Global configuration
Total_time = res.Total_time;                      % maximun simulation time

plot_result = true;                               % whether to plot particles
plot_interval = 1;                                % plotting interval
save_video = plot_result & boolean(0);

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

rp_recorder = Statistics(ntarget, config.target_id_list', Total_time, config, 'simulation', true);
rp_recorder.stats.truth = res.truth;

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
    %plot_tool.update_target_trial([85, 112, 160.6]');
    
    if save_video
        video_name = strcat('vid-rssi-', datestr(now,'mm-dd-HH_MM'), '.avi');
        writeObj = VideoWriter(video_name);
        writeObj.FrameRate = 10;
        open(writeObj);
    end
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
        valid_z = boolean(zeros(3, height(z)));
        for h = 1:height(z)
            if z.SNR(h) > 15  && z.RSSI(h) > -75 
                valid_z(h) = true;
            end
        end
        z = z(valid_z, :);
%         disp(z);
    end
    
    % ===================== filtering ==========================
    for n = 1:length(density)
        if ~density{n}.localized
            [meas, meas_uav] = filter_pulse_by_id(z, n, config.target_id_list);
            
            if ~isempty(meas_uav)
%                 meas_uav(3) = meas_uav(3) + config.area_config.DEM.DATA(round(meas_uav(1)), round(meas_uav(2))) - 0.3;
                meas_uav(3) = absolute_alt;
                density{n} = RSSI_Filter.filtering(density{n}, meas, meas_uav);
            end
        end
    end
    
    [density, all_target_found, found_report, term_val(:, t)] = Terminate.termination_check(density, t);
    [est] = rp_recorder.record(z, density, found_report, t);
      
    % ====================== update plot =========================
    if (plot_result && mod(t, plot_interval) == 0) || t == 1
        plot_tool.update_plot(density, uav(:, 1:t), res.truth, est, selected_target);
        plot_tool.update_error_plot(rp_recorder.stats.error_history, t);
        plot_tool.update_termination_value(term_val(:, 1:t));
        drawnow();
        
        if save_video
            frame = getframe(plot_tool.fig_handle);
            writeVideo(writeObj, frame);
        end
    end 
end


% finishing
% ========================================================
if plot_result
    % plot return path
    plot_tool.update_uav_return(uav(:, t), uav0);
end

rp_recorder.cleanup(t, res.action_index);
rp_recorder.record_unlocalized(t);
rp_recorder.get_error();
rp_recorder.report_result();

if save_video
    close(writeObj);
end
%% ====================================================================

