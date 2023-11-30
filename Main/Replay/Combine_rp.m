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
save_video = plot_result & boolean(0);

ntarget = res.ntarget;                            % number of targets
% ntarget = 2;

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
% config = Experiment_config_wrapper_Swan_Reach(ntarget, reward, Use_Void, rotation_time);
config  = res.config;   % use replay config
% config.RSSI_filter_config.Regularized=true;

% config.area_config.DEM.DATA = rot90(config.area_config.DEM.DATA);
% config.area_config.DEM.DATA = rot90(config.area_config.DEM.DATA);

% initialize each modules;
% =========== [System modules] ===============
target_model = Target_Model(config.target_config);
RSSI_model = RSSI_Model(config.RSSI_sensor_config);
AoA_model = AoA_Model(config.AoA_sensor_config);
UAV_Action = UAV_Actions(config.uav_config, config.area_config);

Terminate = Termination_Condition(config.Term_config);
AoA_Filter = Bernoulli_Filter(config.AoA_filter_config, AoA_model, target_model);
RSSI_Filter = Bernoulli_Filter(config.RSSI_filter_config, RSSI_model, target_model);
% RSSI_Filter = Particle_Filter(config.RSSI_filter_config, RSSI_model, target_model);
Planner = Planner_greedy_POMDP(config.planner_config, RSSI_model, target_model, UAV_Action, AoA_model);

AoA_Generator = Rotation_Bearing(config.AoA_config);

% =========== [Actio checking] ====================
rp_recorder = Statistics(ntarget, config.target_id_list', Total_time, config, 'simulation', true);
rp_recorder.stats.truth = res.recorder.stats.truth;

% % Swan Reach test only
% truth_gps_list = [-34.534156, 139.580463; -34.532685, 139.581054; -34.537712, 139.579234; -34.528899, 139.579922; -34.532949, 139.577721]';
% truth_id_list = [4, 6, 7, 8, 9];
% wanted_ID_list = [4, 7];
% [~, tmp_idx] = intersect(truth_id_list, wanted_ID_list, 'stable');
% truth_gps = truth_gps_list(:, tmp_idx);
% 
% truth_gps = res.truth_gps;

% ==== Uncomment for static target ==========
% try
% % calculate truth
% truth = nan(3, ntarget, Total_time);
% for n = 1:ntarget
%    truth(1:2, n, :) = repmat(GPS_Coord.GPS2Cart([config.area_config.DEM.ref.lat; config.area_config.DEM.ref.lon], truth_gps(:, n)), 1, Total_time);
%    truth(3, n, :) = config.area_config.DEM.DATA(round(truth(1, n, 1)), round(truth(2,n,1)));
% end
% % save truth
% recorder.stats.truth = truth;
% catch e
% end
% rp_recorder.stats.truth = truth;


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
            'ShowSatellite',    boolean(1) ...
        );
%         plot_tool.fig_handle.Position = [100   100   733   720];    % swan reach
%         plot_tool.app.UIAxes_Main.XLim = [138.5095  138.5213];
%         plot_tool.app.UIAxes_Main.YLim = [-35.4609  -35.4510];

        plot_tool.fig_handle.Position = [305, 108, 830, 720];       % victor harbor
%         plot_tool.app.UIAxes_Main.XLim = [138.5083224811, 138.5232886999];
%         plot_tool.app.UIAxes_Main.YLim = [-35.4621872116098, -35.451066239586225];

        plot_tool.app.UIAxes_Main.XLim = [139.5750262970103, 139.5914891376903];
        plot_tool.app.UIAxes_Main.YLim = [-34.539444543675735, -34.527211474449807];
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


    if save_video
        video_name = strcat('vid-combine-', datestr(now,'mm-dd-HH_MM'), '.avi');
        writeObj = VideoWriter(video_name);
        writeObj.FrameRate = 10;
        open(writeObj);
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
%             bearing_z = z{2};     % 
            bearing_z = [];
        end
        
        if ~isempty(filtered_z)
            valid_z = boolean(zeros(3, height(filtered_z)));
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
%                 meas_uav(3) = meas_uav(3) + config.area_config.DEM.DATA(round(meas_uav(1)), round(meas_uav(2))) - 0.3;
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
    
    [est] = rp_recorder.record(z, density, found_report, t);
      
    % ====================== update plot =========================
    if (plot_result && mod(t, plot_interval) == 0) || t == 1
        plot_tool.update_plot(density, uav(:, 1:t), rp_recorder.stats.truth(:,:, 1:t), est, selected_target);
        plot_tool.update_error_plot(rp_recorder.stats.error_history, t);
        plot_tool.update_termination_value(term_val(:, 1:t));
        drawnow();
        if save_video
            frame = getframe(plot_tool.fig_handle);
            writeVideo(writeObj, frame);
        end
    end 
    
    if any([40, 90, 120,150, 203] == t)
        disp('');
    end
    
      
end


% finishing
% ========================================================
if plot_result
    % plot return path
    plot_tool.update_uav_return(uav(:, t), uav0);
end

%rp_recorder.cleanup(t, res.action_index);
rp_recorder.record_unlocalized(t);
rp_recorder.get_error();
rp_recorder.report_result();

if save_video
    close(writeObj);
end
%% ====================================================================
