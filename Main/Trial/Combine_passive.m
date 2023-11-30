%% RSSI only tracking, no planning
clear; close all;

%% Global configuration 
Total_time = 800;                      % maximun simulation time

ID_list = [4,7,8];      % 4 tag detector
% truth_gps_total = [-35.455221, 138.516699; -35.455236, 138.514994; -35.455959, 138.519316; -35.457301, 138.517274]';
truth_gps_total = repmat([nan; nan], 1, length(ID_list));
wanted_ID_list = [4, 7,8];

truth_gps = nan(2, length(wanted_ID_list));
for n = 1:length(wanted_ID_list)
    truth_gps(:, n) = truth_gps_total(:, wanted_ID_list(n)==ID_list);
end

ntarget = length(wanted_ID_list);

plot_result = true;                     % whether to plot particles
plot_interval = 1;                      % plotting interval
rotation_time = 20;
term_val = nan(ntarget, Total_time);

uav_altitude = 50;                      % desire uav operation altitude (AGL)

%% Useful intermedia variable
uav = nan(4, Total_time);
all_target_found = false;           % true if all targets are found 

%% Initialization
addpath(genpath(pwd));

% generate module settings
% config = Experiment_config_wrapper_Victor(ntarget, Reward_Type.Renyi, false, rotation_time);
config = Experiment_config_wrapper_Swan_Reach(ntarget, Reward_Type.Renyi, false, rotation_time);
target_id_list = config.target_id_list;

% initialize each modules
% =========== [System modules] ===============
target_model = Target_Model(config.target_config);
RSSI_model = RSSI_Model(config.RSSI_sensor_config);
AoA_model = AoA_Model(config.AoA_sensor_config);

Terminate = Termination_Condition(config.Term_config);
RSSI_Filter = Bernoulli_Filter(config.RSSI_filter_config, RSSI_model, target_model);
AoA_Filter = Bernoulli_Filter(config.AoA_filter_config, AoA_model, target_model);

AoA_Generator = Rotation_Bearing(config.AoA_config);

% =========== [Data recording] ====================
recorder = Statistics(ntarget, config.target_id_list', Total_time, config, 'simulation', true);
raw_meas_record = cell(100, 1);
bearing_record = cell(100, 1);
bearing_truth = nan(ntarget, Total_time);
bearing_start_end = nan(2, 100);
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
        'ID_list',          config.target_id_list, ...
        'Keyboard_Control', true ...
    );
end
plot_tool.toggle_truth();

try
% calculate truth
truth = nan(3, ntarget, 1);
for n = 1:ntarget
   truth(1:2, n, 1) = GPS_Coord.GPS2Cart([config.area_config.DEM.ref.lat; config.area_config.DEM.ref.lon], truth_gps(:, n));
%    truth(3, n, 1) = config.area_config.DEM.DATA(round(truth(1, n, 1)), round(truth(2,n,1)));
end
% save truth
recorder.stats.truth = truth;
catch e
end

% initialize filter density
density = cell(config.target_config.Ntarget, 1);
for i = 1:config.target_config.Ntarget
    density{i} = Bernoulli(gen_birth_particles_polygon(config.area_config.DEM.DATA, 10000, config.area_config.area, config.area_config.Alt_range), 0.01, config.target_id_list(i));
end


%% ====================== SITL initialization ============================
comm = UAV_Comm('filter_config_udoo.json', 'Debug', true);

%% ======================= Main Program =============================
% get home position & initialize state
home = comm.get_home_pos();
% initialize initial position
uav0 = GPS_Coord.GPS2Cart_struct(config.area_config.DEM.ref, home);
uav0(3) = uav0(3) + config.area_config.DEM.DATA(round(uav0(1)), round(uav0(2)));
warning('The elevation in uav state is relative elevation, not AMSL.');

% ======================== Main Loop ================================
t = 0;

current_speed = [];
meas_count = 0;
rotate_stat.is_finished = true;
rotate_stat.is_started = false;
rotate_stat.start_time = [];
rotate_stat.init_angle = nan;

start_date = datetime();
main_start_time = tic;
proc_time = 1;
while (t < Total_time) && ~all_target_found
    fprintf('Epos: %i/%i.\n', t, Total_time);
    loop_time = tic;
    t = t+1;    % increment simulation time
     
    % ============= update uav location ========================
    current_gps = comm.get_gps();
    if ~isempty(current_gps)
        uav(:,t) = GPS_Coord.GPS2Cart_struct(config.area_config.DEM.ref, current_gps(end, :));
        uav(3,t) = uav(3,t) + config.area_config.DEM.DATA(round(uav(1,t)), round(uav(2,t)));
        current_speed = sqrt(current_gps.vx^2 + current_gps.vy^2);
    end
    
    % ============= get measurement =======================
    pulses = comm.get_pulses();
    disp(pulses);
    z = convert_pulses(pulses, config.area_config.DEM.ref);
    z = remove_invalid_id(z, config.target_id_list);
    
    AoA_Generator.record_data(z, t);
    % ===================== RSSI filtering ======================
    if ~isempty(z)
        for n = 1:length(density)
            % extract ID & RSSI from measurement table z
            idx = find(z.Target_ID == config.target_id_list(n));
            meas = table2array(z(idx, [1,3]));
            if ~isempty(meas) 
                meas_uav = z.UAV(idx, :)';
                meas_uav(3) = meas_uav(3) + config.area_config.DEM.DATA(round(meas_uav(1)), round(meas_uav(2)));

                if ~density{n}.localized
                    density{n} = RSSI_Filter.filtering(density{n}, meas, meas_uav);
                end
            end

        end
    end
    
    if rotate_stat.is_started
        if t - rotate_stat.start_time < 3
            rotate_stat.is_finished = false;
        elseif t - rotate_stat.start_time > rotation_time*1.2
            rotate_stat.is_finished = true;
        elseif abs(angdiff(rotate_stat.init_angle, uav(4, t))) < deg2rad(3)
            rotate_stat.is_finished = true;
        end
        
        if rotate_stat.is_finished
            rotate_stat.is_started = false;
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
                if ~isempty(meas)
                    density{n} = AoA_Filter.filtering(density{n}, meas(1, [1,3]), uav(:, t));
                end

                bearing_truth(n, meas_count) = atan2(squeeze(truth(1, n, 1)) - uav(1,t), squeeze(truth(2, n, 1)) - uav(2, t));
            end
        end
    end
    
    
    % =============== check termination codition ================
    [density, all_target_found, found_report, term_val(:, t)] = Terminate.termination_check(density, t);
    
    % ====================== Collecting data ====================
    [est] = recorder.record(z, density, found_report, t);
    
    % ==================== Time ticking ====================
    proc_time = toc(main_start_time);
    
    %% ==================== Send Movement command ================
    if ~isempty(plot_tool.fig_handle.UserData)
        switch plot_tool.fig_handle.UserData.type
            case Control_Type.Move
                if inpolygon(plot_tool.fig_handle.UserData.waypoint(1), ...
                             plot_tool.fig_handle.UserData.waypoint(2), ...
                             config.area_config.area(1, :), ...
                             config.area_config.area(2, :))
                
                    % issue the move command
                    waypoint = GPS_Coord.Cart2GPS_struct(config.area_config.DEM.ref, plot_tool.fig_handle.UserData.waypoint);
                    % update waypoint alt to relative altitude
                    waypoint.relative_alt = uav_altitude;
                    waypoint.hdg = atan2d(plot_tool.fig_handle.UserData.waypoint(1) - uav(1,t), plot_tool.fig_handle.UserData.waypoint(2) - uav(2,t));
                    pause(0.2);
                    comm.move_global(waypoint);
                end
            case Control_Type.Rotate
                if ~rotate_stat.is_started && rotate_stat.is_finished
                    rotate_stat.init_angle = uav(4, t);
                    comm.turns(1);
                    rotate_stat.is_started = true;
                    rotate_stat.is_finished = false;
                    rotate_stat.start_time = t;
                    AoA_Generator.check_start_condition2(t);
                end
            case Control_Type.Terminate
                all_target_found = true;
            case Control_Type.ToggleTruth
                plot_tool.toggle_truth();
        end
        plot_tool.fig_handle.UserData = [];
    end
   
    % ====================== update plot =========================
    if (plot_result && mod(t, plot_interval) == 0) || t == 1
        plot_tool.update_plot(density, uav(:, 1:t), truth, est, 1, current_speed);
        plot_tool.update_error_plot(recorder.stats.error_history, t);
        plot_tool.update_termination_value(term_val(:, 1:t));
        drawnow;
    end
    
    % wait until looptime reach 1 second
    while toc(loop_time) < 1
        pause(0.05);
    end
    
end

% Show final GPS coordinate
fprintf('Final Estimated Coordinate:\n');
disp(GPS_Coord.Cart2GPS([config.area_config.DEM.ref.lat; config.area_config.DEM.ref.lon], est(1:2, :)));

% send UAV back to home position
% adjust heading
home.hdg = rad2deg(wrapTo2Pi(atan2(uav0(1) - uav(1,t), uav0(2) - uav(2,t))));
comm.move(home);

% close tcp connection to mavlink-proxy
comm.close_tcp();

% save uav path
recorder.record_uav_path(uav);
% cleanup recorded data
recorder.cleanup(t, 1);

raw_meas_record = raw_meas_record(1:meas_count);
bearing_record = bearing_record(1:meas_count);
bearing_truth = bearing_truth(:, 1:meas_count);
bearing_start_end = bearing_start_end(:, 1:meas_count);

%% save result
% save_prompt = input('Save Result? (Y/N)\n', 's');
% if strcmp(save_prompt, 'Y')
%     filename = input('Enter Filename: \n', 's');
%     if ~isempty(filename)
%         filename = strcat('Victor_farm_hilly/', filename, '-', datestr(now, 'mm-dd-HH_MM'), '.mat');
%         save(filename);
%     else
%         error('Invalid filename, file NOT saved.\n');
%     end
% end

filename = strcat('Swan_Reach_test2/', 'combine_passive_test', '-', datestr(now, 'mm-dd-HH_MM'), '.mat');
save(filename);

% finishing
% ========================================================
if plot_result
    % plot return path
    plot_tool.update_uav_return(uav(:, t), uav0);
end
% Show final GPS coordinate
fprintf('Final Estimated Coordinate:\n');
disp(GPS_Coord.Cart2GPS([config.area_config.DEM.ref.lat; config.area_config.DEM.ref.lon], est(1:2, :)));

% Show GPS coordinate when target first found
fprintf('First found coordinate:\n');
for n = 1:ntarget
   if ~isnan(recorder.stats.found_time(n))
        fprintf('Target %i:\n', config.target_id_list(n));
        disp(GPS_Coord.Cart2GPS([config.area_config.DEM.ref.lat; config.area_config.DEM.ref.lon], recorder.stats.estimation{n}(1:2, recorder.stats.found_time(n))));
   end
end

%% ====================================================================

% report error
recorder.record_unlocalized(t);
recorder.get_error();
recorder.report_result();


