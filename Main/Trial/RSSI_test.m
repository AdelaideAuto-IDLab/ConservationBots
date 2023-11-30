% trail rssi only planning and tracking
clear; close all;

%% Global configuration
Total_time = 800;                      % maximun simulation time

plot_result = true;                     % whether to plot particles
plot_interval = 1;                      % plotting interval
ntarget = 4;                            % number of targets
planning_rssi_var = 4;                  % rssi noise variance used in planning

term_val = nan(ntarget, Total_time);

% truth_gps = [-35.456816,138.515819;-35.458384,138.514581;-35.457305,138.517032;-35.458407,138.515828]';  %victor hill testA
% truth_gps = [-35.454932, 138.515114; -35.456779, 138.515771;-35.456226, 138.519508; -35.458508, 138.515726]'; % victor hill testB (more spread out)
% truth_gps = [-35.455089, 138.516584; -35.455173, 138.514701;-35.455948, 138.519347; -35.457246, 138.517026]'; % victor hill testB (more spread out 2) 16/Sep/2021 
% truth_gps = [-35.455182, 138.517068; -35.455195, 138.514849;-35.455958, 138.51936; -35.457344, 138.517267]'; % victor hill testB (more spread out 2) 23/sep/2021
% truth_gps = [-35.455105, 138.516865; -35.455174, 138.514613;-35.455941, 138.519323; -35.457108, 138.51717]'; % victor hill testB (more spread out 3) 27/sep/2021
% truth_gps = [-35.455121, 138.516826; -35.455147, 138.514596;-35.456007, 138.519348; -35.457159, 138.517337]'; % victor hill testB (more spread out 3) 05/Oct/2021

ID_list = [1,2,3,4];      % 4 tag detector
truth_gps_total = [-35.455127, 138.516761; -35.455163, 138.514686; -35.455801, 138.519258; -35.457262, 138.517191]'; 
wanted_ID_list = [1,2,3,4];

truth_gps = nan(2, length(wanted_ID_list));
for n = 1:length(wanted_ID_list)
    truth_gps(:, n) = truth_gps_total(:, wanted_ID_list(n)==ID_list);
end

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
config = Experiment_config_wrapper_Victor(ntarget, reward, Use_Void, rotation_time);
% config = Experiment_config_wrapper_Swan_Reach(ntarget, Reward_Type.Renyi, false, rotation_time);
target_id_list = config.target_id_list;

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
%%

% =========== [Data recording] ====================
recorder = Statistics(ntarget, config.target_id_list', Total_time, config, 'simulation', true);

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
        'Keyboard_Control', true ...
    );
end

% calculate truth
try
truth = nan(3, ntarget, 1);
for n = 1:ntarget
   truth(1:2, n, 1) = GPS_Coord.GPS2Cart([config.area_config.DEM.ref.lat; config.area_config.DEM.ref.lon], truth_gps(:, config.target_id_list(n)));
   truth(3, n, 1) = config.area_config.DEM.DATA(round(truth(1, n, 1)), round(truth(2,n,1)));
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
starting_altitude = config.area_config.DEM.DATA(round(uav0(1)), round(uav0(2)));
warning('The elevation in uav state is relative elevation, not AMSL.');

% ======================== Main Loop ================================
planning_cycle = 0; % planning cycle count
t = 0;

start_date = datetime();
main_start_time = tic;
proc_time = 1;
while (t < Total_time)  && ~all_target_found
    fprintf('Epochs: %i/%i.\n', t, Total_time);
    loop_time = tic;
    t = t+1;    % increment simulation time
     
    % ============= update uav location ========================
    current_gps = comm.get_gps();
    if ~isempty(current_gps)
        uav(:,t) = GPS_Coord.GPS2Cart_struct(config.area_config.DEM.ref, current_gps(end, :));
%         uav(3,t) = uav(3,t) + config.area_config.DEM.DATA(round(uav(1,t)), round(uav(2,t)));
        uav(3, t) = uav(3,t) + starting_altitude;
    end
    
    % check action
    move_fin = Action_check.move_finish(uav(:, t), current_gps);
    
    % ============= get measurement =======================
    pulses = comm.get_pulses();
    if ~isempty(pulses)
        disp(pulses);
        z = convert_pulses(pulses, config.area_config.DEM.ref);
        z = remove_invalid_id(z, config.target_id_list);
    else
        z = [];
    end
    
    % ===================== filtering ==========================
    for n = 1:length(density)
        % extract ID & RSSI from measurement table z
        [meas, meas_uav] = filter_pulse_by_id(z, config.target_id_list(n), config.target_id_list);
        
        if ~density{n}.localized
            if ~isempty(meas_uav) 
                for m = 1:size(meas_uav, 2)
%                     meas_uav(3, m) = -0.3+ meas_uav(3, m) + config.area_config.DEM.DATA(round(meas_uav(1, m)), round(meas_uav(2, m)));
                    meas_uav(3, m) = -0.3 + meas_uav(3, m) + starting_altitude;
                end
                density{n} = RSSI_Filter.filtering(density{n}, meas, meas_uav);
            else
                density{n} = RSSI_Filter.predict(density{n});
            end
        end
    end
    
    % =============== check termination codition ================
    [density, all_target_found, found_report, term_val(:, t)] = Terminate.termination_check(density, t);
    
    % ====================== planning ==========================
    if  (move_fin || planning_cycle == 0) && ~all_target_found
        planning_cycle = planning_cycle + 1;
        planning_time = tic;
        action_index = action_index + 1;
        [selected_action, selected_target] = Planner.plan(uav(:, t), density);
        fprintf('Planner finish in %f s.\n', toc(planning_time));
        fprintf('Selected Action Type: %s .\n', selected_action.type);
        
        % save selected action
        recorder.record_action(uav(:,t), selected_action, action_index, t, toc(planning_time));
        
        % issue the move command
        waypoint = GPS_Coord.Cart2GPS_struct(config.area_config.DEM.ref, selected_action.waypoints(:, end));
        % update waypoint alt to relative altitude
        waypoint.relative_alt = uav_altitude;
        comm.move(waypoint);
        
       % register action
       Action_check.register_action(selected_action);
       move_fin = false;
    end
   
    
    % ====================== Collecting data ====================
    [est] = recorder.record(z, density, found_report, t);
     
    % ==================== Time ticking ====================
    proc_time = toc(main_start_time);
       
    % ====================== update plot =========================
    if (plot_result && mod(t, plot_interval) == 0) || t == 1
        plot_tool.update_plot(density, uav(:, 1:t), truth, est, selected_target);
        plot_tool.update_error_plot(recorder.stats.error_history, t);
        plot_tool.update_termination_value(term_val(:, 1:t));
        drawnow();
    end
    
    % handle figure keyboard event
    if ~isempty(plot_tool.fig_handle.UserData)
        switch plot_tool.fig_handle.UserData.type
            case Control_Type.Terminate
                all_target_found = true;
            case Control_Type.ToggleTruth
                plot_tool.toggle_truth();
        end
        plot_tool.fig_handle.UserData = [];
    end
    
    % wait until looptime reach 1 second
    while toc(loop_time) < 1
        pause(0.05);
    end
    
end

% send UAV back to home position
% adjust heading
home.hdg = rad2deg(wrapTo2Pi(atan2(uav0(1) - uav(1,t), uav0(2) - uav(2,t))));
comm.move(home);

% close tcp connection to mavlink-proxy
comm.close_tcp();

% save uav path
recorder.record_uav_path(uav);

% cleanup recorded data
recorder.cleanup(t, action_index);

%% save result
save_prompt = input('Save Result? (Y/N)\n', 's');
if strcmp(save_prompt, 'Y')
    filename = input('Enter Filename: \n', 's');
    if ~isempty(filename)
        filename = strcat('Victor_farm_hilly/', filename, '-', datestr(now, 'mm-dd-HH_MM'), '.mat');
        save(filename);
    else
        error('Invalid filename, file NOT saved.\n');
    end
end

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

