%% Main problem for SITL
clear; close all;

%% Global configuration
Total_time = 5000;                      % maximun simulation time

plot_result = true;                     % whether to plot particles
plot_interval = 1;                      % plotting interval
ntarget = 4;                            % number of targets

term_val = nan(ntarget, Total_time);

reward = Reward_Type.Renyi;
Use_Void = true;
rotation_time = 20;

uav_altitude = 50;                      % desire uav operation altitude (AGL)

%% Useful intermedia variable
uav = nan(4, Total_time);
all_target_found = false;           % true if all targets are found 

%% Initialization
addpath(genpath(pwd));

% generate module settings
% config = Experiment_config_wrapper_Victor(ntarget, reward, Use_Void, rotation_time);
config = simulation_config_wrapper(ntarget, reward, Use_Void, rotation_time);
target_id_list = config.target_id_list;

% initialize each modules
% =========== [System modules] ===============
target_model = Target_Model(config.target_config);
RSSI_model = RSSI_Model(config.RSSI_sensor_config);
UAV_Action = UAV_Actions(config.uav_config, config.area_config);

Terminate = Termination_Condition(config.Term_config);
RSSI_Filter = Bernoulli_Filter(config.RSSI_filter_config, RSSI_model, target_model);

% =========== [Simulation modules] ============
target_truth_generator = Target_State_Generator(config.sim_target_config, target_model);
RSSI_generator = RSSI_Generator(config.sim_sensor_config, config.sim_target_radio_config, config.area_config.DEM.DATA);

% =========== [Data recording] ====================
recorder = Statistics(ntarget, (1:ntarget)', Total_time, config, 'Simulation', true);

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
        'Term_th',          config.Term_config.threshold, ...
        'Keyboard_Control', true ...
    );
end


% initialize filter density
density = cell(config.target_config.Ntarget, 1);
for i = 1:config.target_config.Ntarget
    density{i} = Bernoulli(gen_birth_particles_polygon(config.area_config.DEM.DATA, 10000, config.area_config.area, config.area_config.DEM.Alt_range), 0.01, config.target_id_list(i));
end


%% Simulation preparation 
% generate target truth
% truth = target_truth_generator.gen_target_state(Total_time, config.area_config.DEM.DATA);
truth = nan(3, ntarget, Total_time);
for n = 1:ntarget
    for t = 1:Total_time
       truth(1:2, n, t) = config.sim_target_config.Init_State(1:2, n);
       truth(3, n, t) = config.area_config.DEM.DATA(round(truth(1, n, 1)), round(truth(2,n,1)));
    end
end
% save truth
recorder.stats.truth = truth;


%% ====================== SITL initialization ============================
comm = UAV_Comm('filter_config_udoo.json', 'Debug', true);
comm.request_gps_sitl(5);

%% ======================= Main Program =============================
% get home position & initialize state
pause(1);
home = comm.get_home_pos();
% initialize initial position
uav0 = GPS_Coord.GPS2Cart_struct(config.area_config.DEM.ref, home);
uav0(3) = uav0(3) + config.area_config.DEM.DATA(round(uav0(1)), round(uav0(2)));

warning('The elevation in uav state is relative elevation, not AMSL.');


% ======================== Main Loop ================================
t = 0;
planning_cycle = 0; % planning cycle count
move_fin = false;

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
    
    % ============= generate measurement =======================
    z = RSSI_generator.get_RSSI_SITL_debug(squeeze(truth(:, 1:ntarget, t)), uav(:, t), 1); 
    
    
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
                    comm.move(waypoint);
                end
            case Control_Type.Rotate
                comm.turns(1);
            case Control_Type.Terminate
                all_target_found = true;
            case Control_Type.ToggleTruth
                plot_tool.toggle_truth();
        end
        plot_tool.fig_handle.UserData = [];
    end
    
    % ====================== update plot =========================
    if (plot_result && mod(t, plot_interval) == 0) || t == 1
        plot_tool.update_plot(density, uav(:, 1:t), truth(:, 1:ntarget, 1:t), est, 1, current_speed);
        plot_tool.update_error_plot(recorder.stats.error_history, t);
        plot_tool.update_termination_value(term_val(:, 1:t));
        drawnow;
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


% finishing
% ========================================================
if plot_result
    % plot return path
    plot_tool.update_uav_return(uav(:, t), uav0);
end

% save uav path
recorder.record_uav_path(uav);
%% ====================================================================

% report error
recorder.get_error();
recorder.report_result();

