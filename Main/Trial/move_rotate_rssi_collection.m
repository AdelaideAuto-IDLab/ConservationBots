%% Differential measurement collection 
clear; close all;

%% Global configuration
start_date = datetime();
Total_time = 400;                      % maximun simulation time

plot_result = true;                     % whether to plot particles
plot_interval = 1;                      % plotting interval
ntarget = 4;                            % number of targets
truth_gps = [-35.455221, 138.516699; -35.455236, 138.514994; -35.455959, 138.519316; -35.457301, 138.517274]';
yaw_rate = deg2rad(40);

uav_altitude = 50;                      % desire uav operation altitude (AGL)
%% Useful intermedia variable
uav = nan(4, Total_time);

%% Initialization
addpath(genpath(pwd));

% generate module settings
target_id_list = [1,2,3,4];
DEM_file = 'Victor_hilly';

area_gps = get_gps_boundary('Victor_Harbor_Hill_good');
area_config = init_area_config( ...
                'GPS', area_gps, ...
                'DEM', DEM_file);       

% initialize each modules
% =========== [Data recording] ====================
recorder.stats.measurement = cell(1, Total_time);
recorder.stats.uav = [];

%% ============== Plotting setting ==================
if plot_result
    plot_tool = Result_plot_experiment( ...
        ntarget,...
        'Plot3D',           false,...
        'Plot_Void',        false, ...
        'DEM',              area_config.DEM.DATA, ...
        'Area',             area_config.area, ...
        'PlotTermination',  false, ...
        'Keyboard_Control', true ...
    );
end

try
% calculate truth
truth = nan(3, ntarget, 1);
for n = 1:ntarget
   truth(1:2, n, 1) = GPS_Coord.GPS2Cart([area_config.DEM.ref.lat; area_config.DEM.ref.lon], truth_gps(:, target_id_list(n)));
   truth(3, n, 1) = area_config.DEM.DATA(round(truth(1, n, 1)), round(truth(2,n,1)));
end
% save truth
recorder.stats.truth = truth;
catch e
end


%% ====================== SITL initialization ============================
comm = UAV_Comm('filter_config_udoo.json', 'Debug', true);

%% ======================= Main Program =============================
% get home position & initialize state
home = comm.get_home_pos();
% initialize initial position
uav0 = GPS_Coord.GPS2Cart_struct(area_config.DEM.ref, home);
uav0(3) = uav0(3) + area_config.DEM.DATA(round(uav0(1)), round(uav0(2)));
warning('The elevation in uav state is relative elevation, not AMSL.');

% ======================== Main Loop ================================
t = 0;

main_start_time = tic;
all_target_found = false;

while (t < Total_time)  && ~all_target_found
    fprintf('Epos: %i/%i.\n', t, Total_time);
    loop_time = tic;
    t = t+1;    % increment simulation time
     
    % ============= update uav location ========================
    current_gps = comm.get_gps();
    if ~isempty(current_gps)
        uav(:,t) = GPS_Coord.GPS2Cart_struct(area_config.DEM.ref, current_gps(end, :));
        uav(3,t) = uav(3,t) + area_config.DEM.DATA(round(uav(1,t)), round(uav(2,t)));
    end
    
    % ============= get measurement =======================
    pulses = comm.get_pulses();
    disp(pulses);
    z = convert_pulses(pulses, area_config.DEM.ref);
    z = remove_invalid_id(z, target_id_list);
    
    recorder.stats.measurement{t} = z;
    
    %% ==================== Send Movement command ================
    if ~isempty(plot_tool.fig_handle.UserData)
        switch plot_tool.fig_handle.UserData.type
            case Control_Type.Move
                if inpolygon(plot_tool.fig_handle.UserData.waypoint(1), ...
                             plot_tool.fig_handle.UserData.waypoint(2), ...
                             area_config.area(1, :), ...
                             area_config.area(2, :))
                
                    % issue the move command
                    waypoint = GPS_Coord.Cart2GPS_struct(area_config.DEM.ref, plot_tool.fig_handle.UserData.waypoint);
                    % update waypoint alt to relative altitude
                    waypoint.relative_alt = uav_altitude;
                    waypoint.hdg = atan2d(plot_tool.fig_handle.UserData.waypoint(1) - uav(1,t), plot_tool.fig_handle.UserData.waypoint(2) - uav(2,t));
                    comm.move_rotate(waypoint, yaw_rate);
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
        plot_tool.update_uav(uav(:, 1:t));
        plot_tool.update_target(truth);
        plot_tool.update_text(truth);
        drawnow;
    end
    
    % wait until looptime reach 1 second
    while toc(loop_time) < 1
        pause(0.05);
    end
    
end

% Show final GPS coordinate

% send UAV back to home position
% adjust heading
% home.hdg = rad2deg(wrapTo2Pi(atan2(uav0(1) - uav(1,t), uav0(2) - uav(2,t))));
% comm.move_global(home);

% close tcp connection to mavlink-proxy
comm.close_tcp();

% save uav path
recorder.stats.uav = uav;
recorder.stats.measurement = recorder.stats.measurement(1:t);

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

%% ====================================================================

