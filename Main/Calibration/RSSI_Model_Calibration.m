%% Main problem for SITL
clear; close all;


%% Global configuration
Total_time = 2000;                      % maximun simulation time
ntarget = 1;                            % number of targets
altitude = 30;

% measurement record


%% Useful intermedia variable
uav = nan(4, Total_time);
action_end = false;

%% Initialization
addpath(genpath(pwd));

% generate module settings
RSSI_sensor_config = init_RSSI_sensor_config( ...
    'Antenna',          Antenna_Type.H, ... 
    'Sensitivity',      -135, ...
    'Sigma_RSSI',       6, ...
    'Path_Loss',        3, ...
    'P0',               40 ...
);

uav_config = init_uav_config('vmax', 5);  

area_config = init_area_config( ...
                [1509.3, 731.2; 1343.9, 479.4; 1053.8, 559.3; 1176.0, 869.3; 1509.3, 731.2]', ...
                'DEM', 'Victor_alt_site'); 


           

% initialize each modules
% movement
target_id_list = [1];
target_truth = [-34.538399; 139.590319];
uav_gps = [-34.53946; 139.5898881];
target_xy = GPS_Coord.GPS2Cart([area_config.DEM.ref.lat; area_config.DEM.ref.lon], target_truth(:,1));
move_direction = 18.6;                 % heading when moving (degree)
target_waypoint = [target_xy(1); target_xy(2); altitude; deg2rad(move_direction)];



% =========== [System modules] ===============
RSSI_model = RSSI_Model(RSSI_sensor_config);
UAV_Action = UAV_Actions(uav_config, area_config);

% =========== [Data recording] ====================
record.meas = cell(Total_time, 1);
record.uav = nan(4, Total_time);


%% ====================== SITL initialization ============================
comm = UAV_Comm('filter_config_udoo.json', 'Debug', true, 'Mode', 'UDOO');


%% ======================= Main Program =============================
main_start_time = tic;
proc_time = 1;

% convert target truth to cartesian coordinate
truth = nan(3, ntarget);
for n = 1:1
    truth(1:2, n) = GPS_Coord.GPS2Cart([area_config.DEM.ref.lat; area_config.DEM.ref.lon], target_truth(:, n));
    truth(3, n) = 0.8;
end

% get home position & initialize state
home = comm.get_home_pos();
% initialize initial position
uav0 = GPS_Coord.GPS2Cart_struct(area_config.DEM.ref, home);
warning('The elevation in uav state is relative elevation, not AMSL.');

% executue move command
comm.yaw(move_direction); pause(3);
waypoint = GPS_Coord.Cart2GPS_struct(area_config.DEM.ref, target_waypoint);
comm.move(waypoint, false);


% ======================== Main Loop ================================
t = 0;
while (t <= Total_time) && ~action_end
    fprintf('Epos: %i/%i.\n', t, Total_time);
    loop_time = tic;
    t = t+1;    % increment simulation time
    
    comm.check_sdr_config();        % check if SDR config is consistent
     
    % ============= update uav location ========================
    current_gps = comm.get_gps();
    uav(:,t) = GPS_Coord.GPS2Cart_struct(area_config.DEM.ref, current_gps(end, :));
    
    % ============= generate measurement =======================
    pulses = comm.get_pulses();

    z = convert_pulses(pulses, home);
    
    disp(z);
    
    % ====================== Collecting data ====================
    record.meas{t} = z;
    idx = find(z.Target_ID == target_id_list);
    if ~isempty(idx)
        record.uav(:, t) = z.UAV(idx, :)';
    end
       
    
    % ==================== Time ticking ====================
    proc_time = toc(main_start_time);
    
    % check whether UAV reach waypoint
    if sqrt(sum((uav(1:2, t) - target_waypoint(1:2, :)).^2)) < 5
        action_end = true;
    end
    
    
    % wait until looptime reach 1 second
    while toc(loop_time) < 1
        pause(0.05);
    end
    
end

% Save UAV Path
record.uav = uav(:, 1:t);
record.meas = record.meas(1:t);


%% ====================================================================


% send UAV back to home position
% adjust heading
home.hdg = rad2deg(wrapTo2Pi(atan2(uav0(1) - uav(1,t), uav0(2) - uav(2,t))));
comm.move(home);

% close tcp connection to mavlink-proxy
comm.close_tcp();

% extract measurement
meas = nan(t, 1);
d = nan(t,1);
uav = uav(:, 1:t);

% get antenna gain 
gain = RSSI_model.get_gain_dist(truth, uav)';
for i = 1:t
    meas(i) = record.meas{i}.RSSI;
    d(i) = sqrt(sum((uav(1:2, i) - truth(1:2,:)).^2));
end

fitfun  = fittype(@(p0, pl, x) p0-10*pl*log10(x) + fix_size(gain, length(x)));
fitted_curve = fit(d, meas, fitfun, 'StartPoint', [30, 2]);
scatter(d, meas); hold on; plot(d, fitted_curve(d));
disp(fitted_curve);
