%% Main problem for SITL
clc; clear; close all;


%% Global configuration
Total_time = 2000;                      % maximun simulation time
ntarget = 1;                            % number of targets
altitude = 10;


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
                [368.8, 1658.3; 672.4, 1819.5; 2193.6, 722.6; 2077.6, 466.7; 731.6, 589.8; 368.8, 1658.3]', ...
                'DEM', 'Kooloka_DEM'); 


            
sim_sensor_config = init_sim_sensor_config(...
    'Antenna'    ,      Antenna_Type.H,...
    'Pd'         ,      0.99,...
    'Sensitivity',      -135,...
    'Sigma'      ,      3 ...
 );

sim_target_radio_config = init_sim_target_radio_config( ...
    'NTarget',      ntarget, ...
    'P0',           10,...
    'Frequency',    150.13e6,...
    'Path_Loss',    3.5,...
    'Tree_Height',  0 ...
 );

% initialize each modules
% movement
target_truth = [-34.537513; 139.590448];
target_xy = GPS_Coord.GPS2Cart([area_config.DEM.ref.lat; area_config.DEM.ref.lon], target_truth);
move_direction = 291.8;                 % heading when moving (degree)
target_waypoint = [target_xy(1); target_xy(2); altitude; deg2rad(move_direction)];



% =========== [System modules] ===============
RSSI_model = RSSI_Model(RSSI_sensor_config);
UAV_Action = UAV_Actions(uav_config, area_config);

RSSI_generator = RSSI_Generator(sim_sensor_config, sim_target_radio_config, area_config.DEM.DATA);

% =========== [Data recording] ====================
record.meas = cell(Total_time, 1);
record.uav = [];


%% ====================== SITL initialization ============================
comm = UAV_Comm('filter_config_udoo.json', 'Debug', true);
comm.request_gps_sitl(5);

%% ======================= Main Program =============================
main_start_time = tic;
proc_time = 1;

% convert target truth to cartesian coordinate
truth = GPS_Coord.GPS2Cart([area_config.DEM.ref.lat; area_config.DEM.ref.lon], target_truth);
truth = [truth; 0.2];

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
     
    % ============= update uav location ========================
    current_gps = comm.get_gps();
    uav(:,t) = GPS_Coord.GPS2Cart_struct(area_config.DEM.ref, current_gps(end, :));
    
    % ============= generate measurement =======================
    z = RSSI_generator.get_RSSI_SITL_debug(truth, uav(:, t), current_gps.time_boot_ms(end)); 
    
   
    
    % ====================== Collecting data ====================
    record.meas{t} = z;
    
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

function out = fix_size(gain, size)
    gain_l = length(gain);
    if size > gain_l
        out = [gain; ones(size-gain_l, 1)];
    else
        out = gain(1:size, 1);
    end
end

