%% Main problem for SITL
clear; close all;


%% Global configuration
Total_time = 1000;                      % maximun simulation time
DEM_file = 'perfect_flat_DEM.mat';      % load DEM data
target_truth_gps = [];                  % enter target truth [lat; lon]

ntarget = 10;                           % number of targets
%% Useful intermedia variable
uav = nan(4, Total_time);

%% Initialization
addpath(genpath(pwd));


% initialize each modules
% =========== [System modules] ===============
% setup sensor config - system
RSSI_sensor_config = init_RSSI_sensor_config( ...
    'Antenna',          Antenna_Type.H, ... 
    'Sensitivity',      -135, ...
    'Sigma_RSSI',       6, ...
    'Path_Loss',        3*ones(1, ntarget), ...
    'P0',               40*ones(1, ntarget), ...
    'Likelihood_Type',  Likelihood_Type.precise ...
);

% area config
area_config = init_area_config( ...
                [1,1; 1, 2000; 2000, 2000; 2000, 1; 1,1]', ...
                'DEM', DEM_file);

% =========== [Data recording] ====================
% recorder = Statistics(ntarget, (1:ntarget)', Total_time, config, 'Simulation');




%% ====================== SITL initialization ============================
comm = UAV_Comm('filter_config_udoo.json', 'Debug', false);


%% ======================= Main Program =============================
main_start_time = tic;
proc_time = 1;

% get home position & initialize state
home = comm.get_home_pos();
% initialize initial position
uav0 = GPS_Coord.GPS2Cart_struct(DEM.ref, home);
warning('The elevation in uav state is relative elevation, not AMSL.');


% ======================== Main Loop ================================
% Move UAV
% [TODO] move

t = 0;
while (t <= Total_time) && (proc_time < Total_time) && ~all_target_found
    fprintf('Epos: %i/%i.\n', t, Total_time);
    loop_time = tic;
    t = t+1;    % increment simulation time
    
    comm.check_sdr_config();        % check if SDR config is consistent
     
    % ============= update uav location ========================
    current_gps = comm.get_gps_and_pulses();
    uav(:,t) = GPS_Coord.GPS2Cart_struct(DEM.ref, current_gps(end, :));
    
    % ============= get measurement =======================
    % [TODO] get measurement
    % z = get_meas();
    
    % ====================== Collecting data ====================
    recorder.record(z, t);
    
    
    % ==================== Time ticking ====================
    proc_time = toc(main_start_time);
    
    
    % wait until looptime reach 1 second
    while toc(loop_time) < 1
        pause(0.05);
    end
    
end

% finishing
% ========================================================

% save uav path
recorder.record_uav_path(uav);
%% ====================================================================


% send UAV back to home position
% adjust heading
home.hdg = rad2deg(wrapTo2Pi(atan2(uav0(1) - uav(1,t), uav0(2) - uav(2,t))));
comm.move(home);

% close tcp connection to mavlink-proxy
comm.close_tcp();



