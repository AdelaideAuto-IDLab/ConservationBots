%% Main problem for SITL
clc; clear; close all;


%% Global configuration
Total_time = 30;                      % maximun simulation time
altitude = 10;
target_id_list = [1, 2];
target_truth = [-34.5402, -34.5402; 139.5918, 139.5918];
ntarget = size(target_truth, 2);


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

area_config = init_area_config( ...
                [0,0; 0, 2000; 2000, 2000; 2000, 0]', ...
                'DEM', 'Kooloka_DEM'); 


            
sim_sensor_config = init_sim_sensor_config(...
    'Antenna'    ,      Antenna_Type.H,...
    'Pd'         ,      0.9,...
    'Sensitivity',      -135,...
    'Sigma'      ,      3 ...
 );

sim_target_radio_config = init_sim_target_radio_config( ...
    'NTarget',      ntarget, ...
    'P0',           10,...
    'Frequency',    150e6 + 0.1e6*ones(1, ntarget),...
    'Path_Loss',    3.5*ones(1,ntarget),...
    'Tree_Height',  0*ones(1,ntarget) ...
 );


% =========== [System modules] ===============
RSSI_model = RSSI_Model(RSSI_sensor_config);

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

% get home position & initialize state
pause(0.5);
home = comm.get_home_pos();
% initialize initial position
uav0 = GPS_Coord.GPS2Cart_struct(area_config.DEM.ref, home);
warning('The elevation in uav state is relative elevation, not AMSL.');

% convert target truth to cartesian coordinate
truth = nan(2, ntarget);
for n = 1:ntarget
    truth(:, n) = GPS_Coord.GPS2Cart([area_config.DEM.ref.lat; area_config.DEM.ref.lon], target_truth(:, n));
end
truth(3,:) = 0.8;


warning('The elevation in uav state is relative elevation, not AMSL.');

% ======================== Main Loop ================================
t = 0;
while (t <= Total_time)
    fprintf('Epos: %i/%i.\n', t, Total_time);
    loop_time = tic;
    t = t+1;    % increment simulation time
     
    % ============= update uav location ========================
    current_gps = comm.get_gps();
    uav(:,t) = GPS_Coord.GPS2Cart_struct(area_config.DEM.ref, current_gps);
    
    % ============= generate measurement =======================
    z = RSSI_generator.get_RSSI_SITL_debug(truth, uav(:, t), current_gps.time_boot_ms(end)); 
    disp(z);
    
    % ====================== Collecting data ====================
    record.meas{t} = z;
    
    % ==================== Time ticking ====================
    proc_time = toc(main_start_time);
    
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
meas = nan(ntarget, t);
d = nan(ntarget,t); % ground distance
uav = uav(:, 1:t);

% get antenna gain  & extract measurement
gain = nan(length(uav), ntarget);
for n = 1:ntarget
    gain(:, n) = RSSI_model.get_gain_dist(truth(:, n), uav)';
end

for i = 1:length(record.meas)
    for n = 1:ntarget
        target_idx = find(record.meas{i}.Target_ID == target_id_list(n));
        if ~isempty(target_idx)
            meas(n, i) = record.meas{i}.RSSI(target_idx);
            d(n, i) = sqrt(sum((uav(1:3, i) - truth(1:3, n)).^2));
        else
            gain(i, n) = nan; 
        end
    end
end


fitted_curve = cell(ntarget,1);


for n = 1:ntarget
    % remove nan
    d_fit = d(n, :)'; d_fit = d_fit(~isnan(d_fit));
    meas_fit = meas(n,:)'; meas_fit = meas_fit(~isnan(meas_fit));
    gain_fit = gain(:, n); gain_fit = gain_fit(~isnan(gain_fit));

    fitfun  = fittype(@(p0, pl, x) p0-10*pl*log10(x) + fix_size(gain_fit, length(x)));
    fitted_curve{n} = fit(d_fit, meas_fit, fitfun, 'StartPoint', [30, 2]);
    figure();
    plot(fitted_curve{n}, d_fit, meas_fit);
    disp(fitted_curve{n});
end


function out = fix_size(gain, size)
    gain_l = length(gain);
    if size > gain_l
        out = [gain; ones(size-gain_l, 1)];
    else
        out = gain(1:size, 1);
    end
end

