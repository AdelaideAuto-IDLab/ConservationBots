% Main problem for SITL
clear; close all;


%% Global configuration
Total_time = 2000;                      % maximun simulation time
ntarget = 1;                            % number of targets

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
                'DEM', 'Kooloka_DEM'); 


           

% initialize each modules
% movement
target_truth = repmat([-34.538399; 139.590319],1,ntarget);

% =========== [System modules] ===============
RSSI_model = RSSI_Model(RSSI_sensor_config);

% =========== [Data recording] ====================
record.meas = cell(Total_time, 1);
record.uav = nan(4, Total_time);


%% ====================== SITL initialization ============================
comm = UAV_Comm('filter_config_udoo.json', 'Debug', true);


%% ======================= Main Program =============================
main_start_time = tic;
proc_time = 1;

% convert target truth to cartesian coordinate
truth = nan(3, ntarget);
for n = 1:ntarget
    truth(1:2, n) = GPS_Coord.GPS2Cart([area_config.DEM.ref.lat; area_config.DEM.ref.lon], target_truth(:, n));
    truth(3, n) = 0.2;% + area_config.DEM.DATA(round(truth(1,n)), round(truth(2,n))); 
end

% get home position & initialize state
home = comm.get_home_pos();
% initialize initial position
uav0 = GPS_Coord.GPS2Cart_struct(area_config.DEM.ref, home);
warning('The elevation in uav state is relative elevation, not AMSL.');



% ======================== Main Loop ================================
t = 0;
while (t <= Total_time) 
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
    record.uav(:, t) = uav(:, t);
    
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

% close tcp connection to mavlink-proxy
comm.close_tcp();


% save record
filename = strcat('Kooloka_results/', 'RSSI_Tuning_receive_test' , datestr(now,'mm-dd-HH_MM'), '.mat');
save(filename);


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

uav(3,:) = 20;
d = d(95:end); meas = meas(95:end);

tmp = reflection(d);

fitfun  = fittype(@(p0, pl, x) p0-10*pl*log10(x) + fix_size(gain, length(x))  + pl*reflection(x));
fitted_curve = fit(d, meas, fitfun, 'StartPoint', [30, 2]);
scatter(d, meas); hold on; plot(d, fitted_curve(d));
disp(fitted_curve);



function r = reflection(d)
    % wavelength
    l = 3e8/(150e6);
    
    % uav height
    uh = 20;
    % target height
    th = 0.8;
    % relative permittivity
    rel_p = 30;
    
    % path differength
    pd = sqrt((uh + th).^2 + d.^2) - sqrt((uh - th).^2 + d.^2);
    
    % phase difference
    phase_d = 2*pi*pd/l;
    
    % incident angle
    inc = atan2(uh+th, d);
    
    % reflection coefficient
    ref_c = (sin(inc) - sqrt(rel_p - cos(inc).^2)) ./ (sin(inc) + sqrt(rel_p - cos(inc).^2));
    
    r = 10*log10(abs(1 + ref_c.*exp(-1j*phase_d)));
    
%     r = 0;
end
