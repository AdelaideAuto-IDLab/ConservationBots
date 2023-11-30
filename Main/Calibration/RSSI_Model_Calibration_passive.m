%% Main problem for SITL
clear; close all;

%% Global configuration
start_date = datetime();
Total_time = 60;                      % maximun simulation time
altitude = 50;
target_id_list = [7,8];
% target_truth = [-34.544006, 139.589998; -34.543942, 139.590086; -34.544006, 139.589998; ...
%                 -34.543934, 139.590453; -34.544006, 139.589998; -34.543942, 139.590086; ...
%                 -34.543934, 139.590453]';   % known tags
target_truth = [nan, nan; nan, nan]';
ntarget = size(target_truth, 2);
DEM = 'Swan_Reach_DEM';


%% Useful intermedia variable
uav = nan(4, Total_time);
action_end = false;

%% Initialization
addpath(genpath(pwd));

% generate module settings
RSSI_sensor_config = init_RSSI_sensor_config( ...
    'Antenna',          Antenna_Type.H, ... 
    'Sensitivity',      -90, ...
    'Sigma_RSSI',       6, ...
    'Path_Loss',        3, ...
    'P0',               40 ...
);
area_gps = get_gps_boundary('Swan_Reach_full_area');

area_config = init_area_config( ...
                'GPS', area_gps, ...
                'DEM', DEM); 


% =========== [System modules] ===============
RSSI_model = RSSI_Model(RSSI_sensor_config);

% =========== [Data recording] ====================
record.meas = cell(Total_time, 1);
record.uav = [];


%% ====================== SITL initialization ============================
comm = UAV_Comm('filter_config_udoo.json', 'Debug', true);


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
truth(3,:) = 0.2;


warning('The elevation in uav state is relative elevation, not AMSL.');


% get to last waypoint
% issue the move command
last_uav = [0.0, 0.0, 0.0, 0.0]';
waypoint = GPS_Coord.Cart2GPS_struct(area_config.DEM.ref, last_uav);
waypoint.relative_alt = 50;
comm.move(waypoint)
comm.yaw(rad2deg(last_uav(4)));

% ======================== Main Loop ================================
% flush old pulses
comm.get_pulses();


t = 0;
while (t < Total_time)
    fprintf('Epos: %i/%i.\n', t, Total_time);
    loop_time = tic;
    t = t+1;    % increment simulation time
     
    % ============= update uav location ========================
    current_gps = comm.get_gps();
    while isempty(current_gps)
        current_gps = comm.get_gps();
    end
    
    if ~isempty(current_gps)
        uav(:,t) = GPS_Coord.GPS2Cart_struct(area_config.DEM.ref, current_gps);
    end
    % ============= generate measurement =======================
    pulses = comm.get_pulses();
    disp(pulses);
    z = convert_pulses(pulses, area_config.DEM.ref);
        
    
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
record.uav = uav;


%% =========================================================
% close tcp connection to mavlink-proxy
comm.close_tcp();

% extract measurement
meas = nan(ntarget, t);
d = nan(ntarget,t); % ground distance
snr = nan(ntarget, t);
uav = uav(:, 1:t);

% get antenna gain  & extract measurement
gain = nan(length(uav), ntarget);
for n = 1:ntarget
    gain(:, n) = RSSI_model.get_gain_dist(truth(:, n), uav)';
end

for i = 1:length(record.meas)
    for n = 1:ntarget
        if ~isempty(record.meas{i})
            target_idx = find(record.meas{i}.Target_ID == target_id_list(n));
            if ~isempty(target_idx)
                [meas(n, i), m_i ]= max(record.meas{i}.RSSI(target_idx));
                snr(n,i) = record.meas{i}.SNR(target_idx(m_i));
                d(n, i) = sqrt(sum((uav(1:3, i) - truth(1:3, n)).^2));
            else
                gain(i, n) = nan; 
            end
        end
    end
end


fitted_curve = cell(ntarget,1);

% % fixed path loss
% pl = 2;
% for n = 1:ntarget
%     % remove nan
%     d_fit = d(n, :)'; d_fit = d_fit(~isnan(d_fit));
%     meas_fit = meas(n,:)'; meas_fit = meas_fit(~isnan(meas_fit));
%     gain_fit = gain(:, n); gain_fit = gain_fit(~isnan(gain_fit));
% 
%     fitfun  = fittype(@(p0, x) p0-10*pl*log10(x) + fix_size(gain_fit, length(x)));
%     fitted_curve{n} = fit(d_fit, meas_fit, fitfun, 'StartPoint', [30]);
%     figure();
%     plot(fitted_curve{n}, d_fit, meas_fit);
%     disp(fitted_curve{n});
% end


for n = 1:ntarget
    % remove nan
    d_fit = d(n, :)'; d_fit = d_fit(~isnan(d_fit));
    meas_fit = meas(n,:)'; meas_fit = meas_fit(~isnan(meas_fit));
    gain_fit = gain(:, n); gain_fit = gain_fit(~isnan(gain_fit));

    fitfun  = fittype(@(p0, pl, x) p0-10*pl*log10(x) + 0*fix_size(gain_fit, length(x)));
    fitted_curve{n} = fit(d_fit, meas_fit, fitfun, 'StartPoint', [30, 2]);
    figure();
    plot(fitted_curve{n}, d_fit, meas_fit);
    disp(fitted_curve{n});
end
% 
%% Save result
save_prompt = input('Save Result? (Y/N)\n', 's');
if strcmp(save_prompt, 'Y')
    filename = input('Enter Filename: \n', 's');
    if ~isempty(filename)
        filename = strcat('Swan_Reach_test2/', filename, '-', datestr(now, 'mm-dd-HH_MM'), '.mat');
        save(filename);
    else
        error('Invalid filename, file NOT saved.\n');
    end
end


function out = fix_size(gain, size)
    gain_l = length(gain);
    if size > gain_l
        out = [gain; ones(size-gain_l, 1)];
    else
        out = gain(1:size, 1);
    end
end





