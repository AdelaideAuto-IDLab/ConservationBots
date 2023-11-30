%% Main problem for SITL
clear; close all;

%% Global configuration
Total_time = 60;                      % maximun simulation time
altitude = 50;
target_id_list = [1];
%target_truth = [-35.469421, -35.469421, -35.469421, -35.469421; 138.489673, 138.489673, 138.489673, 138.489673];
target_truth = repmat([-35.459131; 138.515419], 1, length(target_id_list));
ntarget = size(target_truth, 2);
DEM = 'Victor_alt_site.mat';

%% Important %%
Path_loss_n = 2;


%% Useful intermedia variable
action_end = false;

%% Initialization
addpath(genpath(pwd));

% generate module settings
RSSI_sensor_config = init_RSSI_sensor_config( ...
    'Antenna',          Antenna_Type.H, ... 
    'Sensitivity',      -85, ...
    'Sigma_RSSI',       6, ...
    'Path_Loss',        3, ...
    'P0',               40 ...
);

area_config = init_area_config( ...
                'Vertex', [1,1; 1, 1209; 1970, 1209; 1970, 1]', ...
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
truth(3,:) = 3;


warning('The elevation in uav state is relative elevation, not AMSL.');


%% Plotting setup
f = figure(); 
hold on;
hp = gobjects(ntarget, 1);
for i = 1:ntarget
    hp(i) = scatter(nan, nan, 10, '*');
end
title('EIPR vs distance');
xlabel('Distance (m)');
ylabel('Power (dB)');
grid minor;

meas = nan(ntarget, Total_time);
snr = nan(ntarget, Total_time);
d = nan(ntarget, Total_time);
gain = nan(ntarget, Total_time);
uav = nan(ntarget, 4, Total_time);

% ======================== Main Loop ================================
t = 0;
while (t < Total_time)
    fprintf('Epos: %i/%i.\n', t, Total_time);
    loop_time = tic;
    t = t+1;    % increment simulation time
     
    % ============= generate measurement =======================
    pulses = comm.get_pulses();
    disp(pulses);
    z = convert_pulses(pulses, area_config.DEM.ref);
    
    % ====================== Collecting data ====================
    record.meas{t} = z;
    
    % ============ Data conversion ================================
    for n = 1:ntarget 
        target_idx = find(record.meas{t}.Target_ID == target_id_list(n));
        if ~isempty(target_idx)
            meas(n, t) = record.meas{t}.RSSI(target_idx);
            snr(n,t) = record.meas{t}.SNR(target_idx);
            
            % update uav location
            uav(n, :, t) = record.meas{t}.UAV(target_idx,:)';
            d(n, t) = sqrt(sum((squeeze(uav(n, 1:3, t)') - truth(1:3, n)).^2));
            gain(n, t) = RSSI_model.get_gain_dist(truth(:, n), squeeze(uav(n, :, t)'));
        end
    end
    
    % ==================== Time ticking ====================
    proc_time = toc(main_start_time);
    
    % ==================== Update Plot =====================
    for n = 1:ntarget
        set(hp(n), 'XData', d(n, :), 'YData', meas(n, :) + 10*log10(d(n, :))*Path_loss_n );       
    end
    
    % wait until looptime reach 1 second
    while toc(loop_time) < 1
        pause(0.05);
    end
    
end

% Save UAV Path
record.uav = uav;


%% =========================================================

% % send UAV back to home position
% % adjust heading
% home.hdg = rad2deg(wrapTo2Pi(atan2(uav0(1) - uav(1,t), uav0(2) - uav(2,t))));
% comm.move(home);

% close tcp connection to mavlink-proxy
comm.close_tcp();

%% Save result
save_prompt = input('Save Result? (Y/N)\n', 's');
if strcmp(save_prompt, 'Y')
    filename = input('Enter Filename: \n', 's');
    if ~isempty(filename)
        filename = strcat('Victor_alt_site_2/', filename, '-', datestr(now, 'mm-dd-HH_MM'), '.mat');
        save(filename);
    else
        error('Invalid filename, file NOT saved.\n');
    end
end

