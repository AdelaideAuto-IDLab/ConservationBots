%% Main problem for SITL
clear; close all;


%% Global configuration
% list of truth location
target_id_list = 1;
target_truth = [-35.459131; 138.515419];
ntarget = size(target_truth, 2);
rotate_count = 60;
avg_num = 8;
d_theta = 360/rotate_count;

raw_meas_record = cell(100, ntarget);

%% Useful intermedia variable
uav = nan(4, rotate_count*avg_num);
meas = nan(rotate_count, 1);
snr = nan(rotate_count, 1);
theta = nan(rotate_count, 1);

%% Initialization
addpath(genpath(pwd));

% generate module settings
uav_config = init_uav_config('vmax', 2);  
            
% initialize each modules

% % ========== Plotting =======================
figure();
hold on;
h_meas = stem(nan,nan);
title('Average measurement');
grid on;
ylabel('RSSI (dB)');

%% ====================== SITL initialization ============================
comm = UAV_Comm('filter_config_udoo.json', 'Debug', true);

%% ======================= Main Program =============================

pause(0.5);
% get home position & initialize state
home = comm.get_home_pos();
% initialize initial position
uav0 = GPS_Coord.GPS2Cart_struct(home, home);
warning('The elevation in uav state is relative elevation, not AMSL.');

% convert target truth to cartesian coordinate
truth = nan(2, ntarget);
for n = 1:ntarget
    truth(:, n) = GPS_Coord.GPS2Cart([home.lat; home.lon], target_truth(:, n));
end
truth(3,:) = 3;

% ======================== Main Loop ================================

t = 1;
tmp_meas = nan(avg_num, 1);
tmp_snr = nan(avg_num, 1);
tmp_uav = nan(4, avg_num);
input('Press any key to start.\n');
while t <= rotate_count
    comm.yaw(t*d_theta);
    pause(2);
    
    theta(t) = t*d_theta;
    i = 1;
    while (i <= avg_num)
        % ============= get measurement =======================
        pulses = comm.get_pulses();
        disp(pulses);
        z = convert_pulses(pulses, home);
        z = remove_invalid_id(z, target_id_list);
        target_idx = find(z.Target_ID == target_id_list(1));
        if ~isempty(target_idx)
            tmp_meas(i, 1) = z.RSSI(target_idx);
            tmp_snr(i, 1) = z.SNR(target_idx);
            tmp_uav(:, i) = z.UAV(target_idx)';
            i = i + 1;
        end
    end
    meas(t, 1) = mean(tmp_meas);
    snr(t, 1) = mean(tmp_snr);
    uav(:, t) = mean(tmp_uav, 2);
 
       
    set(h_meas, 'XData', (1:t), 'YData', meas(1:t, 1));
    drawnow;
    pause(0.01);
    
    t = t+1;    % increment simulation time
    
end

%% ====================================================================

save_prompt = input('Save Result? (Y/N)\n', 's');
if strcmp(save_prompt, 'Y')
    filename = input('Enter Filename: \n', 's');
    if ~isempty(filename)
        filename = strcat('Victor_farm_flat_1/', filename, '-', datestr(now, 'mm-dd-HH_MM'), '.mat');
        save(filename);
    else
        error('Invalid filename, file NOT saved.\n');
    end
end

% close tcp connection to mavlink-proxy
comm.close_tcp();


%     %% show angle difference
%     angle_error = nan(size(bearing_meas));
%     for n = 1:size(bearing_meas, 1)
%         angle_error(n, :) = angdiff(bearing_meas(n,:), bearing_truth(n,:)); 
%     end
%     figure();
%     print(rad2deg(angle_error));
%     ylabel('Angle Error (deg)');
