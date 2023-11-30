%% Main problem for SITL
clear; close all;

%% Global configuration
Total_time = 150;                      % maximun simulation time
altitude = 50;
target_id_list = [1];
target_truth = [-35.459131; 138.515419];
ntarget = 1;
DEM = 'Victor_hilly.mat';

%% Important %%
Path_loss_n = 2;

%% Useful intermedia variable
action_end = false;

%% Initialization
addpath(genpath(pwd));

% % generate module settings
% RSSI_sensor_config = init_RSSI_sensor_config( ...
%     'Antenna',          Antenna_Type.H, ... 
%     'Sensitivity',      -85, ...
%     'Sigma_RSSI',       6, ...
%     'Path_Loss',        3, ...
%     'P0',               40 ...
% );

area_config = init_area_config( ...
                'Vertex', [1,1; 1, 1209; 1970, 1209; 1970, 1]', ...
                'DEM', DEM); 


% =========== [System modules] ===============
% RSSI_model = RSSI_Model(RSSI_sensor_config);


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
truth(3,:) = 0.1;


%% Plotting setup
f = figure(); 
hold on;
hp = scatter3(nan, nan, nan, 10, '*');
title('EIPR vs distance');
zlabel('Power (dB)');
grid minor;

meas = nan(ntarget, Total_time);
snr = nan(ntarget, Total_time);
uav = nan(ntarget, 4, Total_time);
theta = nan(ntarget, Total_time);
d = nan(ntarget, Total_time);
phi = nan(ntarget, Total_time);

% ======================== Main Loop ================================
t = 0;
while (t < Total_time)
    t = t+1;    % increment simulation time
    
    p1 = input('Start Measuring? [Y/N]\n', 's');
    if p1 == 'Y'
        p2 = input('Number of measurement:\n', 's');
        p2 = str2double(p2);
        meas_count = 1;
        
        tmp_meas = nan(p2, 1);
        tmp_snr = nan(p2, 1);
        tmp_uav = nan(4, p2);
        tmp_d = nan(p2, 1);
        tmp_gain = nan(p2, 1);
        
        while (meas_count <= p2) 
            % ============= get measurement =======================
            pulses = comm.get_pulses();
            disp(pulses);
            z = convert_pulses(pulses, area_config.DEM.ref);
            % ============ Data conversion ================================
            for n = 1:ntarget 
                target_idx = find(z.Target_ID == target_id_list(n));
                if ~isempty(target_idx)
                    tmp_meas(meas_count, 1) = z.RSSI(target_idx);
                    tmp_snr(meas_count, 1) = z.SNR(target_idx);
                    tmp_uav(:, meas_count) = z.UAV(target_idx, :)';
                    tmp_d(meas_count, 1) = sqrt(sum((tmp_uav(1:3, meas_count) - truth(1:3, n)).^2));
                    
                    meas_count = meas_count + 1;
                end
            end
        end
        
        meas(1, t) = mean(tmp_meas);
        snr(1, t) = mean(tmp_snr);
        uav(1, :, t) = mean(tmp_uav, 2);
        d(1, t) = mean(tmp_d);
        theta(1, t) = atan2(truth(1, 1) - squeeze(uav(1, 1, t)), truth(2, 1) - squeeze(uav(1, 2, t)) );
        phi(1, t) = atan2(truth(3, 1) - squeeze(uav(1, 3, t)), d(1,t));
        
    end

    % ==================== Update Plot =====================
    
    set(hp, 'XData', 10*sin(theta(1, :)), 'YData', 10*cos(theta(1,:)), 'ZData', meas(1, :) + 10*log10(d(1, :))*Path_loss_n );       
    
end



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

