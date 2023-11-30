%% Main problem for SITL
clear; close all;


%% Global configuration
Total_time = 1000;                      % maximun simulation time
altitude = 50;
% list of truth location
target_id_list = [1,2,3,4];
%target_truth = [-35.469361, -35.469361, -35.469361; 138.489903, 138.489903, 138.489903];
target_truth = repmat([-35.459357;138.515037], 1, length(target_id_list));
ntarget = size(target_truth, 2);
rotate_time = 10;

rotation_count = 10;                    % total number of rotations


bearing_meas = nan(1000, ntarget);
bearing_truth = nan(1000, ntarget);
pattern_record = cell(100, ntarget);
raw_meas_record = cell(100, ntarget);
AoA_table_record = cell(100, 1);

full_meas_record = cell(100, 1);

%% Useful intermedia variable
uav = nan(4, Total_time);
action_end = false;

%% Initialization
addpath(genpath(pwd));

% generate module settings
uav_config = init_uav_config('vmax', 2);  

AoA_config = init_bearing_config( ...
    'Interpolate_factor', 10, ...
    'Antenna', Antenna_Type.H, ...
    'Samples', rotate_time ...
    );
            
% initialize each modules

% % ========== Plotting =======================
figure();
hold on;
h_meas = stem(nan,nan);
h_gain_pattern = plot(nan,nan);
legend('Measurement', 'Gain Pattern');

% =========== [System modules] ===============
AoA_Generator = Rotation_Bearing(AoA_config);


%% ====================== SITL initialization ============================
comm = UAV_Comm('filter_config_udoo.json', 'Debug', true);


%% ======================= Main Program =============================
main_start_time = tic;
proc_time = 1;

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
truth(3,:) = 0.8;

% ======================== Main Loop ================================
t = 0;
r = 0;
rotate_fin = false;
AoA_Generator.start_time = 0;
AoA_Generator.end_time = rotate_time;
pause(0.5);
while r < rotation_count
    fprintf('Current rotation: %i \n', r);
    fprintf('Epos: %i/%i.\n', t, Total_time);
    loop_time = tic;
    t = t+1;    % increment simulation time 
     
    % ============= update uav location ========================
    current_gps = comm.get_gps();
    if ~isempty(current_gps)
        uav(:,t) = GPS_Coord.GPS2Cart_struct(home, current_gps);
    end
    
    % ============= generate measurement =======================
    pulses = comm.get_pulses();
    disp(pulses);
    z = convert_pulses(pulses, home);
    z = remove_invalid_id(z, target_id_list);
    
    AoA_Generator.record_data(z, t);
   
    % ============= rotate ====================
    rotate_fin = comm.rotate();
    if rotate_fin
        comm.rotate_reset();
        [bearing, pattern, raw_meas, meas] = AoA_Generator.get_bearing();
        disp(rad2deg(bearing.AoA));
        AoA_Generator.start_time = t;
        AoA_Generator.end_time = t + rotate_time;
        r = r + 1;
        rotate_fin = false;

        fprintf('\n Bearing Result: \n');
        disp(bearing);
        
        % record measurement
        AoA_table_record{r} = bearing;
        for n = 1:height(bearing)
            target_idx = (bearing.Target_ID(n) == target_id_list);
            bearing_truth(r,  target_idx) = atan2(truth(1, target_idx) - uav(1,t), truth(2, target_idx) - uav(2,t));
            if sum(target_idx > 0)
                pattern_record{r, target_idx} = pattern{target_idx};
            end
            raw_meas_record{r, target_idx} = raw_meas{target_idx};
            bearing_meas(r,target_idx) = bearing.AoA(target_idx);
        end
        
        
        % update plot
        m_size = length(meas{1});
        %normalize measurement and pattern
        n_meas = meas{1} - min(meas{1});
        n_meas = n_meas/max(n_meas);
        n_pattern = pattern{1} - min(pattern{1});
        n_pattern = n_pattern/max(n_pattern);   
        
        set(h_meas, 'XData', (1:m_size), 'YData', n_meas);
        set(h_gain_pattern, 'XData', linspace(1, m_size, length(n_pattern)), 'YData', n_pattern);
        drawnow;
        pause(0.01);

    end
    
    % ==================== Time ticking ====================
    proc_time = toc(main_start_time);
    
    % wait until looptime reach 1 second
    while toc(loop_time) < 1
        pause(0.05);
    end
    
    
    
end

% trim measurement
bearing_meas = bearing_meas(1:r, :);
bearing_truth = bearing_truth(1:r, :);
pattern_record = pattern_record(1:r, :);
raw_meas_record = raw_meas_record(1:r, :);
AoA_table_record = AoA_table_record(1:r, :);
full_meas_record = full_meas_record(1:r, :);
%% ====================================================================

% % save record
%filename = strcat('Results/', 'Bearing_Tuning_test-', num2str(rotate_time), 'T-', datestr(now,'mm-dd-HH_MM'), '.mat');
%save(filename);

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
