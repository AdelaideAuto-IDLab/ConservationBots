%% Main problem for SITL
clear; close all;


%% Global configuration
Total_time = 500;                      % maximun simulation time
altitude = 1;
% list of truth location
target_id_list = [1];
ntarget = length(target_id_list);
rotate_time = 20;

rotation_count = 1;                    % total number of rotations

bearing_meas = nan(rotation_count, ntarget);
pattern_record = cell(rotation_count, ntarget);
raw_meas_record = cell(rotation_count, 1);
AoA_table_record = cell(rotation_count, 1);

rotate_meas = cell(rotation_count, ntarget);

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
h_meas = gobjects(ntarget, 1);
for n = 1:ntarget
    h_meas(n) = stem(nan, nan);
end
h_gain_pattern = plot(nan,nan);
legend('Measurement', 'Gain Pattern');

% =========== [System modules] ===============
AoA_Generator = Rotation_Bearing(AoA_config);

%% ====================== SITL initialization ============================
comm = UAV_Comm('filter_config_udoo.json', 'Debug', true);


%% ======================= Main Program =============================
proc_time = 1;

% get home position & initialize state
home = comm.get_home_pos();
% initialize initial position
uav0 = GPS_Coord.GPS2Cart_struct(home, home);
warning('The elevation in uav state is relative elevation, not AMSL.');

% ======================== Main Loop ================================
t = 0;
r = 0;
rotate_fin = false;
AoA_Generator.start_time = 0;
AoA_Generator.end_time = rotate_time;
for i = 1:3
    fprintf('Pause %i/3 \n', i);
    pause(1);
end
t_start = 0;

% flush old pulses
comm.get_pulses();
main_start_time = tic;
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
    if t - t_start < 3
        rotate_fin = false;
    elseif t - t_start == rotate_time+1
        rotate_fin = true;
    end
    if rotate_fin
        comm.rotate_reset();
        [bearing, pattern, raw_meas, meas] = AoA_Generator.get_bearing();
        disp(rad2deg(bearing.AoA));
        t_start = t;
        AoA_Generator.start_time = t;
        AoA_Generator.end_time = t + rotate_time;
        r = r + 1;
        rotate_fin = false;

        fprintf('\n Bearing Result: \n');
        disp(bearing);
        
        % record measurement
        try
        rotate_meas(r,:) = meas;
        catch e
            disp(e);
        end
        
        AoA_table_record{r} = bearing;
        for n = 1:height(bearing)
            target_idx = (bearing.Target_ID(n) == target_id_list);
            if sum(target_idx > 0)
                pattern_record{r, target_idx} = pattern{target_idx};
            end
            raw_meas_record{r} = raw_meas;
            bearing_meas(r,target_idx) = bearing.AoA(target_idx);
        end
        
        %% Plotting
        for n = 1:ntarget
            try
            % update plot
            m_size = length(meas{n});
            %normalize measurement and pattern
            n_meas = meas{n} - min(meas{n});
            n_meas = n_meas/max(n_meas);

            set(h_meas(n), 'XData', (1:m_size), 'YData', n_meas); 
            catch e
            end
        end
        n_pattern = pattern{1} - min(pattern{1});
        n_pattern = n_pattern/max(n_pattern);   
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
uav = uav(:,1:t);
%% ====================================================================

% close tcp connection to mavlink-proxy
comm.close_tcp();
