%% Main problem for SITL
clear; close all;

%% Global configuration
Total_time = 2000;                      % maximun simulation time
altitude = 50;
% list of truth location
target_truth = [-35.457732, -35.454822; 138.516642, 138.514561];
target_id_list = [1, 2];

ntarget = 2;
rotate_time = 20;
rotation_count = 10;                    % total number of rotations

bearing_meas = nan(rotation_count, ntarget);
bearing_truth = nan(rotation_count, ntarget);
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
uav_config = init_uav_config();   

DEM_file = 'Victor_hilly';
area_config = init_area_config( ...
                'GPS', get_gps_boundary('Victor_Harbor_Hill_good'), ...
                'DEM', DEM_file);     

AoA_config = init_bearing_config( ...
    'Interpolate_factor', 10, ...
    'Antenna', Antenna_Type.H, ...
    'Samples', rotate_time ...
    );
            
sim_sensor_config = init_sim_sensor_config(...
    'Antenna'    ,      Antenna_Type.H,...
    'Pd'         ,      0.9,...
    'Sensitivity',      -135,...
    'Sigma'      ,      2 ...
 );

sim_target_radio_config = init_sim_target_radio_config( ...
    'NTarget',      ntarget, ...
    'P0',           25*ones(1,ntarget),...
    'Frequency',    150e6 + 1e5 * (1:ntarget),...
    'Path_Loss',    2.5*ones(1,ntarget),...
    'Tree_Height',  1*ones(1,ntarget) ...
);

% initialize each modules

% ========== Plotting =======================
figure();
hold on;
h_meas = gobjects(ntarget, 1);
for n = 1:ntarget
    h_meas(n) = stem(nan, nan);
end
h_gain_pattern = plot(nan,nan);
legend('Measurement', 'Gain Pattern');
% =========== [System modules] ===============
UAV_Action = UAV_Actions(uav_config, area_config);

RSSI_generator = RSSI_Generator(sim_sensor_config, sim_target_radio_config, area_config.DEM.DATA);

AoA_Generator = Rotation_Bearing(AoA_config);



%% ====================== SITL initialization ============================
comm = UAV_Comm('filter_config_udoo.json', 'Debug', true);
comm.request_gps_sitl(5);

%% ======================= Main Program =============================
proc_time = 1;

% convert target truth to cartesian coordinate
truth = nan(2, ntarget);
for n = 1:ntarget
    truth(:, n) = GPS_Coord.GPS2Cart([area_config.DEM.ref.lat; area_config.DEM.ref.lon], target_truth(:, n));
end
truth(3,:) = 0.3;

% get home position & initialize state
home = comm.get_home_pos();
% initialize initial position
uav0 = GPS_Coord.GPS2Cart_struct(area_config.DEM.ref, home);
warning('The elevation in uav state is relative elevation, not AMSL.');



% ======================== Main Loop ================================
t = 0;
r = 0;
rotate_fin = true;
rotate_init_angle = nan;
AoA_Generator.start_time = 0;
AoA_Generator.end_time = rotate_time;
pause(0.5);
t_start = 0;

main_start_time = tic;
while r < rotation_count
    fprintf('Epos: %i/%i.\n', t, Total_time);
    loop_time = tic;
    t = t+1;    % increment simulation time
     
    % ============= update uav location ========================
    current_gps = comm.get_gps();
    if ~isempty(current_gps)
        uav(:,t) = GPS_Coord.GPS2Cart_struct(home, current_gps(end, :));
    end
    
    % ============= generate measurement =======================
    z = RSSI_generator.get_RSSI_SITL_debug(truth, uav(:,t), current_gps.time_boot_ms(end)); 
    disp(z);
    AoA_Generator.record_data(z, t);
   
    % ============= rotate ====================
    if rotate_fin
        t_start = t;
        comm.turns(1);
        rotate_fin = false;
        rotate_init_angle = uav(4,t);
        
        AoA_Generator.start_time = t;
        AoA_Generator.end_time = t + rotate_time;
    end
    
    if ~rotate_fin  % rotation not finished
        if (t - t_start > 1.1*rotate_time) || ...
           ((t - t_start > 0.8*rotate_time) && abs(angdiff(uav(4,t), rotate_init_angle)) < deg2rad(5))
            rotate_fin = true;
            rotate_init_angle = nan;
            r = r + 1;
        end
    end
    
    if rotate_fin && r ~= 0
        [bearing, pattern, raw_meas, meas, bearing_uav] = AoA_Generator.get_bearing();
        fprintf('\n Bearing Result: \n');
        disp(bearing);
        
        % record measurement
        try
        rotate_meas(r,:) = meas;
        catch e
            disp(e);
        end
        
        AoA_table_record{r} = bearing;
        raw_meas_record{r} = raw_meas;
        for n = 1:height(bearing)
            target_idx = (bearing.Target_ID(n) == target_id_list);
            bearing_truth(r,  target_idx) = atan2(truth(1, target_idx) - uav(1,t), truth(2, target_idx) - uav(2,t));
            if sum(target_idx > 0)
                pattern_record{r, target_idx} = pattern{target_idx};
            end
            bearing_meas(r,target_idx) = bearing.AoA(target_idx);
        end
        
        %% Plotting
        try
        for n = 1:ntarget
            try
            set(h_meas(n), 'XData', wrapTo2Pi(unwrap(bearing_uav(4,:))), 'YData', normalize(meas{n}, 'range')); 
            catch e
            end
        end  
        set(h_gain_pattern, 'XData', wrapTo2Pi(unwrap(bearing_uav(4,:))), length(pattern{1})), 'YData', normalize(pattern{1}, 'range'));
        drawnow;
        catch
        end
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


%% ====================================================================
% close tcp connection to mavlink-proxy
comm.close_tcp();

%% report result
% show bearin error and variance
ang_error = angdiff(bearing_meas, bearing_truth);
fprintf('Mean bearing error: \n');
disp(rad2deg(mean(abs(ang_error))));
fprintf('Variance: \n');
disp(var(ang_error));

% show average detection rate for each rotation
detect_rate = cellfun(@(x) sum(~isnan(x))/length(x), rotate_meas);
fprintf('Mean detect rate: \n');
disp(mean(detect_rate));

% show overall bearing detection
bearing_pd_threshold = 0.6;
fprintf('Bearing Pd:\n');
disp(mean(detect_rate>bearing_pd_threshold));


