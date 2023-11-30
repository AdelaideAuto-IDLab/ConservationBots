bearing_sample = 20;
noise = 4;
repeat_time = 100;
dist_range = [10, 300];

AoA_config = init_bearing_config('Antenna', Antenna_Type.H);
AoA_Generator = Rotation_Bearing(AoA_config);

sim_target_radio_config = init_sim_target_radio_config( ...
    'NTarget'    ,   1, ...
    'P0'         ,   40,...
    'Frequency'  ,   150e6,...
    'Path_Loss'  ,   4,...
    'Tree_Height',   0 ...
 );

sim_sensor_config = init_sim_sensor_config(...
    'Antenna'    ,      Antenna_Type.H,...
    'Pd'         ,      0.8,...
    'Sensitivity',      -135,...
    'Sigma'      ,      noise ...
 );

RSSI_generator = RSSI_Generator(sim_sensor_config, sim_target_radio_config, ones(2000, 2000));

area_config = init_area_config([1,1; 1, 2000; 2000, 2000; 2000, 1; 1,1]');
uav_config = init_uav_config( ...  
    'traject_time', 200, ...
    'rotation_time', bearing_sample ...
);  

UAV_Action = UAV_Actions(uav_config, area_config);

% generate UAV action
uav0 = [500,500,30,pi/4]';
uav = UAV_Action.get_action_rotation(uav0, 2*pi);
x0 = [500, 500, 1.2]';
truth = nan(repeat_time, 1);

bearing = nan(repeat_time,1);
corr_p = nan(repeat_time,1);
for n = 1:repeat_time
    % randomize target location
    x = x0;
    rand_ang = 2*pi*rand();
    dist = dist_range(1) + diff(dist_range)*rand();
    x(1:2) = x(1:2) + dist*[sin(rand_ang); cos(rand_ang)];
    truth(n) = atan2(x(1) - uav0(1), x(2) - uav0(2));
    
    AoA_Generator.start_time = 0;
    AoA_Generator.end_time = bearing_sample;
    for i = 1:bearing_sample
       z = RSSI_generator.get_RSSI_debug(x, uav(:, i));  
       AoA_Generator.record_data(z, i);
    end

    b = AoA_Generator.get_bearing();
    bearing(n) = b.AoA(1);
    corr_p(n) = b.Coef(1);
end


ang_diff = angdiff(bearing, truth);
valid_i = (abs(ang_diff) < deg2rad(50));


ang_var = var(ang_diff(valid_i));
ang_mean = mean(ang_diff(valid_i));
err_rate = 1-mean(valid_i);

fprintf('Error Mean: %f \n', ang_mean);
fprintf('Error Variance: %f \n', ang_var);
fprintf('Error Rate: %f \n', err_rate);

figure(); plot(rad2deg(ang_diff));
