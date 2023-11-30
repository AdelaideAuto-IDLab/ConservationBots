% clear; close all;

RSSI_sensor_config = init_RSSI_sensor_config( ...
    'Antenna',          Antenna_Type.H, ... 
    'Sensitivity',      -135, ...
    'Sigma_RSSI',       0.01, ...
    'Path_Loss',        2.3, ...
    'P0',               10, ...
    'Likelihood_Type',  Likelihood_Type.precise ...
);

sim_sensor_config = init_sim_sensor_config(...
    'Antenna'    ,      Antenna_Type.H,...
    'Pd'         ,      0.99,...
    'Sensitivity',      -135,...
    'Sigma'      ,      0.01 ...
 );

sim_target_radio_config = init_sim_target_radio_config( ...
    'NTarget',      1, ...
    'P0',           10,...
    'Frequency',    150e6,...
    'Path_Loss',    2.3,...
    'Tree_Height',  0.1 ...
 );

area_config = init_area_config( ...
                'Vertex', [1,1; 1, 1200; 1970, 1200; 1970, 1; 1,1]', ...
                'DEM', 'Victor_alt_site.mat');
            
DEM = area_config.DEM.DATA;
            
RSSI_model = RSSI_Model(RSSI_sensor_config);
RSSI_generator = RSSI_Generator(sim_sensor_config, sim_target_radio_config, DEM);

uav_xy = [1, 1];
uav0 = [uav_xy, DEM(uav_xy(1), uav_xy(2))+50, pi/4]';
target_height = 0.2;
resolution = 10;

% calculate measurement generator and model
DEM_size = 10*floor(size(DEM)/10);
meas_z = zeros(round(DEM_size(2)/resolution), round(DEM_size(1)/resolution));
model_z = zeros(round(DEM_size(2)/resolution), round(DEM_size(1)/resolution));


target = [(10:resolution:DEM_size(1));...
        ones(1, round(DEM_size(1)/resolution));...
        zeros(1, round(DEM_size(1)/resolution))];
  
for y = 1*resolution:resolution:DEM_size(2)
    % update target location matrix
    target(2,:) = y*ones(1, round(DEM_size(1)/resolution));
    target(3,:) = DEM(target(1,:), y) + target_height;

    % calculate measurement
    model_z(y/resolution, :) = RSSI_model.meas_model(target, uav0, 1);
    for i = 1:size(target,2)
        meas_z(y/resolution, i) = RSSI_generator.get_RSSI_private(target(:,i), uav0);
    end
end   



% plot result
figure(); surf(meas_z); shading interp;
title('Real measurement');
xlabel('x/10 (m)');
ylabel('y/10 (m)');
zlabel('z (m)');
colorbar;

figure(); surf(model_z); shading interp;
title('Model');
xlabel('x/10 (m)');
ylabel('y/10 (m)');
zlabel('z (m)');
colorbar;

model_diff = model_z - meas_z;
figure(); surf(model_z - meas_z); shading interp;
title('Model');
xlabel('x/10 (m)');
ylabel('y/10 (m)');
zlabel('z (m)');
colorbar;

% downsample DEM
DEM_d = downsample(DEM, resolution);
DEM_d = downsample(DEM_d', resolution);
figure(); surf(DEM_d); shading interp;
title('Model Difference');
xlabel('x/10 (m)');
ylabel('y/10 (m)');
zlabel('z (m)');
colorbar;


% print max min mismatch
fprintf('Mismatch range: [%f, %f] \n', min(model_diff, [], 'all'), max(model_diff, [], 'all'));
figure();
histogram(model_diff);
    