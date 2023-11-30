function [variance, error_rate, ang_diff] = bearing_vs_sample(bearing_sample, repeat_time, noise, distance)

	% result init
	variance = nan(1, length(bearing_sample));
	error_rate = nan(1, length(bearing_sample));

	dist_range = [distance, distance];

	AoA_config = init_bearing_config('Antenna', Antenna_Type.H);
	AoA_Generator = Rotation_Bearing(AoA_config);

	sim_target_radio_config = init_sim_target_radio_config( ...
		'NTarget'    ,   1, ...
		'P0'         ,   30,...
		'Frequency'  ,   150e6,...
		'Path_Loss'  ,   3,...
		'Tree_Height',   0 ...
	 );

	sim_sensor_config = init_sim_sensor_config(...
		'Antenna'    ,      Antenna_Type.H,...
		'Pd'         ,      1,...
		'Sensitivity',      -70,...
		'Sigma'      ,      noise ...
	 );

	RSSI_generator = RSSI_Generator(sim_sensor_config, sim_target_radio_config, ones(5000, 5000));

	area_config = init_area_config('Vertex', [1,1; 1, 5000; 5000, 5000; 5000, 1; 1,1]');


	for j = 1:length(bearing_sample)
        uav_config = init_uav_config( ...  
            'traject_time', 200, ...
            'rotation_time', bearing_sample(j) ...
        );  

        UAV_Action = UAV_Actions(uav_config, area_config);

		% generate UAV action
		uav0 = [2500,2500,30,pi/4]';
		uav = UAV_Action.get_action_rotation(uav0, 2*pi);
		x0 = [2500, 2500, 0.2]';
		truth = nan(repeat_time, 1);

		bearing = nan(repeat_time,1);
		for n = 1:repeat_time
			% randomize target location
			x = x0;
			rand_ang = 2*pi*rand();
			dist = dist_range(1) + diff(dist_range)*rand();
			x(1:2) = x(1:2) + dist*[sin(rand_ang); cos(rand_ang)];
			truth(n) = atan2(x(1) - uav0(1), x(2) - uav0(2));
			
			AoA_Generator.start_time = 1;
			AoA_Generator.end_time = bearing_sample(j);
			for i = 1:bearing_sample(j)
			   z = RSSI_generator.get_RSSI_debug(x, uav(:, i));  
               if z.RSSI < sim_sensor_config.Sensitivity
                   z.RSSI = NaN;
               end
			   AoA_Generator.record_data(z, i);
			end

			b = AoA_Generator.get_bearing();
			bearing(n) = b.AoA(1);
		end


		ang_diff = angdiff(bearing, truth);
		valid_i = (abs(ang_diff) < deg2rad(50));
		
		variance(j) = var(ang_diff(valid_i));
		error_rate(j) = 1-mean(valid_i);
	end
end