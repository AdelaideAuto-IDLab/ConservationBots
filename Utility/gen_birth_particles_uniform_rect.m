function birth = gen_birth_particles_uniform_rect(DEM, area, N)
    % [Usage]
    % birth = gen_birth_particles_uniform
    %
    % [Description]
    % Generate uniform birth particles 
    %
    % [Input]
    % DEM           DEM data
	% area 			vertex of defined area
    % N             Number of particles
    % [Output]
    % birth         birth particles
    %
    
	s = rand(1, N);
	t = rand(1, N);
	birth = repmat(area(:, 1), 1, N) + (area(:, 2) - area(:, 1))*s + (area(:,4) - area(:, 1))*t;
    
    
    I = sub2ind(size(DEM), round(birth(1,:)), round(birth(2,:)));
    elevation_range = [min(DEM(I), [], 'all') - 1; max(DEM(I), [], 'all') + 1];
	birth = [birth; diff(elevation_range)*rand(1,N) + elevation_range(1)];

end

