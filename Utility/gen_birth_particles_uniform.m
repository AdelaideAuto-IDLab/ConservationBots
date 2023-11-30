function birth = gen_birth_particles_uniform(DEM, N)
    % [Usage]
    % birth = gen_birth_particles_uniform
    %
    % [Description]
    % Generate uniform birth particles 
    %
    % [Input]
    % DEM           DEM data
    % N             Number of particles
    % [Output]
    % birth         birth particles
    %
    
    elevation_range = [min(DEM, [], 'all') - 1; max(DEM, [], 'all') + 1];
    
    DEM_x = size(DEM, 2);
    DEM_y = size(DEM, 1);
    
    birth = [DEM_x*rand(1, N); ...
             DEM_y * rand(1, N); ...
             diff(elevation_range)*rand(1,N) + elevation_range(1) ...
            ];

end