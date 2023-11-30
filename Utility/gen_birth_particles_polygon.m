function birth = gen_birth_particles_polygon(DEM, N, area, elevation_range)
    % [Usage]
    % birth = gen_birth_particles_polygon
    %
    % [Description]
    % Generate uniform birth particles inside a polygon
    %
    % [Input]
    % DEM           DEM data
    % N             Number of particles
    % [Output]
    % birth         birth particles
    %
    
    if nargin == 3
        elevation_range = [min(DEM, [], 'all') - 1; max(DEM, [], 'all') + 1];
    end
    
    % get bounding box
    [bound_x_lim, bound_y_lim] = boundingbox(polyshape(area'));
    
    
    remaining_particles = N;
    birth = nan(3,N);
    while remaining_particles ~= 0
        b = [diff(bound_x_lim)*rand(1, remaining_particles) + bound_x_lim(1); ...
             diff(bound_y_lim) * rand(1, remaining_particles) + bound_y_lim(1); ...
             diff(elevation_range)*rand(1,remaining_particles) + elevation_range(1) ...
            ];
            
        % reject particles outside the boundaryre
        I = inpolygon(b(1,:), b(2,:), area(1,:), area(2,:));
        valid_N = sum(I);
        last_valid_index = N-remaining_particles+1;
        birth(:, last_valid_index:last_valid_index+valid_N-1) = b(:, I);
        remaining_particles = remaining_particles - valid_N; 
    end
    

end