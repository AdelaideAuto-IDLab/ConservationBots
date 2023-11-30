
function pf = Particle(birth, ID)
    % [Usage]
    % bf = Particle(birth)
    % 
    % [Description]
    % Create bernoulli term
    %
    % [Input]
    % birth         birth particles
    % ID            target ID
    %
    % [Output]
    % bf            Bernoulli struc
    %
    
    pf.particles = birth;
    pf.N = length(birth);
    pf.weight = 1/pf.N * ones(pf.N, 1);
    pf.birth_particles = birth;
    pf.nx = size(birth, 1);
    pf.localized = false;
    
    pf.ID = ID;
    
end