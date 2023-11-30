
function bf = Bernoulli(birth, r0, ID)
    % [Usage]
    % bf = Bernoulli(birth, r0)
    % 
    % [Description]
    % Create bernoulli term
    %
    % [Input]
    % birth         birth particles
    % r0            initial existence probability
    % ID            target ID
    %
    % [Output]
    % bf            Bernoulli struc
    %
    
    bf.particles = birth;
    bf.N = length(birth);
    bf.weight = 1/bf.N * ones(bf.N, 1);
    bf.r = r0;
    bf.birth_particles = birth;
    bf.nx = size(birth, 1);
    bf.localized = false;
    
    bf.ID = ID;
end