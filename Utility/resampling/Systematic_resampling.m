function i = Systematic_resampling(weight, N)
    %% [Usage]:
    % i = Systematic_resampling(weight, N)
    %
    % [Description]:
    % systemic resampling methods
    %
    % [Input]
    % weight:       array of weight
    % N:            targeted resampled number
    %
    % [Output]
    % i:            resampled index
    %
    
    edges = min([0, cumsum(weight)'], 1);
    edges(end) = 1;
    u1 = rand/N;
    [~, i] = histc(u1:1/N:1, edges);

end