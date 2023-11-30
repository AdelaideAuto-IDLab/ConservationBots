function i = Multinomial_resampling(weight, N)
    %% [Usage]:
    % i = Multinomial_resampling(weight, N)
    %
    % [Description]:
    % multinomial resampling methods
    %
    % [Input]
    % weight:       array of weight
    % N:            targeted resampled number
    %
    % [Output]
    % i:            resampled index
    %
    
    i = randsample(length(weight), N, true, weight);

end