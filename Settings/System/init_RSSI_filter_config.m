%% [System module]

function filter_config = init_RSSI_filter_config(varargin)

    %% --- Instantiate inputParser
    p = inputParser;
    addParameter(p, 'cz', 10);     % range false alarm pdf
    addParameter(p, 'nx', 3);      % number of state variable
    addParameter(p, 'Pb', 1e-3);   % birth probability
    addParameter(p, 'Pd', 0.99);   % range detection probability
    addParameter(p, 'Pf', 1e-2);   % range false alarm rate
    addParameter(p, 'Ps', 0.99);   % survive probability
    addParameter(p, 'lambda', 1);  % mean clutter
    
    addParameter(p, 'Regularized', false, @islogical);   % use regularization
    addParameter(p, 'Resampling', Resampling_Type.Multinomial);         % resampling method
  
    parse(p, varargin{:}); 
    
    filter_config.Pb = p.Results.Pb;
    filter_config.Ps = p.Results.Ps;
    filter_config.Pd = p.Results.Pd;
    filter_config.Pf = p.Results.Pf;
    filter_config.cz = p.Results.cz;
    filter_config.nx = p.Results.nx;
    filter_config.lambda = p.Results.lambda;
    
    filter_config.Regularized = p.Results.Regularized;
    filter_config.Resampling = p.Results.Resampling;
end