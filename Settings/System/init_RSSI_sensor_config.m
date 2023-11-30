%% [System module]
%  configurate the real sensor module

function sensor_config = init_RSSI_sensor_config(varargin)
    p = inputParser;
    addParameter(p, 'Antenna', Antenna_Type.H);        % Antenna type
    addParameter(p, 'Sensitivity', -135);   % Sensitivity of sensor 
    addParameter(p, 'Sigma_RSSI', 6);       % standard deviation of RSSI measurement
    addParameter(p, 'Path_Loss', 4);        % path loss for each target
    addParameter(p, 'P0', 40);              % reference power
    addParameter(p, 'target_id_list', []);  % target id list mapping
    
    addParameter(p, 'Likelihood_Type', Likelihood_Type.precise);    % precise/imprecise likelihood
    addParameter(p, 'Imprecision_Range', []);       % [min1, min2, ... minN;
                                                    % [max1, max2, ... maxN];
    
    parse(p, varargin{:});
    
    sensor_config.Antenna = p.Results.Antenna;
    sensor_config.Sensitivity = p.Results.Sensitivity;  
    sensor_config.Sigma_RSSI = p.Results.Sigma_RSSI;
    sensor_config.Path_Loss = p.Results.Path_Loss;
    sensor_config.P0 = p.Results.P0;
    sensor_config.Likelihood_Type = p.Results.Likelihood_Type;
    sensor_config.Imprecision_Range = p.Results.Imprecision_Range;
    sensor_config.target_id_list = p.Results.target_id_list;
    
    sensor_config.Type = 'System';
    
    if sensor_config.Antenna == Antenna_Type.H
       sensor_config.Antenna_model = load('H_antenna3.mat');   
    end
    
end