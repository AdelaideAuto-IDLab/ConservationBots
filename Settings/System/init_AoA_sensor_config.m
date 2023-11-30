%% [System module]
%  configurate the real sensor module

function sensor_config = init_AoA_sensor_config(varargin)
    p = inputParser;
    addParameter(p, 'Antenna', Antenna_Type.H);        % Antenna type
    addParameter(p, 'Sigma_aoa', 0.03);     
    
    parse(p, varargin{:});
    
    sensor_config.Antenna = p.Results.Antenna; 
    sensor_config.Sigma_aoa = p.Results.Sigma_aoa;
    
    sensor_config.Type = 'System';
    
    if sensor_config.Antenna == Antenna_Type.H
       sensor_config.Antenna_model = load('H_antenna3.mat');   
    end
    
end