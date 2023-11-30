%% [Simulation module]
%  configurate the simulation sensor module

function sim_sensor_config = init_sim_sensor_config(varargin)
    p = inputParser;
    addParameter(p, 'Antenna', Antenna_Type.H);        % Antenna type
    addParameter(p, 'Pd', 0.9);             % Detection probability
    addParameter(p, 'Sensitivity', -135);   % Sensitivity of sensor     
    addParameter(p, 'Sigma', 6);            % Guassian noise std

    
    parse(p, varargin{:});
    
    sim_sensor_config.Antenna = p.Results.Antenna;
    sim_sensor_config.Pd = p.Results.Pd;
    sim_sensor_config.Sensitivity = p.Results.Sensitivity;
    sim_sensor_config.Sigma = p.Results.Sigma;
    
    sim_sensor_config.Type = 'Simulation';  % setting identifier
    
    
    
    if sim_sensor_config.Antenna == Antenna_Type.H
       sim_sensor_config.Antenna_model = load('H_antenna3.mat');   
    end
    
end