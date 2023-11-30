%% [System module]
%  configurate the bearing correlator module

function bearing_config = init_bearing_config(varargin)
    p = inputParser;
    addParameter(p, 'Antenna', Antenna_Type.H);        % Antenna type
    addParameter(p, 'Interpolate_factor', 10);      
    addParameter(p, 'Samples', 10);                 % number of bearing samples
    addParameter(p, 'Move_time', 10);               % UAV move time in AoA action
    
    parse(p, varargin{:});
    
    bearing_config.Antenna = p.Results.Antenna;
    bearing_config.Interpolate_factor = p.Results.Interpolate_factor;
    bearing_config.Samples = p.Results.Samples;
    bearing_config.Move_time = p.Results.Move_time;
    
    bearing_config.Type = 'System';
    
    if bearing_config.Antenna == Antenna_Type.H
       bearing_config.Antenna_model = load('H_antenna3.mat');   
    elseif bearing_config.Antenna == Antenna_Type.NoBackH
        bearing_config.Antenna_model = load('H_antenna_no_back.mat');
    end
    
end