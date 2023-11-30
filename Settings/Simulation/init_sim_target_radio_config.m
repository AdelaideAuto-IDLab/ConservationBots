%% [Simulation module]
%  configurate the simulation target radio setting

function sim_target_radio_config = init_sim_target_radio_config(varargin)
    p = inputParser;
    addParameter(p, 'Ntarget', 1);          % number of targets
    addParameter(p, 'P0', 40);              % Reference power
    addParameter(p, 'Frequency', 150e6);    % frequency for each target
    addParameter(p, 'Path_Loss', 4);        % path loss exponent
    addParameter(p, 'Tree_Height', 1);      % vegetation coverage

    parse(p, varargin{:});
    
    % sanity check
    if p.Results.Ntarget ~= length(p.Results.Frequency) || ...
       p.Results.Ntarget ~= length(p.Results.Path_Loss) || ...
       p.Results.Ntarget ~= length(p.Results.Tree_Height)
   
        error('Config error: target number mismatch.\n');
    end
    
    sim_target_radio_config.P0 = p.Results.P0;
    sim_target_radio_config.Frequency = p.Results.Frequency;
    sim_target_radio_config.Path_Loss = p.Results.Path_Loss;
    sim_target_radio_config.Tree_Height = p.Results.Tree_Height;
    
    sim_target_radio_config.Type = 'Simulation';  % setting identifier
    
    
end