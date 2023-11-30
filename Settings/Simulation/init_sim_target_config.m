%% [Simulation Module]
% use for generating target truth 

function sim_target_config = init_sim_target_config(varargin)

    p = inputParser;
    addParameter(p, 'Ntarget', 1);      % number of target
    addParameter(p, 'Mode', 'WD');      % Target Motion model
                                        % WD - wondering mode
                                        % CV - constant velocity
             
                                        
    addParameter(p, 'nx', 3);           % number of target states
    addParameter(p, 'dt', 1);           % time step
                                        
    addParameter(p, 'Noise_Cov', diag([0.25, 0.25, 0]));    % process noise
                                                            % covariance
                                                            % matrix
                                                        
   
    addParameter(p, 'Transition_Model', eye(3));      % target transition matrix     
    
    addParameter(p, 'Init_State', [100; 100; 0.2]);     % target initial state
    addParameter(p, 'Birth_Death', [1; 1000]);        % birth and death time of each target

    
    parse(p, varargin{:});
    
    
    sim_target_config.NTarget = p.Results.Ntarget;
    sim_target_config.Mode = p.Results.Mode;
    sim_target_config.nx = p.Results.nx;
    sim_target_config.dt = p.Results.dt;
    sim_target_config.Noise_Cov = p.Results.Noise_Cov;
    sim_target_config.Transition_Model = p.Results.Transition_Model;
    sim_target_config.Init_State = p.Results.Init_State;
    sim_target_config.Birth_Death = p.Results.Birth_Death;
    sim_target_config.Height = p.Results.Init_State(3,:);
    
    sim_target_config.Type = 'Simulation';
    
    % sanity check
    % target states should match associated matrix
    if sim_target_config.nx ~= length(sim_target_config.Noise_Cov) || ...
       sim_target_config.nx ~= length(sim_target_config.Transition_Model) ||...
       sim_target_config.nx ~= size(sim_target_config.Init_State, 1)
       error('Target Config Error: number of target states does not match other model parameter.\n');
    end
    
    % target number should match birth/death and height data
    if sim_target_config.NTarget ~= size(sim_target_config.Init_State, 2) ||...
       sim_target_config.NTarget ~= size(sim_target_config.Birth_Death, 2)
       error('Target Config Error: target number does not match [Init_State], [Birth_Death], or [Height].\n');
    end
    
   

    %% conditional parameter:
    if strcmp(p.Results.Mode, 'WD')         % wondering mode
        sim_target_config.nx = 3;   
    elseif strcmp(p.Results.Mode, 'CV')     % constant velocity
        error('Constant Velocity Mode NOT IMPLEMENTED');
    end 

end