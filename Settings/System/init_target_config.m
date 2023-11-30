%% [System Module]
% use in filtering and planning
function target_config = init_target_config(varargin)

    p = inputParser;
    addParameter(p, 'Ntarget', 1);      % maximum number of targets
    addParameter(p, 'Mode', 'WD');      % Target Motion model
                                        % WD - wondering mode
                                        % CV - constant velocity
                                           
    addParameter(p, 'nx', 3);           % number of target states
    addParameter(p, 'dt', 1);           % time step
                                        
    addParameter(p, 'Noise_Cov', diag([0.25, 0.25, 0.025]));    % process noise
                                                                % covariance
                                                                % matrix
                                                                
    addParameter(p, 'Transition_Model', eye(3));      % target transition matrix     
    

    
    parse(p, varargin{:});
    
    target_config.Ntarget = p.Results.Ntarget;
    target_config.Mode = p.Results.Mode;
    target_config.nx = p.Results.nx;
    target_config.dt = p.Results.dt;
    target_config.Noise_Cov = p.Results.Noise_Cov;
    target_config.Transition_Model = p.Results.Transition_Model;
    
    % sanity check
    if target_config.nx ~= length(target_config.Noise_Cov) || ...
       target_config.nx ~= length(target_config.Transition_Model)
       error('Target Config Error: number of target states does not match other model parameter.\n');
    end
         

    %% conditional parameter:
    if strcmp(p.Results.Mode, 'WD')         % wondering mode
        target_config.nx = 3;   
    elseif strcmp(p.Results.Mode, 'CV')     % constant velocity
        error('Constant Velocity Mode NOT IMPLEMENTED');
    end 

end