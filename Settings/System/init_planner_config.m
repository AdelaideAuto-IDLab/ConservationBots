%% [System Module]

function planner_config = init_planner_config(varargin)
    %% --- Instantiate inputParser
    p = inputParser;
    addParameter(p, 'alpha', 0.1);       % optional, used for Renyi divergence
    addParameter(p, 'Discount', 1);      % discount factor
    addParameter(p, 'Heading_number', 8);  % number of headings to choose
    addParameter(p, 'T', 10);            % planning interval
    addParameter(p, 'Variable_lookahead', true, @islogical);
    addParameter(p, 'Lookahead', 3);     % lookahead steps
    addParameter(p, 'Use_Void', false);  % use void
    addParameter(p, 'Void_r', 50);       % Void radius
    addParameter(p, 'Void_th', 0.9);     % Void threshold
    addParameter(p, 'Planning_sample', 1000);   % number of particles used in planning
    addParameter(p, 'Rotation_time', 10);   % bearing rotation time

 
    addParameter(p, 'MC_sample', 10);          % MC samples
    
    addParameter(p, 'Mode', Planning_Meas_Mode.PIMS);     % method for generating future measurement
                                         % PIMS: predicted ideal measurement set
                                         % MC:  Monte-Carlo sampling
                                         
    addParameter(p, 'Reward', Reward_Type.Renyi);  % reward function
                                         % Closest: higher reward for closer distance
                                         % Renyi: Renyi divergence
                                         % Shannon: Shannon entropy
                                         % Cauchy: Cauchy-Schwarz divergence
                                         % Guass-Renti: Gaussian approx renyi divergence
    
                                         
    addParameter(p, 'Plan_target', 'Closest');  % planner type
                                                % Closest: plan for closest target
                                                % All: plan for all (unlocalized) target
                                                
    addParameter(p, 'Density_type', 'Bernoulli');   % density type
                                                    % Bernoulli: used in bernoulli filter
                                                    % Particle: used in particle filter
                                                    
    addParameter(p, 'Circle_radius', 20);           % circle radius for circle reward(task based)
    
    
    
    parse(p, varargin{:});

    planner_config.alpha = p.Results.alpha; 
    planner_config.Discount = p.Results.Discount;
    planner_config.Heading_number = p.Results.Heading_number;
    planner_config.Variable_lookahead = p.Results.Variable_lookahead;
    planner_config.Lookahead = p.Results.Lookahead;
    planner_config.MC_sample = p.Results.MC_sample;
    planner_config.Mode = p.Results.Mode;
    planner_config.Reward = p.Results.Reward;
    planner_config.T = p.Results.T;
    planner_config.Use_Void = p.Results.Use_Void;
    planner_config.Void_r = p.Results.Void_r;
    planner_config.Void_th = p.Results.Void_th;
    planner_config.Plan_target = p.Results.Plan_target;
    planner_config.Planning_sample = p.Results.Planning_sample;
    planner_config.Density_type = p.Results.Density_type;
    planner_config.Circle_radius = p.Results.Circle_radius;
    planner_config.Rotation_time = p.Results.Rotation_time;

end