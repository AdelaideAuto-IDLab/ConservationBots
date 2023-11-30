%% [System & simulaltiuon module]

function uav_config = init_uav_config(varargin)
    %% --- Instantiate inputParser
    p = inputParser;
    addParameter(p, 'accel', 4);     % acceleration
    addParameter(p, 'ChangeHeight', false, @islogical); 
    addParameter(p, 'decel', -4);    % deacceleration
    addParameter(p, 'delta_time', 0.01);    
    addParameter(p, 'turn_rate', pi/1);     % yaw speed
    addParameter(p, 'uav0', [1;1;80;pi/4]);     % initial uav state
    addParameter(p, 'vmax', 10);     %  maximum velocity
    
    addParameter(p, 'traject_time', 10);    % total time allow to exectue action
    addParameter(p, 'rotation_time', 10);   % time to take bearing measurement
    
    parse(p, varargin{:});
    
    % input sanity check
    if(p.Results.rotation_time > p.Results.traject_time)
        erorr('Error: rotation time longer than total trajectory time.');
    end
    
    uav_config.vmax = p.Results.vmax;
    uav_config.uav0 = p.Results.uav0;
    uav_config.accel = p.Results.accel;
    uav_config.decel = p.Results.decel;
    uav_config.delta_time = p.Results.delta_time;
    uav_config.turn_rate = p.Results.turn_rate;
    uav_config.ChangeHeight = p.Results.ChangeHeight;
    uav_config.traject_time = p.Results.traject_time;
    uav_config.rotation_time = p.Results.rotation_time;
    
    uav_config.ta_max = uav_config.vmax/uav_config.accel;   % maximum acceleration time
    
end