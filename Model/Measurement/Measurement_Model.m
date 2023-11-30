classdef (Abstract) Measurement_Model < handle
    properties (Abstract)
        Type;           % Measurement type
        config;         % sensor config
    end
    
    methods (Abstract)
       meas_model(obj); % measurement model
       likelihood(obj); % likelihood function
    end
    
end