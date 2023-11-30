classdef Target_Model < handle
    %% NOTE: target model does not if target move outside search area
    
   properties
      config; 
   end
   
   
   methods
       function obj = Target_Model(config)
           if nargin > 0
               obj.config = config;
           end
       end
           
       function state = sys(obj, x, add_noise)
           state = obj.config.Transition_Model*x;

           if add_noise
               state = state + mvnrnd(zeros(1, obj.config.nx), obj.config.Noise_Cov, size(x, 2))';   
           end
       end    
   end
   
  
end