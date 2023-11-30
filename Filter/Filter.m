classdef (Abstract) Filter < handle
   properties (Abstract)
      ID;               % filter type
      config;           % filter configuration
      Meas_Model;       % measurement model
      Sys_Model;        % system model
   end
    
   
   methods (Abstract)
       predict(obj);     % perform filter prediction
       filtering(obj);   % perform filter action
   end
    
end