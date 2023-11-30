classdef (Abstract) Planner < handle
   properties (Abstract)
      ID;               % filter type
      config;           % filter configuration
      Meas_Model;       % measurement model
      Sys_Model;        % system model
      Action_Model;     % control action generator
   end
    
   
   methods (Abstract)
       plan(obj);       % planning
   end
    
end