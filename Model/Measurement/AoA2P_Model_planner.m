classdef AoA2P_Model_planner < Measurement_Model
    %% temp aoa 2p model used in planner
   properties
      Type = Measurement_Type.AoA_2P;
      config;
   end
   
   
   methods
       % constructor
       function obj = AoA2P_Model_planner(config)
          if nargin > 0
              obj.config = config;
          end  
       end
       
       % bearing (angle of arrival) measurement model (noiseless)
       function z = meas_model(obj, x, uav, target_id)
            if size(x, 2) > size(uav, 2)
                uav = repmat(uav, 1, size(x, 2));
            else
                x = repmat(x, 1, size(uav, 2));
            end

            z = atan2(x(1, :) - uav(1,:), x(2,:) - uav(2,:));
       end
       
       
       % bearing (aoa) measurement likelihood
       function l = likelihood(obj, z, x, uav, target_id)
            if (length(z) == 1) && (length(z) < size(x, 2))
                z = repmat(z,1,size(x,2));
            end
     
           z_sample = obj.meas_model(x, uav, target_id);
           l = normpdf(angdiff(z_sample, z), zeros(1, length(z_sample)), obj.config.Sigma_aoa)';   
       end
   
   end
    
end