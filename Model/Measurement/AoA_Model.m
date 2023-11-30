classdef AoA_Model < Measurement_Model
   properties
      Type = Measurement_Type.AoA;
      config;
   end
   
   
   methods
       % constructor
       function obj = AoA_Model(config)
          if nargin > 0
              obj.config = config;
          end  
       end
       
       % bearing (angle of arrival) measurement model (noiseless)
       function z = meas_model(obj, x, uav, ~)
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
            
            %% State INdependent model
            l = 1*normpdf(angdiff(z_sample, z), zeros(1, length(z_sample)), obj.config.Sigma_aoa)' + 0/(2*pi); 
           
%             %% State dependent model
%             d = sqrt(sum((x(1:2, :) - uav(1:2)).^2));
%             aoa_sigma = sqrt(0.01225*exp(0.0002761.*d));
%             l = normpdf(angdiff(z_sample, z), zeros(1, length(z_sample)), aoa_sigma)';
       end
   
   end
    
    
    
    
end