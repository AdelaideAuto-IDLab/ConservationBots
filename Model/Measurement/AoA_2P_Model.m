classdef AoA_2P_Model < Measurement_Model
    
   properties 
      Type = Measurement_Type.AoA_2P;
      config;
   end
   
   methods
       % constructor
       function obj = AoA_2P_Model(config)
          if nargin > 0
             obj.config = config; 
          end
       end
       
       % R2 point bearing measurement model (noiseless)
       function z = meas_model(obj, x, uav, target_id)  
          [gain1, r1] = obj.get_gain_dist(x{1}, uav(:,1));
          z1 = obj.config.P0(target_id) - 10*obj.config.Path_Loss(target_id).*log10(r1) + gain1; 

          [gain2, r2] = obj.get_gain_dist(x{2}, uav(:,2));
          z2 = obj.config.P0(target_id) - 10*obj.config.Path_Loss(target_id).*log10(r2) + gain2; 
          
          z = z1-z2;
       end
       
       function z = meas_model_l(obj, x, uav, target_id, n)
          [gain1, r1] = obj.get_gain_dist(x{1}, uav(:,1));
          z1 = obj.config.P0(target_id) - 10*n.*log10(r1) + gain1; 

          [gain2, r2] = obj.get_gain_dist(x{2}, uav(:,2));
          z2 = obj.config.P0(target_id) - 10*n.*log10(r2) + gain2; 
          
          z = z1-z2;
       end
       
       function d = is_detected(obj, x, uav)
          [gain1, r1] = obj.get_gain_dist(x{1}, uav(:,1));
          z1 = obj.config.P0(1) - 10*3.*log10(r1) + gain1 + obj.config.Sigma_RSSI*randn(size(x{1}(1,:))); 

          [gain2, r2] = obj.get_gain_dist(x{2}, uav(:,2));
          z2 = obj.config.P0(1) - 10*3.*log10(r2) + gain2 + obj.config.Sigma_RSSI*randn(size(x{2}(1,:))); 
          
          d = ((z1 >= obj.config.Sensitivity) & (z2 >= obj.config.Sensitivity))';
       end
       
       % 2 point bearing measurement likelihood
       % input: x: cell array
       % z: 2 measurement
       % uav: 2 uav location
       function l = likelihood(obj, z, x, uav, target_id)
           switch obj.config.Likelihood_Type
               case Likelihood_Type.precise
                   l = obj.likelihood_precise(z, x, uav, target_id);
               case Likelihood_Type.imprecise
                   l = obj.likelihood_imprecise(z, x, uav, target_id);
               otherwise
                   error('Error: selected likelihood function not exist.');
           end     
       end
       
       
       % precise likelihood function
       function l = likelihood_precise(obj, z, x, uav, target_id)
           z_sample = obj.meas_model(x, uav, target_id)';
           if length(z) == 2
               l = normpdf((z_sample), (z(1) - z(2)), sqrt(2)*obj.config.Sigma_RSSI); 
               
%                if (z(1) - z(2) > 0)
%                    l = 1 - normcdf(0, z_sample, sqrt(2)*obj.config.Sigma_RSSI);
%                else
%                    l = normcdf(0, z_sample, sqrt(2)*obj.config.Sigma_RSSI);
%                end
               
           else
               l = normpdf(z_sample, z(1), sqrt(2)*obj.config.Sigma_RSSI); 
           end
       end
       
       % imprecise likelihood function
       function l = likelihood_imprecise(obj, z, x, uav, target_id)
           z_1 = obj.meas_model_l(x, uav, target_id, 4)';
           z_2 = obj.meas_model_l(x, uav, target_id, 2)';
           if length(z) == 2
               Z = z(1) - z(2);
           else
               Z = z;
           end
           l = (normcdf(Z, min(z_1, z_2), sqrt(2)*obj.config.Sigma_RSSI) - normcdf(Z, max(z_1, z_2), sqrt(2)*obj.config.Sigma_RSSI));
       end
       
       % get antenna gain and LOS distance
       function [gain, r] = get_gain_dist(obj, x, uav)
          switch obj.config.Antenna
              case Antenna_Type.H
                [gain, ~, r] = Get_Antenna_Gain(x, uav, obj.config.Antenna_model);
              case Antenna_Type.A
                [gain, ~, r] = Get_Antenna_Gain(x, uav, obj.config.Antenna_model);
              case Antenna_Type.Array
                [gain, ~, r] = Get_Antenna_Gain_phase_array(x, uav);
              case Antenna_Type.Isotropic
                  gain = 0; 
                  r = distance_to_uav(x, uav);
          end
       end
       
   end
end