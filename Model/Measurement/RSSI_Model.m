classdef RSSI_Model < Measurement_Model
    
   properties 
      Type = Measurement_Type.RSSI;
      config;
   end
   
   methods
       % constructor
       function obj = RSSI_Model(config)
          if nargin > 0
             obj.config = config; 
          end
       end
       
       % RSSI measurement model (noiseless)
       function z = meas_model(obj, x, uav, target_id)  
          [gain, r] = obj.get_gain_dist(x, uav);
          
          idx = find(obj.config.target_id_list ==  target_id);
          z = obj.config.P0(idx) -10*obj.config.Path_Loss(idx).*log10(r) + gain; 
    
       end
       
       function d = is_detected(obj, x, uav, target_id)
          [gain, r] = obj.get_gain_dist(x, uav);
          
          idx = find(obj.config.target_id_list ==  target_id);
          z = obj.config.P0(idx) -10*obj.config.Path_Loss(idx).*log10(r) + gain + ...
              max(obj.config.Imprecision_Range(:, idx)); 
          d = (z > obj.config.Sensitivity)';
       end
              
       
       % RSSI measurement likelihood
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
           l = normpdf(z_sample, z, obj.config.Sigma_RSSI); 
       end
       
       % imprecise likelihood function
       function l = likelihood_imprecise(obj, z, x, uav, target_id)
           idx = find(obj.config.target_id_list ==  target_id);
           z_sample = obj.meas_model(x, uav, target_id)';
           max_z = z_sample + obj.config.Imprecision_Range(2, idx);
           min_z = z_sample + obj.config.Imprecision_Range(1, idx);
           l = (normcdf(z, min_z, obj.config.Sigma_RSSI) - normcdf(z, max_z, obj.config.Sigma_RSSI));
       end
       
       
       % get antenna gain and LOS distance
       function [gain, r] = get_gain_dist(obj, x, uav)
          switch obj.config.Antenna
              case Antenna_Type.H
                [gain, ~, r] = Get_Antenna_Gain(x, uav, obj.config.Antenna_model);
              case Antenna_Type.Array
                [gain, ~, r] = Get_Antenna_Gain_phase_array(x, uav, obj.config.Antenna_model);
              case Antenna_Type.Isotropic
                  gain = 0; 
                  r = distance_to_uav(x, uav);
          end
       end
       
   end
end