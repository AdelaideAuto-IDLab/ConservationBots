classdef AoA2P_planning_rssi_Model < Measurement_Model
    
   properties 
      Type = Measurement_Type.AoA_2P;
      config;
   end
   
   methods
       % constructor
       function obj = AoA2P_planning_rssi_Model(config)
          if nargin > 0
             obj.config = config; 
          end
       end
       
       % RSSI measurement model (noiseless)
       function z = meas_model(obj, x, uav, target_id)  
          [gain, r] = obj.get_gain_dist(x, uav);
          
          if ~obj.config.bootstrap
              z = obj.config.P0(target_id) -10*obj.config.Path_Loss(target_id).*log10(r) + gain; 
          else
              z = x(4,:) - 10*x(5,:).*log10(r) + gain;
          end

          % capped the minimal receiver power
          z(isinf(z)) = -150;
       end
       
       function z = meas_model_bootstrap(obj, x, uav, target_id, p, n)  
          [gain, r] = obj.get_gain_dist(x, uav);
          
          z = p - 10*n*log10(r) + gain;

          % capped the minimal receiver power
          z(isinf(z)) = -150;
       end  
       
       % RSSI measurement likelihood
       function l = likelihood(obj, z, x, uav, target_id, p, n)
           if nargin == 5
               switch obj.config.Likelihood_Type
                   case Likelihood_Type.precise
                       l = obj.likelihood_precise(z, x, uav, target_id);
                   case Likelihood_Type.imprecise
                       l = obj.likelihood_imprecise(z, x, uav, target_id);
                   otherwise
                       error('Error: selected likelihood function not exist.');
               end     
           elseif nargin == 7
                z_sample = obj.meas_model_bootstrap(x, uav, target_id, p, n)';
                l = normpdf(z_sample, z, obj.config.Sigma_RSSI);
           end
       end
       
       
       % precise likelihood function
       function l = likelihood_precise(obj, z, x, uav, target_id)
           z_sample = obj.meas_model(x, uav, target_id)';
           l = normpdf(z_sample, z, obj.config.Sigma_RSSI); 
       end
       
       % imprecise likelihood function
       function l = likelihood_imprecise(obj, z, x, uav, target_id)
           z_sample = obj.meas_model(x, uav, target_id)';
           max_z = z_sample + obj.config.Imprecision_Range(2, target_id);
           min_z = z_sample + obj.config.Imprecision_Range(1, target_id);
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