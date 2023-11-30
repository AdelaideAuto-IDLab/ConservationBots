classdef Particle_Filter < Filter
   properties
      ID = 'PF';        % filter type
      config;           % filter configuration
      Meas_Model;       % measurement model
      Sys_Model;        % system model
   end
    
   
   methods
       function obj = Particle_Filter(config, meas_model, sys_model, varargin)
           if nargin > 0
               obj.config = config;
               obj.Meas_Model = meas_model;
               obj.Sys_Model = sys_model;
           end
           
       end
       
       % PF prediction
       function [pf] = predict(obj, pf)
            % predict survive particles
            pf.particles = obj.Sys_Model.sys(pf.particles, true);
       end
       
       % PF update
       function [pf] = update(obj, pf, z, uav)
            % =============== Update ============================
            weight = pf.weight;
            if ~isnan(z(2))
                weight = weight.*obj.Meas_Model.likelihood(z(2), pf.particles, uav, z(1));
                
                % normalized weight
                weight = weight/sum(weight);
            end
            % =============== End of filter update ================================

            if obj.config.Regularized
                % calculate covariance
                Sk = weightedcov(pf.particles', weight);
            end


            % resampling
            if strcmp(obj.config.Resampling, 'Multinomial')
                sampled_index = Multinomial_resampling(weight, pf.N);
            elseif strcmp(obj.config.Resampling, 'Systematic') 
                sampled_index = Systematic_resampling(weight, pf.N);
            else
                error('Error: selected esampling method not implemented.\n');
            end
            particles = pf.particles(:, sampled_index);

            if obj.config.Regularized
                %% improve sample diversity
                h_bw = (4/(pf.nx+2))^(1/(pf.nx+4))*pf.N^(-1/(pf.nx+4));    % optimal bandwidth for gaussian kernel
                % compute Dk
                Dk = cholcov(Sk);
                % draw samples from gaussian kernel
                epsilon_s = mvnrnd(zeros(1, pf.nx), h_bw.*ones(1, pf.nx), pf.N)';
                % regularized move
                particles_star = pf.particles + h_bw*Dk*epsilon_s;

                % calculate acceptance probability
                w1 = obj.Meas_Model.likelihood(z, particles, uav);
                w2 = obj.Meas_Model.likelihood(z, particles_star, uav);

                acceptance_th = min(1, w2./w1);
                acceptance_p = rand(length(w1), 1);
                accept_i = acceptance_p < acceptance_th;
                particles(accept_i) = particles_star(accept_i);
            end

            % store new weight and particles
            pf.particles = particles;
            pf.weight = 1/pf.N * ones(pf.N, 1);
       end
       
       % PF filtering (predict + update)
       function [pf] = filtering(obj, pf, z, uav)
            if z(1) ~= pf.ID
                error('Error: measurement ID does not match density ID.\n');
            end
           
            % =============== Prediction ========================
            pf = obj.predict(pf);
            
            % =============== Update =============================
            pf = obj.update(pf, z, uav);
       end 
       
   end 
   
end