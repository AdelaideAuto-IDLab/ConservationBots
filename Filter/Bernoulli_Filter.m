classdef Bernoulli_Filter < Filter
   properties
      ID = 'BF';        % filter type
      config;           % filter configuration
      Meas_Model;       % measurement model
      Sys_Model;        % system model
   end
    
   
   methods
       function obj = Bernoulli_Filter(config, meas_model, sys_model, varargin)
           if nargin > 0
               obj.config = config;
               obj.Meas_Model = meas_model;
               obj.Sys_Model = sys_model;
           end            
       end
       
       % bernoulli prediction
       function [bf] = predict(obj, bf)
            % parameter short cut to improve readibility
            Pb = obj.config.Pb;
            Ps = obj.config.Ps;
           
            r_previous = bf.r;  % previouse existence probability
            % predict existence probability
            bf.r = limit_range(Pb*(1 - bf.r) + Ps*bf.r);
            % predict survive particles
            predicted_particles = obj.Sys_Model.sys(bf.particles, true);

            particles = predicted_particles;
            % predict survive particle weights
            weight = bf.weight * Ps * r_previous/bf.r;

            birth_weight = Pb*(1-r_previous)/bf.r * 1/length(bf.birth_particles)*ones(length(bf.birth_particles),1);

            % add new birth particles to the set
            particles = [particles, bf.birth_particles];
            weight = [weight; birth_weight];
           
            % resampling
            sampled_index = obj.resample(weight, bf.N);
            particles = particles(:, sampled_index);
            
            bf.particles = particles;
            bf.weight = 1/bf.N * ones(bf.N, 1);
       end
       
       % bernoulli update(predict + update)
       % note: z format: [id, measurement]
       function [bf] = update(obj, bf, z, uav)
            if isempty(z)
                bf = predict(bf);
                return
            end
           
            if z(1, 1) ~= bf.ID
                error('Error: measurement ID does not match density ID.\n');
            end
           
            % variable to improve readibility
            Pf = obj.config.Pf;
            cz = obj.config.cz;

            weight = bf.weight;
            particles = bf.particles;
            
            if obj.Meas_Model.Type == Measurement_Type.RSSI
                Pd = obj.Meas_Model.is_detected(particles, uav(:, 1), z(1, 1));
            else
                Pd = obj.config.Pd;
            end

            % =============== Bernoulli filter update =====================
            if sum(~isnan(z(:, 2))) == size(z, 1)   % check if measurement contain NaN
                I2 = zeros(size(z, 1), 1);
                w = cell(size(z, 1), 1);
                for i = 1:size(z, 1)
                    w{i} = obj.Meas_Model.likelihood(z(i, 2), particles, uav(:, i), z(i, 1));
                    I2(i) = sum(w{i}.*weight.*Pd);
                end
                I1 = sum(weight .* Pd);
                delta_k = I1 - sum(I2)/(Pf*cz);
                % update existence probability
                bf.r = limit_range(bf.r*(1 - delta_k)/(1 - delta_k * bf.r));
                % update weight
                weight = weight .*((1 - Pd) + Pd .* (w{1})/(Pf*cz));
                % normalized weight
                weight = weight / sum(weight);         
            end
            % =============== End of filter update ========================
            
            
            % =============== Resampling ==================================
            if obj.config.Regularized
                % calculate covariance
                Sk = weightedcov(particles', weight);
            end

            % resampling
            sampled_index = obj.resample(weight, bf.N);
            particles = particles(:, sampled_index);

            if obj.config.Regularized
                particles = obj.regularization(particles, bf.nx, bf.N, Sk, uav, z);
            end

            % store new weight and particles
            bf.particles = particles;
            bf.weight = 1/bf.N * ones(bf.N, 1);
       end  
       
       
       
       % bernoulli filtering (predict + update)
       % note: z format: [id, measurement]
       function [bf] = filtering(obj, bf, z, uav)
            if isempty(z)
                return
            end
           
           
            if z(1) ~= bf.ID
                error('Error: measurement ID does not match density ID.\n');
            end
           
            % parameter short cut to improve readibility
            Pb = obj.config.Pb;
            Pf = obj.config.Pf;
            Ps = obj.config.Ps;
            cz = obj.config.cz;


            % ================== Bernoulli filter prediction ==============
            r_previous = bf.r;  % previouse existence probability
            % predict existence probability
            bf.r = limit_range(Pb*(1 - bf.r) + Ps*bf.r);
            % predict survive particles
            predicted_particles = obj.Sys_Model.sys(bf.particles, true);

            particles = predicted_particles;
            % predict survive particle weights
            weight = bf.weight * Ps * r_previous/bf.r;

            birth_weight = Pb*(1-r_previous)/bf.r * 1/length(bf.birth_particles)*ones(length(bf.birth_particles),1);

            % add new birth particles to the set
            particles = [particles, bf.birth_particles];
            weight = [weight; birth_weight];
            weight = weight/sum(weight);
            % ================ End of filter prediction ===================

            if obj.Meas_Model.Type == Measurement_Type.RSSI
                Pd = obj.Meas_Model.is_detected(particles, uav(:, 1), z(1, 1));
            else
                Pd = obj.config.Pd;
            end

            % =============== Bernoulli filter update =====================
            if sum(~isnan(z(:, 2)))
                I2 = zeros(size(z, 1), 1);
                w = cell(size(z, 1), 1);
                for i = 1:size(z, 1)
                    w{i} = obj.Meas_Model.likelihood(z(i, 2), particles, uav(:, i), z(i, 1));
                    I2(i) = sum(w{i}.*weight.*Pd);
                end
                I1 = sum(weight.* Pd);
                delta_k = I1 - sum(I2)/(Pf*cz);
                % update existence probability
                bf.r = limit_range(bf.r*(1 - delta_k)/(1 - delta_k * bf.r));
                % update weight
                weight = weight .*((1 - Pd) + Pd .* (w{1})/(Pf*cz));
                % normalized weight
                weight = weight / sum(weight);         
            end
            % =============== End of filter update ========================
            
            
            % =============== Resampling ==================================
            if obj.config.Regularized
                % calculate covariance
                Sk = weightedcov(particles', weight);
            end

            % resampling
            sampled_index = obj.resample(weight, bf.N);
            particles = particles(:, sampled_index);

            if obj.config.Regularized
                particles = obj.regularization(particles, bf.nx, bf.N, Sk, uav(:, 1), z);
            end

            % store new weight and particles
            bf.particles = particles;
            bf.weight = 1/bf.N * ones(bf.N, 1);
       end 
       
   end 
   
   
   methods (Access = private)
       function [sampled_index] = resample(obj, weight, N)
            switch obj.config.Resampling
                case Resampling_Type.Multinomial
                    sampled_index = Multinomial_resampling(weight, N);
                case Resampling_Type.Systematic
                    sampled_index = Systematic_resampling(weight, N);
                otherwise
                    error('Error: selected esampling method not implemented.\n');
            end
       end
       
       function particles = regularization(obj, particles, nx, N, Sk, uav, z)
                %% improve sample diversity
                h_bw = (4/(nx+2))^(1/(nx+4))*N^(-1/(nx+4));    % optimal bandwidth for gaussian kernel
                % compute Dk
                Dk = cholcov(Sk);
                % draw samples from gaussian kernel
                epsilon_s = mvnrnd(zeros(1, nx), h_bw.*ones(1, nx), N)';
                % regularized move
                particles_star = particles + h_bw*Dk*epsilon_s;

                % calculate acceptance probability
                w1 = obj.Meas_Model.likelihood(z(1,2), particles, uav, z(1, 1));
                w2 = obj.Meas_Model.likelihood(z(1,2), particles_star, uav, z(1, 1));

                acceptance_th = min(1, w2./w1);
                acceptance_p = rand(length(w1), 1);
                accept_i = acceptance_p < acceptance_th;
                particles(accept_i) = particles_star(accept_i);
       end
   end
   
end
