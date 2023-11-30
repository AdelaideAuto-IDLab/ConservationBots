classdef Termination_Condition < handle
    
   properties
      config;
   end   
   
   methods
       % constructor
       % threshold
       function obj = Termination_Condition(config)
           obj.config = config;
       end
       
       % check whether terminaltion conditions are met
       function [density, all_found, localized_report, termination_val]= termination_check(obj, density, current_time)
           % report newly localized ID & time
           % Format: [ID; time]
           localized_report = nan(length(density), 2);
           termination_val = nan(length(density), 1);
           all_found = true;
           for n = 1:length(density)
               if ~density{n}.localized
                    [density{n}.localized, termination_val(n)] = obj.termination_method(density{n});
                    
                    if density{n}.localized
                        % update localized report 
                        localized_report(n,:) = [density{n}.ID, current_time];
                    end
                    
                    all_found = all_found && density{n}.localized;
               end
           end  
           % remove all row contains NaN in localized_report
           localized_report = localized_report(any(~isnan(localized_report), 2), :);  
       end
       
       % check based on selected termination method
       function [isfound, val] = termination_method(obj, density)
            if length(density) ~= 1
                error('Error: expected cell array of size 1.\n');
            end
           
            switch obj.config.mode 
                case Termination_Method.det     % determination method
                    val = det(cov(density.particles(1:2, :)'));
                case Termination_Method.std     % maximum standard deviation method
                    val = max(diag(cov(density.particles(1:2, :)')));
                case Termination_Method.eigen   % maximum semi-axis length of 90% error ellipse
                    [~,D] = eig(cov(density.particles(1:2,:)'));
                    val = max(sqrt(chi2inv(0.9, 2).*diag(D)));
                case Termination_Method.circle
                                                % percentge of particles
                                                % inside a circle with
                                                % radius r
                    center = mean(density.particles(1:2, :), 2);
                    d = sqrt(sum((density.particles(1:2,:) - center).^2));
                    val = mean(d > obj.config.threshold(2)); % percentage of particles inside circle
                                                
                otherwise
                    error('Error: termination method not implemented.\n');
            end
            
            isfound = val < obj.config.threshold(1);
       end
        
   end
    
 
end




