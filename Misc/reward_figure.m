%% plot Varying number of target figures

% filename format:
% cauchy, cauchy(void), renyi, renyi(void), shannon, shannon(void), circle,
% circle (void)
filename = uigetfile('*.mat', 'MultiSelect', 'on');

if ~iscell(filename)
    filename = {filename};
end

id = {'Cauchy', 'Cauchy (Void)', ...
      'Renyi', 'Renyi (Void)', ...
      'Shannon', 'Shannon (Void)', ...
      'Circle', 'Circle (Void)'};

% collecting data
for f = 1:length(filename)
    result = read_result(filename{f}); 
    
    if f == 1
        mc = length(result.time);
        time = nan(mc, length(filename));
        error = nan(mc, length(filename));
        dist = nan(mc, length(filename));
    end
    
    time(:, f) = result.time;
    error(:, f) = result.error;
    dist(:, f) = result.dist;
    
    fprintf('ID: %s \n', id{f});
    fprintf('Time: %f (s), std: %f (s) \n', mean(result.time), std(result.time));
    fprintf('RMSE: %f (m), std: %f (m) \n', mean(result.error), std(result.error));
    fprintf('Distance: %f (m), std: %f (m) \n', mean(result.dist), std(result.dist));
    fprintf('Mean bearing percent: %f \n', mean(result.meas_count(:,1)./result.meas_count(:,2)));
    fprintf('===================================\n\n');
end





function result = read_result(filename)
    raw = load(filename);
    
    mc = length(raw.action_history);    % number of mc runs
    ntarget = raw.ntarget;
    
    error = nan(1, mc);
    tmp_error = nan(1, ntarget);
    
    for m = 1:mc
        for n = 1:ntarget
            tmp_error(n) = sqrt(sum((raw.est_history{m}{n}(1:2, raw.found_time(m, n)) - raw.truth(1:2, n, raw.found_time(m, n))).^2));
        end 
        error(m) = mean(tmp_error);
    end
    
    
    result.time = max(raw.found_time, [], 2);
    result.error = error';
    result.dist = raw.travel_distance;
    result.meas_count = raw.measurement_stats;
end