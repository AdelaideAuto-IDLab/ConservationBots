%% plot Varying number of target figures


filename = uigetfile('*.mat', 'MultiSelect', 'on');
if ~iscell(filename)
    filename = {filename};
end



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
    fprintf('Time: %f (s), std: %f (s) \n', mean(result.time), std(result.time));
    fprintf('RMSE: %f (m), std: %f (m) \n', mean(result.error), std(result.error));
    fprintf('Distance: %f (m), std: %f (m) \n', mean(result.dist), std(result.dist));
    fprintf('===================================\n\n');
end


target_n = (1:2:2*length(filename)-1);

% plot time vs target
figure();
shadedErrorBar(target_n, time, {@mean, @std});
xlabel('Target Number');
ylabel('Time (s)');


% plot error vs target
figure();
shadedErrorBar(target_n, error, {@mean, @std});
xlabel('Target Number');
ylabel('RMSE (m)');

% plot distance vs target
figure();
shadedErrorBar(target_n, dist, {@mean, @std});
xlabel('Target Number');
ylabel('Travel Distance (m)');








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
end