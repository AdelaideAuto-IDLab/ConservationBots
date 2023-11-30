function result = read_result_single(filename)
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
    result.meas_stats = raw.measurement_stats;
end