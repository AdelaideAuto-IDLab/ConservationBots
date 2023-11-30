function [total_detection, receive_detection] = cal_missed_detection(filename)

    lc = line_count(filename);
    N = zeros(lc, 1);

    fid = fopen('pulse_log.jsonl');

    tline = fgetl(fid);
    i = 1;
    while ischar(tline)
        de_json = jsondecode(tline);
        N(i,1) = de_json.PULSE.N;
        i = i + 1;
        tline = fgetl(fid);
    end

    % start from 0
    start_i = find(N == 0);

    total_detection = nan(length(start_i), 1);
    receive_detection = nan(length(start_i), 1);

    for m = 1:length(start_i)
        if m == length(start_i)
            total_detection(m, 1) = N(end, 1);
            receive_detection(m, 1) = length(N) - start_i(m);
        else
            total_detection(m, 1) = N(start_i(m+1)-1, 1);
            receive_detection(m, 1) = start_i(m+1) - start_i(m);
        end
    end
end


function lc = line_count(filename)
    [status, cmdout] = system(['wc -l ', filename]);
    if (status ~= 1)
        scanCell = textscan(cmdout, '%u %s');
        lc = scanCell{1};
    else
        fprintf('Failed to find line count');
        lc = -1;
    end
end