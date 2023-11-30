function [meas, hdg, uav] = bearing_raw_extract(raw_measurement, target_id, plot_fig)
%% Extract measurement during rotation bearing measurement
    n_rotate = length(raw_measurement);
    n_meas = length(raw_measurement{1});
    
    meas = nan(n_rotate, n_meas);
    hdg = nan(n_rotate, n_meas);
    uav = cell(n_rotate, 1);
    
    for k = 1:n_rotate
        uav{k} = nan(4, n_meas);
        for i = 1:n_meas
            if iscell(raw_measurement{k}) && ~isempty(raw_measurement{k}{i})
                idx = find(raw_measurement{k}{i}.Target_ID == target_id);
                if ~isempty(idx)
                    meas(k, i) = raw_measurement{k}{i}.RSSI(idx);
                    hdg(k, i) = raw_measurement{k}{i}.UAV(idx, 4);
                    uav{k}(:, i) = raw_measurement{k}{i}.UAV(idx, :)';
                end
            end
        end
    
    end
    hdg = wrapTo2Pi(hdg);
    
    
    if plot_fig
       figure();
       polarplot(nan,nan);
       hold on;
       for k = 1:n_rotate
           polarplot(hdg(k, :), meas(k, :));
       end
       rlim([min(meas, [], 'all') - 10, max(meas, [], 'all') + 10]);
    end


end