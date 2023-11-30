function [meas, meas_uav] = filter_pulse_by_id(z, id, id_list)
    if istable(z)
        idx = find(z.Target_ID == id_list(id));
        meas_uav = z.UAV(idx, :)';
        meas = table2array(z(idx, [1,3]));
        
%         if nargin == 4 && ~isempty(previous_z) && ~isempty(meas)
%             idx2 = find(previous_z.Target_ID == id_list(id));
%             if ~isempty(idx2) &&  ~isnan(previous_z.RSSI(idx2))
%                 if meas(1,2) - previous_z.RSSI(idx2) < -25
%                     meas(1, 2) = nan;
%                 end
%             end
%         end

    else
        meas = [];
        meas_uav = [];
    end
end