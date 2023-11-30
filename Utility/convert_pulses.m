function z = convert_pulses(pulses, gps_base)
    % convert pulses's UAV coordinate from lon-lat to x-y
    if isempty(pulses)
        z = [];
    else
        z = pulses;
        for n = 1:height(pulses)
            z.UAV(n, 1:2) = GPS_Coord.GPS2Cart([gps_base.lat; gps_base.lon], [z.UAV(n,1); z.UAV(n,2)]);
        end
    end
end