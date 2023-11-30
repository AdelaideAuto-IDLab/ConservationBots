function [area, v] = cal_gps_area(gps_coord)
    %% Calculate polygon area given column vector of gps coordinates
    n_point = size(gps_coord, 2);
    v = nan(2, n_point);
    
    for i = 1:n_point
        v(:, i) = GPS_Coord.GPS2Cart(gps_coord(:, 1), gps_coord(:,i));
    end
    
    area = polyarea(v(1,:), v(2,:));
end


