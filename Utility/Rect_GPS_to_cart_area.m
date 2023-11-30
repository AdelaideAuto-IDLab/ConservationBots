function area = Rect_GPS_to_cart_area(ref, vertex)
    %% generate rectangular area given two point
    % format:
    % ref.lat, ref.lon
    
    ref_mat = [ref.lat; ref.lon];
    if size(vertex, 2) == 2
        % vertex == [lat; lon], first point being left bottom
        p1 = GPS_Coord.GPS2Cart(ref_mat, vertex(:, 1));
        p2 = GPS_Coord.GPS2Cart(ref_mat, vertex(:, 2));

        area = [p1(1), p2(1), p2(1), p1(1), p1(1); ...
                p1(2), p1(2), p2(2), p2(2), p1(2)];
    elseif size(vertex, 2) > 2
        area = nan(2, size(vertex, 2)+1);
        for i = 1:size(vertex, 2)
            area(:, i) = GPS_Coord.GPS2Cart(ref_mat, vertex(:, i));
        end
        area(:, end) = area(:, 1);
    else
        error('Boundary: not enough points.\n');
    end
end