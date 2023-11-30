function waypoint = Cart2Waypoint(gps_base, X, relative_height)

%% Convert Cartesian coordinate to waypoint

    if size(X, 2) ~= 1
        error('Error: Only accept single coordinate column vector.\n');
    end

    gps = GPS_Coord.Cart2GPS(gps_base, X);
    waypoint.lat = gps(1);
    waypoint.lon = gps(2);
    waypoint.hdg = X(4);
    waypoint.relative_alt = relative_height;

end