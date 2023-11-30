function DATA = convert_geotiff(filename, gps_ws, gps_ne, ID)
    raw = geotiffread(filename);
    
    xy = GPS_Coord.GPS2Cart(gps_ws, gps_ne);
    
    x_len = round(xy(1));
    y_len = round(xy(2));
    
    DATA = imresize(raw, [y_len, x_len])';
    
    X_Min = 0;
    X_Max = x_len;
    Y_Min = 0;
    Y_Max = y_len;
    ref.lat = gps_ws(1);
    ref.lon = gps_ws(2);
    ref.alt = DATA(1,1);
    resolution = 1;
    Alt_range = [min(DATA, [], 'all'), max(DATA, [], 'all')];
    
    [save_filename, save_path] = uiputfile({'*.mat'});
    
    save(strcat(save_path,save_filename),  'X_Min', 'X_Max', 'Y_Min', 'Y_Max', ...
                                           'ref', 'resolution', 'ID', 'Alt_range', 'DATA');
    
end