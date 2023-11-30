%% [System & Simulation setting]

function area_config = init_area_config(varargin)
    %% --- Instantiate inputParser
    p = inputParser;
    addParameter(p, 'DEM', []);
    addParameter(p, 'GPS', []);
    addParameter(p, 'Vertex', []);
    
    parse(p, varargin{:});
    
    if ~isempty(p.Results.DEM)
        area_config.DEM = load(p.Results.DEM);
    else
        area_config.DEM.DATA = ones(5000, 5000);
    end
    
    if ~isempty(p.Results.GPS)
        area_config.area = Rect_GPS_to_cart_area(area_config.DEM.ref, p.Results.GPS);
    elseif ~isempty(p.Results.Vertex)
        area_config.area = p.Results.Vertex;
    end
    
    % find the altitude range given area boundary
    x_min = round(min(area_config.area(1,:)));
    x_max = round(max(area_config.area(1,:)));
    y_min = round(min(area_config.area(2,:)));
    y_max = round(max(area_config.area(2,:)));
    
    area_config.Alt_range = [min(area_config.DEM.DATA(x_min:x_max, y_min:y_max), [], 'all') - 3; ...
                             max(area_config.DEM.DATA(x_min:x_max, y_min:y_max), [], 'all') + 3];
    
end