classdef GPS_Coord
    methods (Static)
        %% IMPORTANT: Gps coordinate: latituide comes first
        % Convert GPS to Cartesian coordinate
        function [X] = GPS2Cart(gps_base, gps_point)
            % gps_base: [latitude; lontitude]
            X = zeros(size(gps_point));
            
            % Correct after compare with http://andrew.hedges.name/experiments/haversine/
            lat_average = deg2rad(gps_point(1,:)+ gps_base(1,:))/2.0;
            lat_average_cos = cos(lat_average);
            X(1,:) = deg2rad(gps_point(2,:) - gps_base(2,:)) * 6371e3 .* lat_average_cos;
            X(2,:) = (gps_point(1,:) - gps_base(1,:)) * 110540.0;
        end  
        
        % convert Cartesian coordinate to GPS 
        function gps_point = Cart2GPS(gps_base, X)
            gps_point = zeros(size(X));
            
            lat_offset = X(2,:)/110540;
            lon_offset = X(1,:)/(111320*cosd(gps_base(1)));

            gps_point(1,:) = gps_base(1) + lat_offset;
            gps_point(2,:) = gps_base(2) + lon_offset;
        end
        
        % Convert GPS to Cartesian Coordinate
        % Input: stuct
        function [X] = GPS2Cart_struct(gps_base, gps_point)
            if(size(gps_point, 1) ~= 1)
                error('Only one GPS point is accepted.');
            end
            
            X = zeros(4,1);
            X(1:2) = GPS_Coord.GPS2Cart([gps_base.lat; gps_base.lon], [gps_point.lat; gps_point.lon]);
%             X(3) = gps_point.alt;
            X(3) = gps_point.relative_alt;
            X(4) = deg2rad(gps_point.hdg);
        end
        
        % Convert GPS to Cartesian Coordinate
        % Input: stuct
        function [gps] = Cart2GPS_struct(gps_base, X)
            if(size(X, 2) ~= 1)
                error('Only one waypoint is accepted.');
            end
            
            tmp_gps = GPS_Coord.Cart2GPS([gps_base.lat; gps_base.lon], X);
            gps.lat = tmp_gps(1);
            gps.lon = tmp_gps(2);
            gps.relative_alt = X(3);
            gps.hdg = rad2deg(X(4)); 
        end   
        
    end
    
end