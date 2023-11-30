classdef MAV_FRAME < int32
    enumeration
        MAV_FRAME_GLOBAL                    (0)     % WGS84 coordinate + MSL altitude
        MAV_FRAME_LOCAL_NED                 (1)     % Local coordinate, z-down (x: north, y: east, z, down)
        MAV_FRAME_GLOBAL_RELATIVE_ALT       (3)     % WGS84 coordinate + relative alt
        MAV_FRAME_GLOBAL_INT                (5)     % WGS84 coordinate + MSL altitude, x,y *1e7
        MAV_FRAME_GLOBAL_RELATIVE_ALT_INT   (6)     % WGS84 coordinate + altitude relative to home, x:lat*1e7. y:lon*1e7
        MAV_FRAME_LOCAL_OFFSET_NED          (7)     % offset to current frame
        MAV_FRAME_GLOBAL_TERRAIN_ALT_INT    (11)    % WGS84 coordinate with AGL altitude, x,y *1e7
        MAV_FRAME_BODY_FRD                  (12)    % Boxy fixed frame, z-down
    end
end