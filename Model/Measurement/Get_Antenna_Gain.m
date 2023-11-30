%% NOTE: the the antenna gain pattern data use ES coordinate (0 degree start at +x axis)
%% Here we convert the axis to NE coordinate

function [gain,phi, r] = Get_Antenna_Gain(target_pos, uav_pos, gain_pattern)
%% calculate antenna gain given TX and RX state
%% return gain, incident angle and distance from TX to RX
 
        if size(target_pos,2) > size(uav_pos,2)
           uav_pos = repmat(uav_pos,1,size(target_pos,2));
        else
           target_pos = repmat(target_pos,1,size(uav_pos,2));
        end

        pos_relative = (target_pos(1:3,:)-uav_pos(1:3,:));
        theta = wrapTo2Pi(atan2(pos_relative(1, :), pos_relative(2, :)) - uav_pos(4,:));
        
        
        gh = gain_pattern.gh';
        ge = gain_pattern.ge';
        N = gain_pattern.N;

        [~,phi,r] = cart2sph(pos_relative(1,:),pos_relative(2,:),pos_relative(3,:)); % convert from cartesian to spherical coordinate
        phi = wrapTo2Pi(phi);
        gain = 10*log10(gh(floor(phi/(2*pi)*N)+1)) + 10*log10(ge(floor(theta/(2*pi)*N)+1));
        
end