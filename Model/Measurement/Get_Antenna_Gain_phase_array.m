%% cauculate the antenna pattern of a 2 point phase array

function [gain,phi, r] = Get_Antenna_Gain_phase_array(target_pos, uav_pos, gain_pattern, varargin)
%% return gain, incident angle and distance from TX to RX
    p = inputParser;
    addParameter(p, 'freq', 150e6);         % frequency of signal
    addParameter(p, 'gap', 0.5);            % distance between array element
    addParameter(p, 'wave_speed', 3e8);     % speed of EM wave

    pos_relative = (target_pos(1:3,:)-uav_pos(1:3,:));

    %% NOTE: NS coordinate, 0 degree at +Y
    [theta,phi,r] = cart2sph(pos_relative(1,:),pos_relative(2,:),pos_relative(3,:)); % convert from cartesian to spherical coordinate
     theta = -theta+pi/2;

    gain = 10*log10(abs(exp(1j*pi/2) + exp(1j*2*pi*f*p.Results.gap*cos(theta-uav_pos(4)).*abs(cos(phi))/p.Results.wave_speed))/2);

    % capped the minimal gain to -25 dB
    gain(gain<-25) = -25;
    gain(isnan(gain)) = -25;
    gain(isinf(gain)) = -25;
        
end