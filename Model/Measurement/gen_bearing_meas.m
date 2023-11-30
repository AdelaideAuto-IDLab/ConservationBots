
function bearing = gen_bearing_meas(ref_gain, measurements, uav_pos, gain_pattern, varargin)
%% gen_bearing_meas.m
%
% given reference gain pattern and measurement, calculate the bearing
%
% [ARGS]:
% ref_gain      - reference gain pattern vector
% measurements  - measurement vector
% uav_pos       - uav state when each measurement is taken
% gain_pattern  - antenna gain pattern
%
% [RETURN]:
%
% bearing       - bearing in radius
%               - Nan, invalid bearing
%


p = inputParser;
addParameter(p, 'Antenna', 'H'); 
parse(p, varargin{:});

% count the number of missed detection
miss_count = sum(measurements == 0) + sum(isinf(measurements));

if miss_count/length(measurements) <= 0.3

    %% remove 0 from measurements
    for i = 1:length(measurements)
        n = i;
        while(measurements(i) = = 0)
            n = n-1;
            measurements(i) = measurements(wrap_index(n, length(measurements)));
        end
    end


    % interpolate data
    interp_n = 10;
    l = length(ref_gain);
    meas_i = interp(measurements, interp_n);
    theta_i = interp(unwrap(uav_pos(4, :)), interp_n);
    theta_i = [zeros(3, l*interp_n); theta_i];
    % calculate ref_gain
    ref_i = zeros(size(meas_i));
    for i = 1:l*interp_n
        ref_i(i) = gen_ref_gain(theta_i(:, i), [], gain_pattern.pf.gain_angle, 'Antenna', p.Results.Antenna);
    end



    idx = angle_corr(meas_i, ref_i);
    %% NOTE: pi/2 is added to compensate for the coordinate difference in antenna pattern and gain calculation
    % antenna pattern assume NW coordinate (0 degree at +y)
    % gain calculation assume WE coordinate (0 degree at +x)
    bearing = wrapTo2Pi(theta_i(4, idx)- theta_i(4, 1) - pi/2); 
else
    % return nan if not enough sample for bearing measurement
    bearing = nan;
end

end

function i = wrap_index(i, lsize)
    if i <= 0
        i = lsize + i;
    elseif i > lsize
        i = i - lsize;
    end
end
