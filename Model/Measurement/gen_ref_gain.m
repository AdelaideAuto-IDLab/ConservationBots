function gain = gen_ref_gain(uav, xk, antenna_gain, varargin)
%% calculate the reference antenna gain
%% assume target is located at height 0 and 250 m away if target estimate is not present


p = inputParser;
addParameter(p, 'Antenna', 'H'); 
parse(p, varargin{:});

r = 250;
if isempty(xk)
   ref_target = [r*cos(-uav(4)), r*sin(-uav(4)), 0]'; 
else
    % uav to target horizontal distance
   d = sqrt(sum((uav(1:2) - xk(1:2)).^2));
   ref_target = [d*cos(-uav(4)), d*sin(-uav(4)), uav(3)]';
end
ref_target(1:2) = ref_target(1:2) + uav(1:2);

ref_uav = uav;
ref_uav(4) = 0;

if strcmp(p.Results.Antenna, 'H')
    gain = Get_Antenna_Gain(ref_target, ref_uav, antenna_gain);
elseif strcmp(p.Results.Antenna, 'Array')
    gain = Get_Antenna_Gain_phase_array(ref_target, ref_uav);
end


end