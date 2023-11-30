%% plot measured gain pattern vs model

meas_pattern = load('gain_pattern_trial.mat');

% plot pattern
figure();
% normalize measured data
avg_s = (meas_pattern.avg_s - min(meas_pattern.avg_s));
avg_s = avg_s / max(avg_s);
h = polarplot(deg2rad((240:10:240+360)), avg_s, 'LineWidth', 3);
haxes=get(h, 'Parent');
haxes.RTickLabel = {'-20', '-15', '-10', '-5', '0', '5'};
hold on;

% % calculate model gain pattern
model_data = load('H_antenna.mat');
i = deg2rad((0:10:350));
x = [20*cos(i); 20*sin(i); 5*ones(1, 36)];
[gain] = Get_Antenna_Gain(x, [0, 0, 5, 0]', model_data);
gain = gain - min(gain);
gain = gain/max(gain);
polarplot(gain, '--', 'LineWidth', 3);
ax = gca;
ax.FontSize = 16;

legend('Measured', 'Model');