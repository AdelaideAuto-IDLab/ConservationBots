%% result plot

% plot setting
tick_font_size = 13;


%% flat terrain
X_label = categorical({'Cauchy', 'Renyi', 'Shannon'});
X_label = reordercats(X_label, {'Cauchy', 'Renyi', 'Shannon'});
colorset = [0.000000000000000,  0.450980392156863,  0.741176470588235; ...
            0.5, 0.5, 0.5; ...
            0.668, 0.668, 0.129];
%
% 1. no void
% format: Column [Combine, RSSI, AoA]
%         Row    [Cauchy; Renyi; Shannon] 
%
Time_flat_wo_void = [1137, 1156, 6917; ...
                     1187, 1178, 7351; ...
                     1138, 1159, 7146];
Time_std_flat_wo_void = [68.90, 86.19, 721.85; ...
                         57.46, 85.13, 841.94; ...
                         55.97, 82.20, 748.40];
RMSE_flat_wo_void = [17.1, 15.4, 11.2; ...
                     18.2, 15.6, 11.9; ...
                     17.0, 15.7, 11.7];
RMSE_std_flat_wo_void = [2.11, 2.05, 1.86; ...
                        2.20, 2.18, 2.69; ...
                        2.08, 2.34, 2.61];


%
% 3. with void
% 
Time_flat_w_void = [1207, 1185, 11188; ...
                    1219, 1156, 10318; ...
                    1202, 1171, 10719];
Time_std_flat_w_void = [102.69, 108.85, 1273.12; ...
                        79.00, 81.45, 1616.69; ...
                        79.27, 100.16, 1436.39];  
RMSE_flat_w_void = [16.3, 14.9, 14.6; ...
                    17.5, 15.2, 13.9; ...
                    16.6, 14.7, 13.8];
RMSE_std_flat_w_void = [1.84, 1.90, 3.29; ...
                        2.01, 2.18, 2.82; ...
                        1.95, 1.59, 2.65];

% =============================== no void ======================
figure('Position', [10,10,400, 300]);
h1 = barwitherr(3*Time_std_flat_wo_void, Time_flat_wo_void);
xticklabels(X_label);
legend('Ours', 'RSSI', 'AoA', 'Location', 'NorthEastOutside');
ylabel('Time (s)');
grid minor;
ax = gca;
ax.FontSize = tick_font_size;
% ylim([0, 10000]);
% set color
for n = 1:3
    h1(n).FaceColor = colorset(n,:);
end
% TODO add numbers on top


figure('Position', [10,10,400, 300]);
h2 = barwitherr(3*RMSE_std_flat_wo_void, RMSE_flat_wo_void);
xticklabels(X_label);
legend('Ours', 'RSSI', 'AoA', 'Location', 'NorthEastOutside');
ylabel('RMSE (m)');
ylim([0, 50]);
grid minor;
ax = gca;
ax.FontSize = tick_font_size;
% set color
for n = 1:3
    h2(n).FaceColor = colorset(n,:);
end

% =========================== with void =========================
figure('Position', [10,10,400, 300]);
h3 = barwitherr(3*Time_std_flat_w_void, Time_flat_w_void);
xticklabels(X_label);
legend('Ours', 'RSSI', 'AoA', 'Location', 'NorthEastOutside');
ylabel('Time (s)');
grid minor;
ax = gca;
ax.FontSize = tick_font_size;
% ylim([0, 10000]);
% set color
for n = 1:3
    h3(n).FaceColor = colorset(n,:);
end
% TODO add numbers on top


figure('Position', [10,10,400, 300]);
h4 = barwitherr(3*RMSE_std_flat_w_void, RMSE_flat_w_void);
xticklabels(X_label);
legend('Ours', 'RSSI', 'AoA', 'Location', 'NorthEastOutside');
ylabel('RMSE (m)');
ylim([0, 50]);
grid minor;
ax = gca;
ax.FontSize = tick_font_size;
% set color
for n = 1:3
    h4(n).FaceColor = colorset(n,:);
end


% % VarN
% %
% % Format: column: [target nummbers]
% %         row:    [Combine; RSSI; AoA]
% %
% x = (1:2:19);
% Time_varN_flat = [363,  536,  689,  768,  1088, 1199, 1309, 1455, 1547, 1665; ...
%                   368,  632,  853,  970,  1327, 1500, 1637, 1852, 1941, 2118; ...
%                   1115, 1715, 2717, 3423, 3968, 4928, 5095, 5563, 6106, 6683];
% Time_varN_flat_std = [23.15, 40.60, 44.71, 57.74 ,67.84, 91.50, 74.74, 85.61, 83.91, 89.51; ...
%                       48.22, 106.58, 107.92, 126.51, 126.09, 158.61, 142.66, 192.26, 177.62, 93.68; ...
%                       468.62, 553.72, 718.24, 660.21, 749.12, 751.91, 733.61, 789.10, 969.87, 844.34];
% RMSE_varN_flat = [19.7, 21.5, 20.6, 20.2, 19.5, 18.3, 18.0, 16.5, 18.3, 18.5;...
%                   14.0, 14.4, 12.8, 12.0, 12.6, 12.1, 12.5, 11.9, 12.1, 12.3;...
%                   21.2, 28.6, 32.2, 21.1, 22.8, 22.1, 21.6, 18.8, 21.2, 18.3];
% RMSE_varN_flat_std = [11.87, 8.77, 5.99, 4.84, 3.85, 3.05, 3.18, 3.00, 2.88, 2.96; ... 
%                       9.90, 4.17, 3.49, 2.83, 2.25, 2.06, 1.95, 1.96, 1.83, 1.81; ...
%                       38.05, 61.42, 56.87, 17.05, 25.79, 27.15, 20.49, 12.91, 15.36, 8.66];
% 
% figure();
% hold on;
% errorbar(x, Time_varN_flat(1,:), 3*Time_varN_flat_std(1,:), 'LineStyle', '--');
% errorbar(x, Time_varN_flat(2,:), 3*Time_varN_flat_std(2,:), 'LineStyle', '-');
% errorbar(x, Time_varN_flat(3,:), 3*Time_varN_flat_std(3,:), 'LineStyle', '-.');
% xlabel('Target Number');
% ylabel('Time (s)');
% legend('Ours', 'RSSI', 'AoA');
% grid minor;
% 
% figure();
% hold on;
% errorbar(x, RMSE_varN_flat(1,:), 3*RMSE_varN_flat_std(1,:), 'LineStyle', '--');
% errorbar(x, RMSE_varN_flat(2,:), 3*RMSE_varN_flat_std(2,:), 'LineStyle', '-');
% errorbar(x, RMSE_varN_flat(3,:), 3*RMSE_varN_flat_std(3,:), 'LineStyle', '-.');
% xlabel('Target Number');
% ylabel('RMSE (m)');
% legend('Ours', 'RSSI', 'AoA');
% grid minor;

