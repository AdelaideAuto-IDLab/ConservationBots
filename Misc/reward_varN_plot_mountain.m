%% result plot


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
Time_mountain_wo_void = [2184, nan, 5436; 2195, nan, 5135; 2152, nan, 5371];
Time_std_mountain_wo_void = [143.45, nan, 790.25; 137.76, nan, 659.58; 136.37, nan, 680.43];
RMSE_mountain_wo_void = [34.43, nan, 22.14;36.53, nan, 25.7; 35.79, nan, 27.39];
RMSE_std_mountain_wo_void = [8.52, nan, 11.56;10.15, nan, 16.61; 8.95, nan, 18.85];


%
% 3. with void
% 
Time_mountain_w_void = [2317, nan, 7178; 2328, nan, 6942; 2331, nan, 7114];
Time_std_mountain_w_void = [139.66, nan, 884.3; 141.41, nan, 895.8; 138.34, nan, 904.79];  
RMSE_mountain_w_void = [32.5, nan, 18.2; 34.80, nan, 19.4; 33.05, nan, 19.56];
RMSE_std_mountain_w_void = [9.59, nan, 10.12; 11.83, nan, 12.67; 8.57, nan, 11.34];

% =============================== no void ======================
figure('Position', [10,10,400, 300]);
h1 = barwitherr(3*Time_std_mountain_wo_void, Time_mountain_wo_void);
xticklabels(X_label);
legend('Combine', 'RSSI', 'AoA', 'Location', 'NorthWest');
ylabel('Time (s)');
grid minor;
ylim([0, 10000]);
% set color
for n = 1:3
    h1(n).FaceColor = colorset(n,:);
end
% TODO add numbers on top


figure('Position', [10,10,400, 300]);
h2 = barwitherr(3*RMSE_std_mountain_wo_void, RMSE_mountain_wo_void);
xticklabels(X_label);
legend('Ours', 'RSSI', 'AoA', 'Location', 'NorthWest');
ylabel('RMSE (m)');
ylim([0, 50]);
grid minor;
% set color
for n = 1:3
    h2(n).FaceColor = colorset(n,:);
end
% =========================== with void =========================
figure('Position', [10,10,400, 300]);
h3 = barwitherr(3*Time_std_mountain_w_void, Time_mountain_w_void);
xticklabels(X_label);
legend('Ours', 'RSSI', 'AoA', 'Location', 'NorthWest');
ylabel('Time (s)');
grid minor;
ylim([0, 10000]);
% set color
for n = 1:3
    h3(n).FaceColor = colorset(n,:);
end
% TODO add numbers on top


figure('Position', [10,10,400, 300]);
h4 = barwitherr(3*RMSE_std_mountain_w_void, RMSE_mountain_w_void);
xticklabels(X_label);
legend('Ours', 'RSSI', 'AoA', 'Location', 'NorthWest');
ylabel('RMSE (m)');
ylim([0, 50]);
grid minor;
% set color
for n = 1:3
    h4(n).FaceColor = colorset(n,:);
end

