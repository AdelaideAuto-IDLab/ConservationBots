%% Main problem for SITL
clear; close all;
%% Global configuration
Total_time = 800;                      % maximun simulation time
max_target = 7;
snr_threshold = -85;
%% Initialization
addpath(genpath(pwd));

%% ====================== SITL initialization ============================
comm = UAV_Comm('filter_config_udoo.json', 'Debug', true);

color_table = {'#000000','#00FF00','#0000FF','#FF0000','#01FFFE','#FFA6FE','#FFDB66','#006401', ...
               '#010067','#95003A','#007DB5','#FF00F6','#FFEEE8','#774D00','#90FB92','#0076FF', ...
               '#D5FF00','#FF937E','#6A826C','#FF029D','#FE8900','#7A4782','#7E2DD2','#85A900', ...
               '#FF0056','#A42400','#00AE7E','#683D3B','#BDC6FF','#263400','#BDD393','#00B917', ...
               '#9E008E','#001544','#C28C9F','#FF74A3','#01D0FF','#004754','#E56FFE','#788231', ...
               '#0E4CA1','#91D0CB','#BE9970','#968AE8','#BB8800','#43002C','#DEFF74','#00FFC6', ...
               '#FFE502','#620E00','#008F9C','#98FF52','#7544B1','#B500FF','#00FF78','#FF6E41', ...
               '#005F39','#6B6882','#5FAD4E','#A75740','#A5FFD2','#FFB167','#009BFF','#E85EBE'};


% figure setting
f = figure();
hold on;
hl = gobjects(max_target, 1);
for n = 1:max_target
    hl(n) = plot(nan, nan, 'LineWidth', 2, 'Color', color_table{n});
end
grid on;
xlabel('Time (s)');
ylabel('RSSI (dBm)');
ylim([-90, 2]);


pulse_id_list = [];
pulse_record = nan(max_target, Total_time);
prev_length = 0;

%% ======================= Main Program =============================
t = 0;
while (t < Total_time) 
    fprintf('Epos: %i/%i.\n', t, Total_time);
    t = t+1;    % increment simulation time
    loop_time = tic;
    
    % ============= generate measurement =======================
    pulses = comm.get_pulses();
    if ~isempty(pulses)
        fprintf('Current detected pulse list:\n');
        disp(pulse_id_list);
        fprintf('\nCurrent Pulses:\n');
        disp(pulses);
        prev_length = length(pulse_id_list);
        pulse_id_list = union(pulse_id_list, pulses.Target_ID)';
        for n = 1:height(pulses)
            idx = find(pulses.Target_ID(n) == pulse_id_list);
            if pulses.RSSI(n) > snr_threshold
                pulse_record(idx, t) = pulses.RSSI(n);
            end
        end
    end 
    
    % ====================== update plot =========================
    for n = 1:length(pulse_id_list)
        set(hl(n), 'XData', (1:t), 'YData', pulse_record(n, 1:t));
    end
    if prev_length ~= length(pulse_id_list)
        legend(hl(1:length(pulse_id_list)), string(pulse_id_list(1:length(pulse_id_list))), 'Location', 'NorthEastOutside');
    end
    drawnow();
   
    % wait until looptime reach 1 second
    while toc(loop_time) < 1
        pause(0.1);
    end
    
end
