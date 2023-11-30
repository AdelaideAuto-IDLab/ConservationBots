%% Main problem for SITL
clear; close all;

%% Global configuration
% list of truth location
target_id_list = [1];
%target_truth = [-35.469361, -35.469361, -35.469361; 138.489903, 138.489903, 138.489903];
target_truth = [-35.459222; 138.51505];
ntarget = size(target_truth, 2);
cal_radius = 20;
n_sample = 100;
phi = 0;
DEM = 'Victor_alt_site.mat';

raw_meas_record = cell(n_sample, ntarget);
meas = nan(n_sample, 1);
uav = nan(4, n_sample);

%% Initialization
addpath(genpath(pwd));

% generate module settings
uav_config = init_uav_config('vmax', 2);  
area_config = init_area_config( ...
                'Vertex', [1,1; 1, 1209; 1970, 1209; 1970, 1]', ...
                'DEM', DEM); 

%% ====================== SITL initialization ============================
comm = UAV_Comm('filter_config_udoo.json', 'Debug', false);

%% ======================= Main Program =============================
pause(0.5);

% get home position & initialize state
home = comm.get_home_pos();
% initialize initial position
uav0 = GPS_Coord.GPS2Cart_struct(area_config.DEM.ref, home);
warning('The elevation in uav state is relative elevation, not AMSL.');

% convert target truth to cartesian coordinate
truth = nan(3, ntarget);
for n = 1:ntarget
    truth(1:2, n) = GPS_Coord.GPS2Cart([area_config.DEM.ref.lat; area_config.DEM.ref.lon], target_truth(:, n));
end
truth(3,:) = 2;

comm.yaw(rad2deg(atan2(truth(1) - uav0(1), truth(2) - uav0(2))));

% generate waypoint
[wpn, theta] = gen_waypoint(uav0, truth(:, 1), phi, cal_radius, n_sample);

% ======================== Main Loop ================================
n = 0;
pause(0.5);
while n < n_sample
    fprintf('%i/%i Measurement.\n',n, n_sample);
    n = n+1;
    loop_time = tic;
%     
%     % move to waypoint
%     active_waypoint = GPS_Coord.Cart2GPS_struct(area_config.DEM.ref, wpn(:,n));
%     comm.move_global(active_waypoint);
%     
%     % ============= update uav location ========================
%     while(1)
%         current_gps = comm.get_gps();
%         if ~isempty(current_gps)
%             uav(:,n) = GPS_Coord.GPS2Cart_struct(area_config.DEM.ref, current_gps);
%         end
%         
%         if sqrt(sum(uav(1:2, n) - wpn(1:2, n)).^2) < 1.3 && abs(uav(3,n) - wpn(3,n)) < 0.5
% %         if sqrt(sum((uav(1:3, n) - wpn(1:3, n)).^2)) < 1  % waypoint reached
%             fprintf('Waypoint reached. \n');
%             break;
%         end
%         comm.move_global(active_waypoint);
%         pause(1);
%     end
%     
    % ============= generate measurement =======================
    while(1)
        pulses = comm.get_pulses();
        disp(pulses);
        z = convert_pulses(pulses, home);
        z = remove_invalid_id(z, target_id_list);
        if z.Target_ID == 1
            raw_meas_record{n} = z;
            meas(n) = z.RSSI;
            break;
        end
        
        pause(0.2);
    end
    
    while toc(loop_time) < 1
        pause(0.05);
    end
      
end

%% ====================================================================
scatter(theta, meas);
xlabel('Theta (rad)');
ylabel('Meas (dBm)');

% % save record
%filename = strcat('Victor_alt_site/', 'Verticle_Pattern', num2str(rad2deg(phi)), 'T-', datestr(now,'mm-dd-HH_MM'), '.mat');
%save(filename);

% close tcp connection to mavlink-proxy
comm.close_tcp();


function [wpn, theta] = gen_waypoint(uav, target, phi, r, npoints)
    wpn = nan(4, npoints);
    theta = nan(npoints, 1);
    ang_uav_target = atan2(target(2) - uav(2), target(1) - uav(1));

    init_ang = asin((uav(3) - target(3))/r);
    totle_ang = pi - 2*init_ang;
    d_ang = totle_ang / (npoints-1);
    
    for n = 1:npoints
       theta(n) = init_ang + d_ang*(n-1);
       wpn(:, n) = [r*cos(theta(n)); 0; r*sin(theta(n)); ang_uav_target];
    end
    
    % rotate waypoints
    tmpy = wpn(2,:)*cos(phi) - wpn(3,:)*sin(phi);
    tmpz = wpn(2,:)*sin(phi) + wpn(3,:)*cos(phi);
    wpn(2,:) = tmpy;
    wpn(3,:) = tmpz;
    tmpx = wpn(1,:)*cos(ang_uav_target) - wpn(2,:)*sin(ang_uav_target);
    tmpy = wpn(1,:)*sin(ang_uav_target) + wpn(2,:)*cos(ang_uav_target);
    wpn(1,:) = tmpx;
    wpn(2,:) = tmpy;

    
    % translation
    wpn(1,:) = wpn(1,:) + target(1);
    wpn(2,:) = wpn(2,:) + target(2);
    wpn(3,:) = wpn(3,:) + target(3);  
end
