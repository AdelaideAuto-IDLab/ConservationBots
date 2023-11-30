function waypoint_list = Generate_waypoint_pattern(area, movetime, gap, dir, mode, alt)
    % generate lawn mower path
    
    
    if strcmp(dir, 'NS')
        tmp = round(max(area(2,:)) - min(area(2,:))) - gap*2;
        long = tmp - mod(tmp, gap);
        short = round(max(area(1,:)) - min(area(1,:))) - gap*2;
    elseif strcmp(dir, 'WE')
        tmp = round(max(area(1,:)) - min(area(1,:))) - gap*2;
        long = tmp - mod(tmp, gap);
        short = round(max(area(2,:)) - min(area(2,:))) - gap*2;
    end
    
    n_gap = floor(short/gap);
    n_waypoint = 2*(n_gap+1);
    waypoint = nan(2, n_waypoint);

    if strcmp(dir, 'NS')
     waypoint(1,:) = area(1,1)+ gap + gap*repelem((0:1:n_waypoint/2-1), 2);
     tmp = repmat([0, long, long, 0], 1, ceil(n_gap/2+1));
     waypoint(2,:) = area(2,1) + gap + tmp(1, 1:n_waypoint);
    elseif strcmp(dir, 'WE')
     tmp = repmat([0, long, long, 0], 1, ceil(n_gap/2+1));
     waypoint(1,:) = area(1,1) + gap + tmp(1, 1:n_waypoint); 
     waypoint(2,:) = area(2,1)+ gap + gap*repelem((0:1:n_waypoint/2-1), 2);
    end
    
    tmp_x = diff(waypoint(1,:));
    tmp_y = diff(waypoint(2,:));
    heading = atan2(tmp_x, tmp_y);
    heading = [heading, heading(end)];
    
    waypoint = [waypoint; alt*ones(1, n_waypoint); heading];
    
    
    if mode == Action_Type.RSSI
        waypoint_list = cell(1, n_waypoint);
        for n = 1:n_waypoint
            waypoint_list{n}.type = mode;
            waypoint_list{n}.waypoints = waypoint(:, n);
        end
        
    elseif mode == Action_Type.AoA || mode == Action_Type.Combine
        n_subwaypoint = round(long/gap);
        insert_count = n_subwaypoint - 1;
        waypoint_list = cell(1, n_waypoint + (n_gap+1)*insert_count);
        
        for n = 1:n_waypoint
            if n == 1
                waypoint_list{n}.type = mode;
                waypoint_list{n}.waypoints = waypoint(:, n);
                
                global_count = 2;
            else
               % see if new waypoint need to be added 
               d = sqrt( sum((waypoint(1:2, n) - waypoint(1:2, n-1)).^2) );
               if d > gap
                   dx = sign(waypoint(1, n) - waypoint(1, n-1));
                   dy = sign(waypoint(2, n) - waypoint(2, n-1));
                   for m = 1:insert_count+1
                       waypoint_list{global_count}.type = mode;
                       waypoint_list{global_count}.waypoints = waypoint(:, n-1) + m*gap*[dx;dy;0;0];
                       global_count = global_count + 1;
                   end
               else
                   waypoint_list{global_count}.type = mode;
                   waypoint_list{global_count}.waypoints = waypoint(:, n);
                   global_count = global_count + 1;
               end
            end
        end
        
        
        % expand time
        for n = 1:length(waypoint_list)
            waypoint_list{n}.waypoints = repmat(waypoint_list{n}.waypoints, 1, movetime);
        end
        
    end
    

end

