%% class for reading gps & measurement from file

classdef GPS_Pulse_Reader < handle
    properties
        log_filename;
        pulse_filename;
        gps_filename;
        pause_time = 0.1;
    end
    
    properties %(Access = private)
        gps_lines_read = 0;
        pulse_lines_read = 0;
        
        max_lines_read = 50;
        
        snr_threshold;
        rssi_threshold;
    end
    
    methods
        function obj = GPS_Pulse_Reader(log_filename, pulse_filename, gps_filename, varargin)
            p = inputParser;
            obj.log_filename = log_filename;
            obj.pulse_filename = pulse_filename;
            obj.gps_filename = gps_filename;
            
            addParameter(p, 'SNR_threshold', 15);
            addParameter(p, 'RSSI_threshold', -90);
            parse(p, varargin{:});
            
            obj.snr_threshold = p.Results.SNR_threshold;
            obj.rssi_threshold = p.Results.RSSI_threshold;
        end
           
        % get home gps location
        function home = get_home_pos(obj)
            GPS = [];
            while isempty(GPS)
                GPS = obj.get_gps();
                pause(obj.pause_time);
            end
            home = GPS(end, :);
        end         
        
        % get gps data
        function [gps] = get_gps(obj)
            % check new lines need to be read
            new_n_line = obj.lines_read(obj.gps_filename);
            n_to_read = new_n_line - obj.gps_lines_read;
            obj.gps_lines_read = new_n_line;
            
            [~,TCP_Data_Raw] = system(['tail -n ',num2str(min(n_to_read, obj.max_lines_read)),' ', obj.gps_filename]);
            if n_to_read ~= 0
                try
                TCP_Data = cellstr(splitlines(TCP_Data_Raw));
                TCP_Data =  TCP_Data(~cellfun('isempty',TCP_Data));

                data = cellfun(@(x) jsondecode(x), TCP_Data,'UniformOutput',false);

                gps = data(cellfun(@(x) isfield(x,'GLOBAL_POSITION_INT'),data));
                gps = struct2table(cellfun(@(x) x.GLOBAL_POSITION_INT, gps));
                gps.lat = gps.lat/1e7;
                gps.lon = gps.lon/1e7;
                gps.relative_alt = gps.relative_alt/1e3;
                gps.alt = gps.alt/1e3;
                gps.hdg = gps.hdg/1e2;
                gps.vx = gps.vx/1e2;
                gps.vy = gps.vy/1e2;
                gps.vz = gps.vz/1e2;   

                gps = gps(end, :);
                catch e
                    gps = [];
                end
            else
                gps = [];
            end

            % only report the latest GPS


        end 
        
        % get pulse from pulse log
        function [pulses] = get_pulses(obj)
            new_n_line = obj.lines_read(obj.pulse_filename);
            n_to_read = new_n_line - obj.pulse_lines_read;
            obj.pulse_lines_read = new_n_line;
            
            [~,TCP_Data_Raw] = system(['tail -n ',num2str(min(n_to_read, obj.max_lines_read)),' ', obj.pulse_filename]);     
            if n_to_read ~= 0
                TCP_Data = cellstr(splitlines(TCP_Data_Raw));
                TCP_Data =  TCP_Data(~cellfun('isempty',TCP_Data));
                data = cellfun(@(x) jsondecode(x), TCP_Data,'UniformOutput',false);
                pulses = data(cellfun(@(x) isfield(x,'P'),data));

                pulses = struct2table(cellfun(@(x) x.P, pulses));
            else
                pulses = [];
            end
                  
            % compare pulses
            pulses = obj.format_pulses(pulses);     
        end
    
    end
    
    
    methods (Access = private)
        
        % return the total number of lines in a file
        function n = lines_read(obj, filename)
            [~, str_l] = system(['wc -l <', filename]);
            n = str2double(str_l);
        end
        
        % only keep the latest pulses
        function [z] = filter_table(obj, z_table, ID_list)
            ID_index = boolean(zeros(length(ID_list), 1));
            for n = 1:length(ID_list)
                % only keep valid ID
                i = find(z_table.Target_ID == ID_list(n));
                
                if length(i) == 1 && z_table(i, :).SNR > obj.snr_threshold && z_table(i, :).RSSI > obj.rssi_threshold
                    ID_index(i) = true;
                else
%                     % only keep if uav location are similar
%                     last_location = z_table(i(end), :).UAV;
%                     for k = 1:length(i)
%                         if sqrt(sum((z_table(i(k), :).UAV(1:3) - last_location(1:3)).^2)) < 0.5 && ...
%                                   angdiff(z_table(i(k), :).UAV(4), last_location(4)) < deg2rad(1) && ...
%                                   z_table(i(k),:).SNR > 10   % SNR must be high enough
%                             ID_index(i(k)) = true;   
%                         end
%                     end
                    
                    % only keep the latest pulse & high enough snr
                    if z_table(i(end), :).SNR > obj.snr_threshold && z_table(i(end), :).RSSI > obj.rssi_threshold
                        ID_index(i(end)) = true;
                    end
                end
            end
            z = z_table(ID_index, :);
        end

        function z = format_pulses(obj, pulses)
            if ~isempty(pulses)
                %a: id,  b:freq,    c:signal strength
                %d: SNR, e:lat,     f:lon
                %g: alt, h:rel_alt, i:hdg 
                
                Target_ID = pulses.ID;
                Frequency = pulses.frequency;
                RSSI = pulses.RSSI/1e6;
                snr = pulses.SNR/1e2;
                relative_alt = pulses.relative_alt/1e2;
                
                ID_list = unique(Target_ID);

                % convert gps coordinate to cartesian 
                UAV = nan(height(pulses), 4);

                for l = 1:height(pulses)
                    UAV(l, 1:2) = [pulses.lat(l)/1e7; pulses.lon(l)/1e7];
                    UAV(l, 3) = relative_alt(l);
                    UAV(l, 4) = deg2rad(pulses.hdg(l)/1e2);   
                end
                
                z = table(Target_ID, Frequency, RSSI, snr, relative_alt, UAV);
                z.Properties.VariableNames = {'Target_ID', 'Frequency', 'RSSI', 'SNR', 'relative_alt', 'UAV'};

                % filter pulses
                z = obj.filter_table(z, ID_list);
            else
                z = [];
            end
        end     

    end
    
    
    
end
    