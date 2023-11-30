% pulse reader test

comm = UAV_Comm('filter_config.json', 'Debug', true);

Total_time = 1000;

t = 0;
while (t < Total_time)
    loop_time = tic;
    t = t+1;
    
    current_gps = comm.get_gps();
    
    pulses = comm.get_pulses();
    disp(pulses);
    
    while toc(loop_time) < 1
        pause(0.05);
    end
    
end
