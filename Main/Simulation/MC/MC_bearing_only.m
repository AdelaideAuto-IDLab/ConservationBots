function MC_bearing_only(MC_runs, Total_time, ntarget, reward, Use_Void, rotation_time, truth_file)

    %% Global configuration
    %MC_runs                         % number of Monte-Carlo runs
    %rnd_seed                       
    %ntarget                         % total number of target
    %Total_time                      % maximun simulation time
    %truth_file                      % truth data


    mode = Planner_Type.AoA;


    MC_results = cell(MC_runs,1);   


    parfor mc = 1:MC_runs
        uav0 = [993, 504, 80, pi/4]';        % uav initial location 
        %% Useful intermedia variable
        uav = nan(4, Total_time);
        selected_action = [];               % planner output
        selected_target = nan;              % planner selected target
        action_index = 0;                   % the nth action
        all_target_found = false;           % true if all targets are found 
        %% Initialization

        % generate module settings
        config = simulation_config_wrapper(ntarget, reward, Use_Void, rotation_time);

        % initialize each modules
        % =========== [System modules] ===============
        target_model = Target_Model(config.target_config);
        AoA_model = AoA_Model(config.AoA_sensor_config);
        UAV_Action = UAV_Actions(config.uav_config, config.area_config);

        Terminate = Termination_Condition(config.Term_config);
        AoA_Filter = Bernoulli_Filter(config.AoA_filter_config, AoA_model, target_model);
        Planner = Planner_greedy_POMDP(config.planner_config, AoA_model, target_model, UAV_Action);

        AoA_Generator = Rotation_Bearing(config.AoA_config);
        % =========== [Simulation modules] ============
        target_truth_generator = Target_State_Generator(config.sim_target_config, target_model);
        RSSI_generator = RSSI_Generator(config.sim_sensor_config, config.sim_target_radio_config, config.area_config.DEM.DATA);

        % =========== [Data recording] ====================
        recorder = Statistics(ntarget, (1:ntarget)', Total_time, config, 'Simulation', false);


        %% Simulation preparation 
        % initialize uav height
        uav0(3) = uav0(3)+config.area_config.DEM.DATA(round(uav0(1)), round(uav0(2)));
        
        % load truth
        truth = target_truth_generator.load_truth(truth_file);
        truth = truth.truth(:, 1:ntarget, :);
        % save truth 
        recorder.record_truth(truth);

        % initialize filter density
        density = cell(config.target_config.Ntarget, 1);
        for i = 1:config.target_config.Ntarget
            density{i} = Bernoulli(gen_birth_particles_uniform(config.area_config.DEM.DATA, 20000), 0.01, i);
        end

        % generate initial action
        selected_action.waypoints = UAV_Action.get_action_md(uav0, pi/4, config.uav_config.vmax);
        selected_action.type = Action_Type.RSSI;



        %% ======================== Main Loop ================================
        t = 0;
        while t <= Total_time && ~all_target_found
            t = t+1;    % increment simulation time
            n_cycle = ceil(t/config.planner_config.T);  % current planning cycle

            % ============= update uav location ========================
            if n_cycle == 1
                uav(:, t) = selected_action.waypoints(:, t);
            else
                uav(:, t) = selected_action.waypoints(:, t- (n_cycle-1)*config.planner_config.T);
            end

            % ============= generate measurement =======================
            z = RSSI_generator.get_RSSI(squeeze(truth(:, 1:ntarget, t)), uav(:, t)); 
            % record data for bearing
            AoA_Generator.record_data(z, t);

            % ===================== filtering ==========================
            % filter prediction
            for n = 1:length(density)
                density{n} = AoA_Filter.predict(density{n});
            end
            
            if mod(t, config.planner_config.T) == 0
                % get bearing measurement
                bearing = AoA_Generator.get_bearing();
                for n = 1:length(density)
                    % extract ID & RSSI from measurement table z
                    density{n} = AoA_Filter.filtering(density{n}, table2array(bearing(n, [1,3])), uav(:, t));
                end
            end

            % ====================== planning ==========================
            if mod(t, config.planner_config.T) == 0
                action_index = action_index + 1;
                [selected_action, selected_target] = Planner.plan(uav(:, t), density);

                % init bearing action
                AoA_Generator.check_start_condition(selected_action, t+config.uav_config.traject_time - config.uav_config.rotation_time);

                % save selected action
                recorder.record_action(uav(:,t), selected_action, action_index, t);
            end

            % =============== check termination codition ================
            [density, all_target_found, found_report] = Terminate.termination_check(density, t);

            % ====================== Collecting data ====================
            [est] = recorder.record(z, density, found_report, t);


        end

        % Save UAV Path
        recorder.record_uav_path(uav(:, 1:t));
        
        % terminate unlocalized target
        recorder.record_unlocalized(t);
        % calculate error
        recorder.get_error();
        
        recorder.report_result();

        %% ====================================================================
        % save result 

        MC_results{mc} = recorder.export();

    end


    %% Analyse MC results
    % time, RMSE, travel distance
    analyzer = MC_Analysis(MC_results, mode);
    analyzer.gen_report();

end
