function MC_RSSI(MC_runs, Total_time, ntarget, reward, Use_Void, rotation_time, truth_file)
    %% Global configuration
    %MC_runs                         % number of Monte-Carlo runs
    %rnd_seed                       
    %ntarget                         % total number of target
    %Total_time                      % maximun simulation time
    %truth_file                      % truth data

    mode = Planner_Type.RSSI;


    MC_results = cell(MC_runs,1);       


    parfor mc = 1:MC_runs
        uav0 = [993, 504, 50, pi/4]';        % uav initial location 
        %% Useful intermedia variable
        uav = nan(4, Total_time);
        selected_action = [];               % planner output
        action_index = 0;                   % the nth action
        all_target_found = false;           % true if all targets are found 
        %% Initialization

        % generate module settings
        config = simulation_config_wrapper(ntarget, reward, Use_Void, rotation_time);

        % initialize each modules
        % =========== [System modules] ===============
        target_model = Target_Model(config.target_config);
        RSSI_model = RSSI_Model(config.RSSI_sensor_config);
        UAV_Action = UAV_Actions(config.uav_config, config.area_config);

        Terminate = Termination_Condition(config.Term_config);
        RSSI_Filter = Bernoulli_Filter(config.RSSI_filter_config, RSSI_model, target_model);
        Planner = Planner_greedy_POMDP(config.planner_config, RSSI_model, target_model, UAV_Action);

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
            density{i} = Bernoulli(gen_birth_particles_polygon(config.area_config.DEM.DATA, 10000, config.area_config.area, config.area_config.Alt_range), 0.01, config.target_id_list(i));
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


            % ===================== filtering ==========================
            for n = 1:length(density)
                % extract ID & RSSI from measurement table z
                if ~isempty(z)
                    idx = find(z.Target_ID == config.target_id_list(n));
                    if ~isempty(idx)
                        density{n} = RSSI_Filter.filtering(density{n}, [config.target_id_list(n), z.RSSI(idx)], uav(:,t));
                    else
                        % predicting
                    end
                end
            end

            % ====================== planning ==========================
            if mod(t, config.planner_config.T) == 0
                action_index = action_index + 1;
                [selected_action] = Planner.plan(uav(:, t), density);
                % save selected action
                recorder.record_action(uav(:,t), selected_action, action_index, t);
            end

            % =============== check termination codition ================
            [density, all_target_found, found_report] = Terminate.termination_check(density, t);

            % ====================== Collecting data ====================
            recorder.record(z, density, found_report, t);


        end

        % Save UAV Path
        recorder.record_uav_path(uav(:, 1:t));
        
        % terminate unlocalized target
        recorder.record_unlocalized(t);
        % calculate error
        recorder.get_error();

        %% ====================================================================
        % save result 

        MC_results{mc} = recorder.export();

    end


    %% Analyse MC results
    % time, RMSE, travel distance
    analyzer = MC_Analysis(MC_results, mode);
    analyzer.gen_report();
end

