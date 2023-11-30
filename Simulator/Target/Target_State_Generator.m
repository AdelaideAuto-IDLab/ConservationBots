%% [Simulation Module]
% generate target truth


classdef Target_State_Generator < handle
   properties
       config;
       Target_Model;
   end
   
   methods
       function obj = Target_State_Generator(config, target_model)
           if nargin > 0
               obj.config = config;
               obj.Target_Model = target_model;
           end
       end    
       
       function state = gen_target_state(obj, T, DEM, rnd_seed)
           if nargin > 3
               % set random seed
               rng(rnd_seed);
           end
           
           % initialize output
           state = nan*zeros([obj.config.nx, obj.config.NTarget, T]); 
           DEM_size = size(DEM);
           
           for t = min(obj.config.Birth_Death(1,:)):obj.config.dt:min(T, max(obj.config.Birth_Death(2,:)))
               % check birth death
               birth = (t == obj.config.Birth_Death(1,:));
               survive = ((t > obj.config.Birth_Death(1,:)) & (t <= obj.config.Birth_Death(2,:)));
               death = t > obj.config.Birth_Death(2,:);

               % target birth
               state(:, birth, t) = obj.config.Init_State(:, birth);
               if t > 1
                   % target survive
                   state(:, survive, t) = obj.Target_Model.sys(state(:, survive, t-1), true);
                   % target death
                   state(:, death, t) = nan*zeros(obj.config.nx, sum(death));
               end


               % move target to the surface of DEM if provided
               if ~isempty(DEM)
                   exist = birth|survive;
                   exist_state = round(state(1:2, exist, t));
                   I = sub2ind(DEM_size, exist_state(1,:), exist_state(2,:));
                   state(3, exist, t) = DEM(I) + obj.config.Height(exist);
               end
           end
           
           % reset seed
           if nargin > 3
               rng('shuffle');
           end
       end   
       
       
       function state = gen_target_state_tmp(obj, i_state, T, DEM, rnd_seed)
           if nargin > 4
               % set random seed
               rng(rnd_seed);
           end
           
           % initialize output
           state = nan*zeros([obj.config.nx, obj.config.NTarget, T]); 
           DEM_size = size(DEM);
           
           for t = min(obj.config.Birth_Death(1,:)):obj.config.dt:min(T, max(obj.config.Birth_Death(2,:)))
               % check birth death
               birth = (t == obj.config.Birth_Death(1,:));
               survive = ((t > obj.config.Birth_Death(1,:)) & (t <= obj.config.Birth_Death(2,:)));
               death = t > obj.config.Birth_Death(2,:);

               % target birth
               state(:, birth, t) = i_state(:, birth);
               if t > 1
                   % target survive
                   state(:, survive, t) = obj.Target_Model.sys(state(:, survive, t-1), true);
                   % target death
                   state(:, death, t) = nan*zeros(obj.config.nx, sum(death));
               end


               % move target to the surface of DEM if provided
               if ~isempty(DEM)
                   exist = birth|survive;
                   exist_state = round(state(1:2, exist, t));
                   I = sub2ind(DEM_size, exist_state(1,:), exist_state(2,:));
                   state(3, exist, t) = DEM(I) + obj.config.Height(exist);
               end
           end
           
           % reset seed
           if nargin > 4
               rng('shuffle');
           end
       end   
       
       % load truth from file
       function state = load_truth(obj, filename)
           state = load(filename);
       end
  
       
   end
  
    
end

   
    
