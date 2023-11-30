function pr = Void_Probability(density, waypoints, r)
    % [Usage]:
    % pr = Void_Probability(density, waypoints, r)
    %
    % [Description]:
    % Calculating void probability
    %
    % [Input]:
    % density           particle represented pdf
    % waypoints         array of column vector represented waypoints
    % r                 void radius
    %
    % [Output]
    % pr                void probability
    %
    
    
    ntarget = length(density);
    nwaypoint = size(waypoints, 2);
    
    void_pr = zeros(ntarget, nwaypoint);
    
    for n = 1:ntarget
        for i = 1:nwaypoint
            d = sqrt(sum((density{n}.particles(1:2, :) - waypoints(1:2, i)).^2));
            void_pr(n, i) = mean(d>r);
        end
    end
    
    pr = min(void_pr, [], 'all');
end