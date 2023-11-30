function r = distance_to_uav(x, uav)
%% calculate the Euclidean distance from target with state x  to uav

        if size(x,2) > size(uav,2)
           uav = repmat(uav,1,size(x,2));
        else
           x = repmat(x,1,size(uav,2));
        end
        
        pos_relative = (x(1:3,:)-uav(1:3,:));
        [~,~,r] = cart2sph(pos_relative(1,:), pos_relative(2,:), pos_relative(3,:));

end