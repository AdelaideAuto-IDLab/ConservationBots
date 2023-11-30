classdef UAV_shape < handle
    properties 
        h_axes = [];
        h_fan = gobjects(4);
        h_arm = gobjects(2);
        h_head = [];
        
        UAV_size = 1;
        UAV_color = 'k';
        UAV_linewidth = 3;
        
        UAV_pos = [0,0,0,0]'; %[x, y, z, heading]'
        
    end
    
    properties (Access = private)
        circle_x = 0.75*cos((0:pi/50:2*pi));
        circle_y = 0.75*sin((0:pi/50:2*pi));
        
        arm_x = (-1:0.1:1);
        arm_y1 = (1:-0.1:-1);
        arm_y2 = (-1:0.1:1);
        
        head_x = [0,0];
        head_y = [0,3];
    end
    
    
    methods
        function obj = UAV_shape(ax, color, size)
            obj.h_axes = ax;
            
            obj.UAV_size = size;
            obj.UAV_color = color;
            
            obj.circle_x = size * obj.circle_x;
            obj.circle_y = size * obj.circle_y;
            obj.arm_x = size * obj.arm_x;
            obj.arm_y1 = size * obj.arm_y1;
            obj.arm_y2 = size * obj.arm_y2;
            
            obj.head_x = size * obj.head_x;
            obj.head_y = size * obj.head_y;
            
            obj.h_fan(1) = plot(obj.circle_x - size, obj.circle_y + size, 'Color', color, 'LineWidth', obj.UAV_linewidth, 'Parent', obj.h_axes);
            obj.h_fan(2) = plot(obj.circle_x + size, obj.circle_y + size, 'Color', color, 'LineWidth', obj.UAV_linewidth, 'Parent', obj.h_axes);
            obj.h_fan(3) = plot(obj.circle_x + size, obj.circle_y - size, 'Color', color, 'LineWidth', obj.UAV_linewidth, 'Parent', obj.h_axes);
            obj.h_fan(4) = plot(obj.circle_x - size, obj.circle_y - size, 'Color', color, 'LineWidth', obj.UAV_linewidth, 'Parent', obj.h_axes);
            obj.h_arm(1) = plot(obj.arm_x, obj.arm_y1, 'LineStyle', '-', 'Color', color, 'LineWidth', obj.UAV_linewidth, 'Parent', obj.h_axes);
            obj.h_arm(2) = plot(obj.arm_x, obj.arm_y2, 'LineStyle', '-', 'Color', color, 'LineWidth', obj.UAV_linewidth, 'Parent', obj.h_axes);
            obj.h_head = plot(obj.head_x, obj.head_y, 'LineStyle', '-.', 'Color', color, 'LineWidth', obj.UAV_linewidth, 'Parent', obj.h_axes);
        end
        
        function update_pos(obj, pos)
            % update UAV position
            obj.UAV_pos = pos;
            pos(4) = pos(4) + pi/2;
            set(obj.h_fan(1), 'XData', obj.circle_x - obj.UAV_size*sin(pos(4)) - obj.UAV_size*cos(pos(4)) + pos(1), ...
                              'YData', obj.circle_y - obj.UAV_size*cos(pos(4)) + obj.UAV_size*sin(pos(4)) + pos(2));
            set(obj.h_fan(2), 'XData', obj.circle_x + obj.UAV_size*sin(pos(4)) - obj.UAV_size*cos(pos(4)) + pos(1), ...
                              'YData', obj.circle_y + obj.UAV_size*cos(pos(4)) + obj.UAV_size*sin(pos(4)) + pos(2));
            set(obj.h_fan(3), 'XData', obj.circle_x + obj.UAV_size*sin(pos(4)) + obj.UAV_size*cos(pos(4)) + pos(1), ...
                              'YData', obj.circle_y + obj.UAV_size*cos(pos(4)) - obj.UAV_size*sin(pos(4)) + pos(2));
            set(obj.h_fan(4), 'XData', obj.circle_x - obj.UAV_size*sin(pos(4)) + obj.UAV_size*cos(pos(4)) + pos(1), ...
                              'YData', obj.circle_y - obj.UAV_size*cos(pos(4)) - obj.UAV_size*sin(pos(4)) + pos(2));
                          
            set(obj.h_arm(1), 'XData', sin(pos(4))*obj.arm_x - cos(pos(4))*obj.arm_y1 + pos(1),...
                              'YData', cos(pos(4))*obj.arm_x + sin(pos(4))*obj.arm_y1 + pos(2));
            set(obj.h_arm(2), 'XData', sin(pos(4))*obj.arm_x - cos(pos(4))*obj.arm_y2 + pos(1),...
                              'YData', cos(pos(4))*obj.arm_x + sin(pos(4))*obj.arm_y2 + pos(2));

            set(obj.h_head, 'XData', obj.head_x*sin(pos(4)) - obj.head_y*cos(pos(4)) + pos(1),...
                              'YData', obj.head_x*cos(pos(4)) + obj.head_y*sin(pos(4)) + pos(2));         
        end
        
    end
    
end