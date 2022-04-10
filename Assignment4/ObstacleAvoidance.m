classdef ObstacleAvoidance
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        %% BURGER
%         robot_width = 178 * 10^-3; % meters Burger
%         dist_lidar_to_robot_front = 90 * 10^-3; 
%         look_ahead_dist = .2; 
%         buffer_dist = 130 * 10^-3;
%         cmd_ang_vel = .1;
%         cmd_lin_vel = .025;
        %% WAFFLE
        robot_width = 306 * 10^-3; % meters Waffle
        dist_lidar_to_robot_front = 100 * 10^-3; 
        look_ahead_dist = .5; 
        buffer_dist = 120 * 10^-3;
        cmd_ang_vel = .5;
        cmd_lin_vel = .2;
        %% Velocity Parameters

        back_forth_counter = 0;
        direction = '';
        previous_direction = 'straight';
    end

    methods
        function [obj_in_way, lin_vel, ang_vel] = calcObstAvoidVels(obj, scan_data)
            xy_scan = readCartesian(scan_data);
            [object_in_way_left, object_in_way_right] = is_there_object_in_way(obj, xy_scan);
            turn_dir = obj.calcDirToTurn(scan_data, object_in_way_left, object_in_way_right);
            
            if strcmp(turn_dir, 'left')
                ang_vel = obj.cmd_ang_vel;
                lin_vel = 0;
                obj_in_way = true;
            elseif strcmp(turn_dir, 'right')
                ang_vel = -obj.cmd_ang_vel;
                lin_vel = 0;
                obj_in_way = true;
            else 
                ang_vel = 0;
                lin_vel = obj.cmd_lin_vel;
                obj_in_way = false;
            end

        end

        function [object_in_way_left, object_in_way_right] = is_there_object_in_way(obj, xy_scan)
            object_in_way_left = false;
            object_in_way_right = false;
            
            for j=1:length(xy_scan)
                point = xy_scan(j, :);
                if abs((obj.robot_width + obj.buffer_dist)/2) > abs(point(2))
                    if (obj.look_ahead_dist + obj.dist_lidar_to_robot_front) > abs(point(1)) && (point(1) > obj.dist_lidar_to_robot_front)
                        if point(2) > 0
                            object_in_way_left = true;
                        else
                            object_in_way_right = true;
                        end
                    end 
                end 
            end
        end

        function resetBackForthCount(obj)
            obj.back_forth_counter = 0;
        end
        
        function dir_to_turn = calcDirToTurn(obj, scan_data, obj_to_left, obj_to_right)
            dir_to_turn = 'straight';

            % Calc direction to turn
            if ~obj_to_left && ~obj_to_right
                dir_to_turn = 'straight';   
                obj.resetBackForthCount();
            elseif obj_to_left && obj_to_right
                dir_to_turn = obj.calcBestDirToTurn(scan_data);
            elseif obj_to_left
                dir_to_turn = 'right';
            else
                dir_to_turn = 'left';
            end

            if ~strcmp(dir_to_turn, obj.previous_direction) && ~strcmp(dir_to_turn, 'straight') && ~strcmp('straight', obj.previous_direction)
                obj.back_forth_counter = obj.back_forth_counter + 1
            end

            if obj.back_forth_counter > 5
                dir_to_turn = 'right';
            end

            obj.previous_direction = dir_to_turn;

        end

        function direction = calcBestDirToTurn(obj, scan_data)
            num_ranges = length(scan_data.Ranges);
            num_points_per_quadrant = ceil(num_ranges / 4);
            
            left_points = scan_data.Ranges(1:num_points_per_quadrant);
            left_points_in_range = left_points( ~any( isnan( left_points ) | isinf( left_points ), 2 ),: );
            left_mean = mean(left_points_in_range);
            right_points = scan_data.Ranges(num_ranges - num_points_per_quadrant +1:num_ranges);
            right_points_in_range = right_points( ~any( isnan( right_points ) | isinf( right_points ), 2 ),: );
            right_mean = mean(right_points_in_range);
            if left_mean > right_mean
                direction = 'left';
            else
                direction = 'right';
            end            
        end
    end
end