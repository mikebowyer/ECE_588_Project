clear all;
% ip_TurtleBot = '127.0.0.1';    
% ip_Matlab = '127.0.0.1 ';  

rosshutdown;
ip_TurtleBot = '10.0.1.57';    
ip_Matlab = '10.0.1.54'; 

setenv('ROS_MASTER_URI', strcat('http://', ip_TurtleBot,':11311'))
setenv('ROS_IP', ip_Matlab)

rosinit(ip_TurtleBot)
%% Setup Topic Subscriptions and Publications
% Subscriptions
%% Camera
%TurtleBot_Topic.picam = "/camera/rgb/image_raw/compressed";
%TurtleBot_Topic.picam = "/raspicam_node/image/compressed";
%image_sub = rossubscriber(TurtleBot_Topic.picam);
%% Lidar
TurtleBot_Topic.laser = '/scan';
laser_sub = rossubscriber('/scan'); 
%% 
TurtleBot_Topic.odom = '/odom';
odom_sub = rossubscriber('/odom');
%% Publications
[cmd_vel_pub,twist_msg] = rospublisher('/cmd_vel','geometry_msgs/Twist');

%% Run Main Robot Control Loop
close all
%figure('units','normalized','outerposition',[0 0 1 1]), hold on
figure
%% BURGER
robot_width = 178 * 10^-3; % meters Burger
dist_lidar_to_robot_front = 90 * 10^-3; 
look_ahead_dist = .2; 
buffer_dist = 130 * 10^-3;
angle_max = .1;
speed_max = .025;
%% WAFFLE
% robot_width = 306 * 10^-3; % meters Waffle
% dist_lidar_to_robot_front = 100 * 10^-3; 
% look_ahead_dist = .5; 
% buffer_dist = 120 * 10^-3;
% angle_max = .5;
% speed_max = .2;
%%
odom_data = receive(odom_sub);
initial_x_position = odom_data.Pose.Pose.Position.X;
initial_y_position = odom_data.Pose.Pose.Position.Y;
relative_x = [];
relative_y = [];
backAndForthCounter = 0;
while true 
    odom_data = receive(odom_sub);
    relative_x = [relative_x, initial_x_position - odom_data.Pose.Pose.Position.X];
    relative_y = [relative_y, initial_y_position - odom_data.Pose.Pose.Position.Y];
    odom_data.Pose.Pose.Position.Y;
    
    % Read and show lidar data
    scan_data = receive(laser_sub);
    data = readCartesian(scan_data);
    
    % Plot robot buffer and scan data
    plot_rect(robot_width,look_ahead_dist, buffer_dist, dist_lidar_to_robot_front); %hold on
    scatter(data(:,1), data(:,2))
    xlim([-1 4]); ylim([-2,2]); grid on; %set(gca,'XAxisLocation','bottom','YAxisLocation','left','ydir','reverse')
    %scatter(relative_x,relative_y,[],'green','filled')
    hold off

    % Detect if there are any points in the way of the robot
    [obj_to_left, obj_to_right] = is_there_object_in_way(data, robot_width, look_ahead_dist, buffer_dist, dist_lidar_to_robot_front);
    direction = '';
    directionForTitle = '';
    previousDirection = direction;
    if ~obj_to_left && ~obj_to_right
        direction = 'straight';    
        backAndForthCounter = 0;
        directionForTitle = 'straight';
    elseif obj_to_left && obj_to_right
        direction = calcBestDirToTurn(scan_data);
        directionForTitle = ['not clear - turning ' direction];        
    elseif obj_to_left
        direction = 'right';
        directionForTitle = 'clear - turning right';
    else
        direction = 'left';
        directionForTitle = 'clear - turning left';     
    end
    if ~strcmp(direction, previousDirection) && ~strcmp(direction, 'straight') && ~strcmp('straight', previousDirection)
        backAndForthCounter = backAndForthCounter + 1;
    elseif strcmp(direction, previousDirection)
        %if does two in a row of the same direction, is making progress
        backAndForthCounter = 0;
    end
    previousDirection = direction;
    if backAndForthCounter > 10
        direction = 'right';
        directionForTitle = 'Avoidance';
    end
    title(["Direction: " directionForTitle]);
    xlabel('Distance in front of robot (meters)');
    ylabel('Lateral distance relative to robot (meters)');
    legend('Robot size', 'Look ahead outline', 'lidar points')
    twist_msg = calcCmdVelMsg(direction, twist_msg, angle_max, speed_max);
    % Send Control Command
    send(cmd_vel_pub,twist_msg);
end
     
%% FUNCTIONS
function direction = calcBestDirToTurn(scan_data)
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
function plot_rect(robot_width, look_ahead_dist, buffer_dist, dist_lidar_to_robot_front)
    % Plot Robot Size
    y1=-(robot_width)/2;
    y2=(robot_width)/2;
    x1=-dist_lidar_to_robot_front;
    x2=dist_lidar_to_robot_front;
    x = [x1, x2, x2, x1, x1];
    y = [y1, y1, y2, y2, y1];
    plot(x, y, 'r', 'LineWidth', 3);
    hold on

    % Plot Look Ahead Rectangle
    y1=-(robot_width + buffer_dist)/2;
    y2=(robot_width + buffer_dist)/2;
    x1=dist_lidar_to_robot_front;
    x2=dist_lidar_to_robot_front + look_ahead_dist;
    x = [x1, x2, x2, x1, x1];
    y = [y1, y1, y2, y2, y1];
    plot(x, y, 'b-', 'LineWidth', 3);
    
end

function [object_in_way_left, object_in_way_right] = is_there_object_in_way(xy_scan, robot_width, look_ahead_dist, buffer_dist, dist_lidar_to_robot_front)
    object_in_way_left = false;
    object_in_way_right = false;
    
    for j=1:length(xy_scan)
        point = xy_scan(j, :);
        if abs((robot_width + buffer_dist)/2) > abs(point(2))
            if (look_ahead_dist + dist_lidar_to_robot_front) > abs(point(1)) && (point(1) > dist_lidar_to_robot_front)
                if point(2) > 0
                    object_in_way_left = true;
                else
                    object_in_way_right = true;
                end
            end 
        end 
    end
end

function twist_out = calcCmdVelMsg(direction, twist_in, angle_max, speed_max)

    twist_out = twist_in;
    if strcmp(direction, 'left')
        twist_out.Angular.Z = angle_max;
        twist_out.Linear.X = 0;
    elseif strcmp(direction, 'right')
        twist_out.Angular.Z = -angle_max;
        twist_out.Linear.X = 0;
    else
        twist_out.Angular.Z = 0;
        twist_out.Linear.X = speed_max;
    end
end
