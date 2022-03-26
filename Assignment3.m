clear all;
%ip_TurtleBot = '127.0.0.1';    
%ip_Matlab = '127.0.0.1 ';  

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
robot_width = 178 * 10^-3; % meters Burger
%robot_width = 306 * 10^-3; % meters Waffle
look_ahead_dist = .3; 
buffer_dist = 120 * 10^-3;
odom_data = receive(odom_sub);
initial_x_position = odom_data.Pose.Pose.Position.X;
initial_y_position = odom_data.Pose.Pose.Position.Y;
relative_x = [];
relative_y = [];
while true 
    odom_data = receive(odom_sub);
    relative_x = [relative_x, initial_x_position - odom_data.Pose.Pose.Position.X];
    relative_y = [relative_y, initial_y_position - odom_data.Pose.Pose.Position.Y];
    odom_data.Pose.Pose.Position.Y
    % Read and show lidar data
    scan_data = receive(laser_sub);
    data = readCartesian(scan_data);
    
    % Plot robot buffer and scan data
    plot_rect(robot_width,look_ahead_dist, buffer_dist); %hold on
    scatter(data(:,1), data(:,2))
    xlim([-1 4]); ylim([-2,2]); grid on; set(gca,'XAxisLocation','bottom','YAxisLocation','left','ydir','reverse')
    scatter(relative_x,relative_y,[],'green','filled')
    hold off
    % Detect if there are any points in the way of the robot
    if is_there_object_in_way(data, robot_width, look_ahead_dist, buffer_dist)
        title("OBJECT IN THE WAY!!!!!!!!!!")
        twist_msg = calcCmdVelMsg('right', twist_msg);
        % turn right
    else
        title("nah")
        % turn go straight
        twist_msg = calcCmdVelMsg('straight', twist_msg);

    end
%     pause(1);
%     clf
    send(cmd_vel_pub,twist_msg);
end
     
%% FUNCTIONS

function plot_rect(robot_width, look_ahead_dist, buffer_dist)
    y1=-(robot_width + buffer_dist)/2;
    y2=(robot_width + buffer_dist)/2;
    x1=0;
    x2=look_ahead_dist;
    x = [x1, x2, x2, x1, x1];
    y = [y1, y1, y2, y2, y1];
    plot(x, y, 'b-', 'LineWidth', 3);
    hold on
end

function object_in_way = is_there_object_in_way(xy_scan, robot_width, look_ahead_dist, buffer_dist)
    %[num_points, width]= size(xy_scan);
    %num_points_per_quad = ceil(num_points / 4);
    object_in_way= false;
    for j=1:length(xy_scan)
        point = xy_scan(j, :);
        if abs((robot_width + buffer_dist)/2) > abs(point(2))
            if look_ahead_dist > abs(point(1)) && point(1) > 0
                object_in_way= true;
                return
            end 
        end 
    end

end

function twist_out = calcCmdVelMsg(direction, twist_in)

    twist_out = twist_in;
    if strcmp(direction, 'left')
        twist_out.Angular.Z = .1;
        twist_out.Linear.X = 0;
    elseif strcmp(direction, 'right')
        twist_out.Angular.Z = -.1;
        twist_out.Linear.X = 0;
    else
        twist_out.Angular.Z = 0;
        twist_out.Linear.X = 0.025;
    end
end
