ip_TurtleBot = '127.0.0.1';    
ip_Matlab = '127.0.0.1 ';  
clear all;
rosshutdown;
%ip_TurtleBot = '10.0.1.57';    
%ip_Matlab = '10.0.1.54'; 

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
%% Publications
[cmd_vel_pub,twist_msg] = rospublisher('/cmd_vel','geometry_msgs/Twist');

%% Run Main Robot Control Loop
close all
%figure('units','normalized','outerposition',[0 0 1 1]), hold on
figure
lastDirection = 'unknown';
while true 
    
    % Read and show lidar data
    scan_data = receive(laser_sub);
    plot(scan_data);

    %data = readCartesian(scan_data); 
    
    
    % Control the boto
    %twist_msg = calcCmdVelMsg(intercept, theta, twist_msg, img_width);
    %PlotInterceptTheta(extent_points, intercept,theta, image, twist_msg.Angular.Z)
    %send(cmd_vel_pub,twist_msg);
end
     
%% FUNCTIONS

function twist_out = calcCmdVelMsg(intercept_pixel, theta, twist_in, img_width)
    twist_out = twist_in;
    

    ratio_intercept_from_img_center = (intercept_pixel - (img_width/2)) / (img_width/2);
    
    twist_out.Linear.X = .01;
    
    max_turn_z_val = .1;
    intercept_part = -.5*((max_turn_z_val) * ratio_intercept_from_img_center);
    theta_part = -.5*max_turn_z_val * (-theta/90);
    twist_out.Angular.Z = theta_part + intercept_part;

    if abs(twist_out.Angular.Z) > 0.2
        twist_out.Angular.Z = 0.2 * sign(twist_out.Angular.Z)
    end
    
    %twist_out.Angular.Z = max_turn_z_val * (-theta/90) * (-(1 - ratio_intercept_from_img_center)*1.5);
 
end