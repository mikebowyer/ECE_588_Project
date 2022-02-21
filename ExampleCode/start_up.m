%% Connect to Turtlebot
% Connect to an External ROS Master
%%% ip address of TurtleBot and Matlab, replace these values accordingly
ip_TurtleBot = '23.28.1.142';    
ip_Matlab = '192.168.1.100';      

setenv('ROS_MASTER_URI', strcat('http://', ip_TurtleBot,':11311'))
setenv('ROS_IP', ip_Matlab)

rosinit(ip_TurtleBot)

%%% check nodes and topics
% rosnode list
% rostopic list
% rostopic info topic name;

%%% topics we are interested
TurtleBot_Topic.vel = '/cmd_vel';
TurtleBot_Topic.laser = '/scan';
TurtleBot_Topic.picam = '/raspicam_node/image/compressed';
TurtleBot_Topic.odom = '/odom';
TurtleBot_Topic.imu = '/imu';

%%
addpath('helper_functions');
%% Disconnect from the Robot

%%% clear workspace when you are finished with them
% clear

%%% Shut down the global node and disconnect from the TurtleBot.
% rosshutdown
