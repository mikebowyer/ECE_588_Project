clear all;
rosshutdown;
ip_TurtleBot = '127.0.0.1';    
ip_Matlab = '127.0.0.1 ';  
%ip_TurtleBot = '10.0.1.57';    
%ip_Matlab = '10.0.1.54'; 
setenv('ROS_MASTER_URI', strcat('http://', ip_TurtleBot,':11311'))
setenv('ROS_IP', ip_Matlab)
rosinit(ip_TurtleBot)
%% ROS SUbscription Setup
% Odometry Subscription
TurtleBot_Topic.odom = '/odom';
odom_sub = rossubscriber('/odom');
[cmd_vel_pub,twist_msg] = rospublisher('/cmd_vel','geometry_msgs/Twist');

% Lidar
TurtleBot_Topic.laser = '/scan';
laser_sub = rossubscriber('/scan'); 
%% Target Parameters
stop_dist_thresh  = .5;
[targ_pose_pub,targ_pose] = rospublisher("/pose","geometry_msgs/Pose","DataFormat","struct");
targ_pose.Position.X = 3;
targ_pose.Position.Y = 1.75;
%% Run Main Robot Control Loop
close all

% Position Setup
initial_odom_data = receive(odom_sub);
target_nav = TargetNavigator(initial_odom_data);

% Setup Obstacle Avoidance Class
obst_avoid = ObstacleAvoidance();

% Setup PID Controllers and stop robot to start with.
ang_pid = AngularPIDController();
lin_pid = LinearPIDController();

actuate(cmd_vel_pub, twist_msg, 0, 0); pause(1);

% Start while loop
tic
 while true
    % Find target

    % Get Odom and Plot Target
    odom_data = receive(odom_sub);
    target_nav.PlotTarget(odom_data, targ_pose)
    
    % Read and show lidar data
    scan_data = receive(laser_sub);

    % Determine Object is in the way
    [obj_in_way, lin_vel, ang_vel] = obst_avoid.calcObstAvoidVels(scan_data);

    if ~obj_in_way
        [dist_to_targ, ang_to_targ] = target_nav.CalcDeltaPoseToTarget(odom_data, targ_pose);
        % Run Control Algos
        curr_time = toc;
        if dist_to_targ < stop_dist_thresh
            actuate(cmd_vel_pub, twist_msg, 0, 0)
            %title("Arrived at target, stopping!")
            %return
        else
            ang_vel = ang_pid.CalcAngVel(curr_time, ang_to_targ);
            lin_vel = lin_pid.CalcLinVel(curr_time, dist_to_targ);
            %title(["DistToTarg: " dist_to_targ, "AngToTarg: " ang_to_targ, "LinVel: " lin_vel, "AngVel:" ang_vel])
        end
    end    

    actuate(cmd_vel_pub, twist_msg, lin_vel, ang_vel)
    tic;
end     

function actuate(cmd_vel_pub, twist_in, linear_vel, ang_vel)

    twist_out = twist_in;

    % assign linear velocity
    twist_out.Linear.X = linear_vel;
    twist_out.Linear.Y = 0;
    twist_out.Linear.Z = 0;
    % assign angular velocity
    twist_out.Angular.X = 0;
    twist_out.Angular.Y = 0;
    twist_out.Angular.Z = ang_vel;

    % send this velocity command to robot
    send(cmd_vel_pub,twist_out);
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
