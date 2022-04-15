clear all;
rosshutdown;
%ip_TurtleBot = '127.0.0.1';    
%ip_Matlab = '127.0.0.1 ';  
ip_TurtleBot = '10.0.1.2';    
ip_Matlab = '10.0.1.54'; 
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

% Camera
%TurtleBot_Topic.picam = "/camera/rgb/image_raw/compressed";
TurtleBot_Topic.picam = "/raspicam_node/image/compressed";
image_sub = rossubscriber(TurtleBot_Topic.picam);
%% Target Parameters
[targ_pose_pub,targ_pose] = rospublisher("/pose","geometry_msgs/Pose","DataFormat","struct");
target_found = false;
targ_pose.Position.X = 0;
targ_pose.Position.Y = 0;
%% Run Main Robot Control Loop
close all

% Position Setup
initial_odom_data = receive(odom_sub);
target_nav = TargetNavigator(initial_odom_data);

% Setup Obstacle Avoidance Class
obst_avoid = ObstacleAvoidance();

% Setup Line Following Class
line_follower = LineFollower();

% Setup Target Findger
target_finder = TargetPositionFinder();

% Stop the Robot
lin_vel=0;
ang_vel=0;
actuate(cmd_vel_pub, twist_msg, lin_vel, ang_vel); pause(1);

%set up figures


tic
while true
    % Get Odom and Plot Target
    odom_data = receive(odom_sub);
    target_nav.PlotTarget(odom_data, targ_pose, target_found)
    
    % Detect and avoid objects
    scan_data = receive(laser_sub);
    [obj_in_way, lin_vel, ang_vel] = obst_avoid.calcObstAvoidVels(scan_data);

    % Find target
    image_compressed = receive(image_sub);
    [ellipseX, ellipseY] =target_finder.FindTargetPixelCoords(image_compressed);
    if(~target_found)        
        if ~isnan(ellipseX)
            [targ_pose.Position.X, targ_pose.Position.Y, target_found] = target_finder.CalcTargetPosition(ellipseX,scan_data,odom_data);
        end
    end

    if ~obj_in_way

        if target_found
            curr_time = toc;
            [lin_vel , ang_vel, dist_to_targ] =target_nav.CalcWayPointNavVels(odom_data, targ_pose, curr_time);
            if dist_to_targ < 0.35
                obst_avoid.buffer_dist = 0.05;               
            end
            obst_avoid.look_ahead_dist = min(max(dist_to_targ - 0.25, 0.02),0.2);
        else
            % Follow Lines
            [lin_vel , ang_vel] = line_follower.CalcLineFollowVels(image_compressed);
        end
    end    

    actuate(cmd_vel_pub, twist_msg, lin_vel, ang_vel)
    tic;
end     

%% Actuation Function
function actuate(cmd_vel_pub, twist_in, linear_vel, ang_vel)
    
    twist_out = twist_in;

    % assign linear velocity
    twist_out.Linear.X = min(max(linear_vel,0),0.05);
    twist_out.Linear.Y = 0;
    twist_out.Linear.Z = 0;
    % assign angular velocity
    twist_out.Angular.X = 0;
    twist_out.Angular.Y = 0;
    twist_out.Angular.Z = min(max(ang_vel,-0.1),0.1);

    % send this velocity command to robot
    send(cmd_vel_pub,twist_out);
end