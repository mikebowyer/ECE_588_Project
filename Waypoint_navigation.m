clear all;
ip_TurtleBot = '127.0.0.1';    
ip_Matlab = '127.0.0.1 ';  

rosshutdown;
%ip_TurtleBot = '10.0.1.57';    
%ip_Matlab = '10.0.1.54'; 

setenv('ROS_MASTER_URI', strcat('http://', ip_TurtleBot,':11311'))
setenv('ROS_IP', ip_Matlab)

rosinit(ip_TurtleBot)

%% Setup
% Odometry Subscription
TurtleBot_Topic.odom = '/odom';
odom_sub = rossubscriber('/odom');
[cmd_vel_pub,twist_msg] = rospublisher('/cmd_vel','geometry_msgs/Twist');


%% Target Pose Information

%% Run Main Robot Control Loop
close all
figure
initial_odom_data = receive(odom_sub);
initial_x_position = initial_odom_data.Pose.Pose.Position.X;
initial_y_position = initial_odom_data.Pose.Pose.Position.Y;
[targ_pose_pub,targ_pose] = rospublisher("/pose","geometry_msgs/Pose","DataFormat","struct");
targ_pose.Position.X = 5;
targ_pose.Position.Y = 3;
relative_x = [];
relative_y = [];

ang_pid = AngularPIDController();
lin_pid = LinearPIDController();
tic
 while true
    % Get and plot recent odometry
    odom_data = receive(odom_sub);
    current_pose = odom_data.Pose.Pose;
    relative_x = [relative_x, initial_x_position - current_pose.Position.X];
    relative_y = [relative_y, initial_y_position - current_pose.Position.Y];
    scatter(relative_x,relative_y,[],'green','filled')

    [dist_to_targ, ang_to_targ] = CalcDeltaPoseToTarget(current_pose, targ_pose);
    ang_to_targ  ;
    curr_time = toc;
    ang_vel = ang_pid.CalcAngVel(curr_time, ang_to_targ);
    lin_vel = lin_pid.CalcLinVel(curr_time, dist_to_targ)
    lin_vel
    
    actuate(cmd_vel_pub, twist_msg, 0.06, ang_vel)
    % Saving things for next iteration
    tic;
    
end     
%% Calculate Delta between Current Pose and Target Pose
function [delta_dist, delta_theta] = CalcDeltaPoseToTarget(curr_pose, targ_pose)
    d_x = targ_pose.Position.X - curr_pose.Position.X;
    d_y = targ_pose.Position.Y - curr_pose.Position.Y;

    % Get current robot angle
    cur_angle_quat = quaternion([curr_pose.Orientation.X curr_pose.Orientation.Y curr_pose.Orientation.Z curr_pose.Orientation.W ]);
    cur_angle_mat = quat2rotm(cur_angle_quat);
    cur_angle = wrapToPi(atan2(cur_angle_mat(2,3), cur_angle_mat(3,3)))

    % Get angle from robot directly to the target
    targ_ang = atan2(d_y,d_x)
    delta_theta = targ_ang-cur_angle

    delta_dist = sqrt(d_x^2 + d_y^2);

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

