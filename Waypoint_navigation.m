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
initial_odom_data = receive(odom_sub);
initial_x_position = initial_odom_data.Pose.Pose.Position.X;
initial_y_position = initial_odom_data.Pose.Pose.Position.Y;
relative_x = [];
relative_y = [];

%% Target Pose Information
[targ_pose_pub,targ_pose] = rospublisher("/pose","geometry_msgs/Pose","DataFormat","struct");
targ_pose.Position.X = 10;
targ_pose.Position.Y = 10;
%% Run Main Robot Control Loop
close all
figure
 while true
    % Get and plot recent odometry
    odom_data = receive(odom_sub);
    current_pose = odom_data.Pose.Pose;
    relative_x = [relative_x, initial_x_position - current_pose.Position.X];
    relative_y = [relative_y, initial_y_position - current_pose.Position.Y];
    scatter(relative_x,relative_y,[],'green','filled')

    % Calc desired spee
    delta_cur_to_targ_pose = CalcDeltaPoseToTarget(current_pose, targ_pose);
   
    
    


    % Preparing shit 
%    robot_Orientation = [new_odom_data.Pose.Pose.Orientation.W, new_odom_data.Pose.Pose.Orientation.X, new_odom_data.Pose.Pose.Orientation.Y, new_odom_data.Pose.Pose.Orientation.Z]
    

    

%     robot_Rotation = quat2rotm(robot_Orientation);
%     eulZYX = rotm2eul(robot_Rotation);
%     current_pose = [new_odom_data.Pose.Pose.Position.X new_odom_data.Pose.Pose.Position.Y eulZYX(1)];
%     fprintf("\nCurrent Pose:\n\tX: %f\n\tY: %f\n\tT: %f", new_odom_data.Pose.Pose.Position.X, new_odom_data.Pose.Pose.Position.Y, eulZYX(1));
%     [linear_vel, angular_vel] = DifferentialDrive_PD_Control(target_pose, current_pose);
%     % send this velocity command to robot
%     actuate(cmd_vel_pub, twist_msg, linear_vel,angular_vel);
    
end     
%% Calculate Delta between Current Pose and Target Pose
function delta_cur_to_targ_pose = CalcDeltaPoseToTarget(curr_pose, targ_pose)
    delta_cur_to_targ_pose = curr_pose;
    delta_cur_to_targ_pose.Position.X = curr_pose.Position.X - targ_pose.Position.X;
    delta_cur_to_targ_pose.Position.Y = curr_pose.Position.Y - targ_pose.Position.Y;

    cur_angle_quat = quaternion([curr_pose.Orientation.X curr_pose.Orientation.Y curr_pose.Orientation.Z curr_pose.Orientation.W ]);
    cur_angle_mat = quat2rotm(cur_angle_quat)
    angle_diff = atan2(cur_angle_mat(2,3), cur_angle_mat(3,3))


%     cur_angle_quat = quaternion([curr_pose.Orientation.X curr_pose.Orientation.Y curr_pose.Orientation.Z curr_pose.Orientation.W ]);
%     cur_angle_vec = quat2rotm(cur_angle_quat);
%     axang = rotm2axang(cur_angle_vec)

    % TODO Delta Angle
end


function actuate(cmd_vel_pub, twist_in, linear_vel, angular_vel)

    twist_out = twist_in;

    % assign linear velocity
    twist_out.Linear.X = linear_vel(1);
    twist_out.Linear.Y = linear_vel(2);
    twist_out.Linear.Z = linear_vel(3);
    % assign angular velocity
    twist_out.Angular.X = angular_vel(1);
    twist_out.Angular.Y = angular_vel(2);
    twist_out.Angular.Z = angular_vel(3);

    % send this velocity command to robot
    send(cmd_vel_pub,twist_out);
end