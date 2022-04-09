clear all;
rosshutdown;
ip_TurtleBot = '127.0.0.1';    
ip_Matlab = '127.0.0.1 ';  
%ip_TurtleBot = '10.0.1.57';    
%ip_Matlab = '10.0.1.54'; 
setenv('ROS_MASTER_URI', strcat('http://', ip_TurtleBot,':11311'))
setenv('ROS_IP', ip_Matlab)
rosinit(ip_TurtleBot)
%% Odometry Setup
% Odometry Subscription
TurtleBot_Topic.odom = '/odom';
odom_sub = rossubscriber('/odom');
[cmd_vel_pub,twist_msg] = rospublisher('/cmd_vel','geometry_msgs/Twist');

%% Target Parameters
stop_dist_thresh  = .25;
[targ_pose_pub,targ_pose] = rospublisher("/pose","geometry_msgs/Pose","DataFormat","struct");
targ_pose.Position.X = 0;
targ_pose.Position.Y = 0;
%% Run Main Robot Control Loop
close all
figure('units','normalized','outerposition',[0 0 1 1])
%figure

% Position Setup
initial_odom_data = receive(odom_sub);
initial_x_position = initial_odom_data.Pose.Pose.Position.X;
initial_y_position = initial_odom_data.Pose.Pose.Position.Y;
relative_x = [];
relative_y = [];

%Plot Starting Position, Target, and robot Position
subplot(211); 
scatter(targ_pose.Position.X,targ_pose.Position.Y,1000,'red','x','M');hold on;
scatter(initial_x_position,initial_y_position,1000,'green','x');
lnh = scatter(initial_x_position,initial_y_position,1000,'yellow','x');
title("Robot Start, Target, and Current Position")
legend("Target Position", "Starting Position","Robot Position");
linkdata on; grid on;

% Setup PID Controllers and stop robot to start with.
ang_pid = AngularPIDController();
lin_pid = LinearPIDController();
actuate(cmd_vel_pub, twist_msg, 0, 0); pause(1);

% Start while loop
subplot(212); 
tic
 while true
    % Get odometry
    odom_data = receive(odom_sub);
    current_pose = odom_data.Pose.Pose;
    % Plot odometry
    set(lnh,'XData',current_pose.Position.X,'YData',current_pose.Position.Y); % change the line data
    relative_x = [relative_x, initial_x_position - current_pose.Position.X];
    relative_y = [relative_y, initial_y_position - current_pose.Position.Y];
    scatter(relative_x,relative_y,[],'green','filled'); grid on;

    % Calculate the robot position delta to the target
    [dist_to_targ, ang_to_targ] = CalcDeltaPoseToTarget(current_pose, targ_pose);

    % Run Control Algos
    curr_time = toc;
    if dist_to_targ < stop_dist_thresh
        actuate(cmd_vel_pub, twist_msg, 0, 0)
        title("Arrived at target, stopping!")
        return
    else
        ang_vel = ang_pid.CalcAngVel(curr_time, ang_to_targ);
        lin_vel = lin_pid.CalcLinVel(curr_time, dist_to_targ);
        actuate(cmd_vel_pub, twist_msg, lin_vel, ang_vel)
        title(["DistToTarg: " dist_to_targ, "AngToTarg: " ang_to_targ, "LinVel: " lin_vel, "AngVel:" ang_vel])
    end
    
    
    tic;
end     
%% Calculate Delta between Current Pose and Target Pose
function [delta_dist, delta_theta] = CalcDeltaPoseToTarget(curr_pose, targ_pose)
    d_x = targ_pose.Position.X - curr_pose.Position.X;
    d_y = targ_pose.Position.Y - curr_pose.Position.Y;

    % Get current robot angle
    cur_angle_quat = quaternion([curr_pose.Orientation.X curr_pose.Orientation.Y curr_pose.Orientation.Z curr_pose.Orientation.W ]);
    cur_angle_mat = quat2rotm(cur_angle_quat);
    cur_angle = wrapToPi(atan2(cur_angle_mat(2,3), cur_angle_mat(3,3)));

    % Get angle from robot directly to the target
    targ_ang = atan2(d_y,d_x);
    delta_theta = targ_ang-cur_angle;

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

