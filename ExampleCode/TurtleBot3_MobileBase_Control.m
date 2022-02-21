function [velocity_msg] = TurtleBot3_MobileBase_Control(target_pose_msg)


%%% read odometry
if ismember(TurtleBot_Topic.odom, rostopic('list'))
%     use rostopic info topicname to detemine the message type
    odom_sub = rossubscriber(TurtleBot_Topic.odom, 'nav_msgs/Odometry');
end
% odom_data = receive(odom_sub);

%%% read imu
if ismember(TurtleBot_Topic.imu, rostopic('list'))
    imu_sub = rossubscriber(TurtleBot_Topic.imu, 'sensor_msgs/Imu');
end
% imu_data = receive(imu_sub);

draw_odomData = animatedline('Color', 'c', 'LineWidth', 1.5, 'LineStyle', '-');
draw_imuData = animatedline('Color', 'm', 'LineWidth', 1.5, 'LineStyle', ':');
tic;
while toc < 20
    odom_data = receive(odom_sub);
    imu_data = receive(imu_sub);
    robot_Position = odom_data.Pose.Pose.Position;
    robot_Orientation = [imu_data.Orientation.W, imu_data.Orientation.X, imu_data.Orientation.Y, imu_data.Orientation.Z];
    robot_Rotation = quat2rotm(robot_Orientation);
    addpoints(draw_odomData, robot_Position.X, robot_Position.Y, robot_Position.Z);
    drawnow
end
%%% convert quaternion to rotation matrix
% rot = quat2rotm(quat);

%%% send velocity command to TurtleBot
if ismember(TurtleBot_Topic.vel, rostopic('list'))
    velocity_pub = rospublisher('/cmd_vel', 'geometry_msgs/Twist');
    %     velocity_sub = rossubscriber('cmd_vel', 'geometry_msgs/Twist');
end
velocity_msg = rosmessage(velocity_pub);

vel_x = animatedline('Color', 'c', 'LineWidth', 1.5, 'LineStyle', '-');
vel_theta = animatedline('Color', 'm', 'LineWidth', 1.5, 'LineStyle', ':');
tic;
while toc < 10
    velocity_msg = receive(robot_sub);
    addpoints(vel_x, toc, velocity_msg.Linear.X);
    %         hold on
    addpoints(vel_theta, toc, velocity_msg.Angular.Z);
    drawnow
end

[linear_vel, angular_vel] = DifferentialDrive_PD_Control(target_pose, current_pose);
% assign linear velocity
velocity_msg.Linear.X = linear_vel(1);
velocity_msg.Linear.Y = linear_vel(2);
velocity_msg.Linear.Z = linear_vel(3);
% assign angular velocity
velocity_msg.Angular.X = angular_vel(1);
velocity_msg.Angular.Y = angular_vel(2);
velocity_msg.Angular.Z = angular_vel(3);
% send this velocity command to robot
tic;
while toc < 20
    %     toc
    send(velocity_pub,velocity_msg);
end

end

function [linear_vel, angular_vel] = DifferentialDrive_PD_Control(target_pose, current_pose)
% controller parameters
parameters.Krho = 0.5;
parameters.Kalpha = 1.5;
parameters.Kbeta = -0.6;
parameters.Ktheta = 0.1;
parameters.backwardAllowed = true;
parameters.useConstantSpeed = false;
parameters.constantSpeed = 0.8;

% current robot position and orientation
x = current_pose(1);
y = current_pose(2);
theta = current_pose(3);

% goal position and orientation
xg = target_pose(1);
yg = target_pose(2);
thetag = target_pose(3);

% compute control quantities
rho = sqrt((xg-x)^2+(yg-y)^2);  % pythagoras theorem, sqrt(dx^2 + dy^2)
lambda = atan2(yg-y, xg-x);     % angle of the vector pointing from the robot to the goal in the inertial frame
alpha = lambda - theta;         % angle of the vector pointing from the robot to the goal in the robot frame
alpha = normalizeAngle(alpha);

beta = -lambda;
omega = parameters.Kalpha * alpha + parameters.Kbeta * beta + parameters.Ktheta * (thetag-theta); % [rad/s]
if parameters.useConstantSpeed
    %     omega = parameters.constantSpeed/vu * omega;
    vu = parameters.constantSpeed;
    omega = parameters.constantSpeed/(parameters.Krho * rho) * omega;
else
    vu = parameters.Krho * rho; % [m/s]
end

linear_vel = [vu; 0; 0]; % meters per second
angular_vel = [0; 0; omega];  % radius per second
end

function [angle1] = normalizeAngle(angle)
    %normalizeAngle   set angle to the range [-pi,pi)

    angle1 = mod( angle+pi, 2*pi) - pi;
end