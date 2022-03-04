rosshutdown

ip_TurtleBot = '127.0.0.1';    
ip_Matlab = '127.0.0.1';      

setenv('ROS_MASTER_URI', strcat('http://', ip_TurtleBot,':11311'))
setenv('ROS_IP', ip_Matlab)

rosinit(ip_TurtleBot)
%% Ros Topic Subscriptions and Publications
[cmd_vel_pub,twist_msg] = rospublisher('/cmd_vel','geometry_msgs/Twist');
%%

rostopic info /camera/rgb/image_raw/compressed
msg_camera = rostopic("echo", "camera/rgb/image_raw/compressed")
TurtleBot_Topic.picam = 'camera/rgb/image_raw/compressed';
image_sub = rossubscriber(TurtleBot_Topic.picam);
image_compressed = receive(image_sub);
%%
%%% to display a continuously updating image from Pi camera
tic;
prev_time = now
while toc < 20  
    %%% Edge detector
    %%% Takes raw image and isolated the line
    % 
    % Brandon's task, should try out the other camera
    image_compressed = receive(image_sub);
    image_compressed.Format = 'bgr8; jpeg compressed bgr8';
    rotatedImage = imrotate(readImage(image_compressed), 180);
    subplot(211); imshow(rotatedImage)
    subplot(212); imshow(edge(rgb2gray(rotatedImage), 'canny'))
    %%
    %input: the binary image containing only the line
    %Output: delta angle between robot heading and line, and the bottom of
    %image intercept
    %Once binary image is created, determin position
    %assume that we will be following a single line
    %find delta between current heading of robot - correct so line is at same
    %angle as robot's heading
    
    %%
    %input: angle delta between robot and line and bottom of image intercept
    %output: twist message in ROS to steer the robot
    %PID control - angle gets passed to this and must be corrected for
    %Determine if we are on the line, and if not get back on the line, then
    %follow line
    %will require an object to store the state
    line_angle = 0 % Angle from -90 = horzontal left, 90 = horizontal right
    intercept = 0 % 0 in middle, 50% at right edge, -50% at left edge

    target_ang=0
    target_intercept = 0

    % Error Calc
    err_angle = line_angle - target_ang
    err_intercept = intercept - target_intercept

    % delta time calc
    current_time = now
    time_delta = current_time - prev_time
    
    %

    twist_msg.Linear.X = .1
    twist_msg.Angular.Z = .25
    send(cmd_vel_pub,twist_msg);

end
