function [pose_msg] = TurtleBot3_Perception()

%% Read sensor data

%%% read lidar data
if ismember(TurtleBot_Topic.laser, rostopic('list'))
    laser_sub = rossubscriber(TurtleBot_Topic.laser);
end
scan_data = receive(laser_sub);
cart = readCartesian(scan_data, 'RangeLimit', [0.12 3.5]);
plot(cart(:,1),cart(:,2), 'o');

%%% plot laser data
tic;
while toc < 20
    scan_data = receive(laser_sub);
    plot(scan_data);
end

%%% read images
% images captured by Pi camera, if you are using Gazebo, the topic list is different.
if ismember(TurtleBot_Topic.picam, rostopic('list'))
    image_sub = rossubscriber(TurtleBot_Topic.picam);
end
image_compressed = receive(image_sub);

image_compressed.Format = 'bgr8; jpeg compressed bgr8';
figure
imshow(readImage(image_compressed));
%%% to display a continuously updating image from Pi camera
tic;
while toc < 20
    image_compressed = receive(image_sub);
    image_compressed.Format = 'bgr8; jpeg compressed bgr8';
    imshow(readImage(image_compressed))
end