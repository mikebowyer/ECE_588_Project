ip_TurtleBot = '10.0.1.57';    
ip_Matlab = '10.0.1.54';      

setenv('ROS_MASTER_URI', strcat('http://', ip_TurtleBot,':11311'))
setenv('ROS_IP', ip_Matlab)

rosinit(ip_TurtleBot)
%%

rostopic info /raspicam_node/image/compressed
msg_camera = rostopic("echo", "/raspicam_node/image/compressed")
TurtleBot_Topic.picam = '/raspicam_node/image/compressed';
image_sub = rossubscriber(TurtleBot_Topic.picam);
image_compressed = receive(image_sub);
%%
%%% to display a continuously updating image from Pi camera
tic;
while toc < 20
    image_compressed = receive(image_sub);
    image_compressed.Format = 'bgr8; jpeg compressed bgr8';
    rotatedImage = imrotate(readImage(image_compressed), 180);
    subplot(211); imshow(rotatedImage)
    subplot(212); imshow(edge(rgb2gray(rotatedImage), 'canny'))
end