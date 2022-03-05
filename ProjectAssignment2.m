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
    %%% Edge detector
    %%% Takes raw image and isolates the line
    % 
    % Brandon's task, should try out the other camera
    image_compressed = receive(image_sub);
    image_compressed.Format = 'bgr8; jpeg compressed bgr8';
    rotatedImage = imrotate(readImage(image_compressed), 180);
    %eliminate the upper half of the image
    cutImage = imcrop(rotatedImage, [0, 240, 640, 480]);
    cannyImage = edge(rgb2gray(cutImage), 'canny');
    subplot(211); imshow(cutImage)
    subplot(212); imshow(cannyImage)
    [H, T, R] = hough(cannyImage, 'Theta', 0:0.5:80)
    P = houghpeaks(H,20, 'Theta', 0:0.5:80)
    lines = houghlines(cannyImage,T,R,P)
    figure, imshow(cutImage), hold on
    max_len = 0;
    for k = 1:length(lines)
       xy = [lines(k).point1; lines(k).point2];
       plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
    
       % Plot beginnings and ends of lines
       plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
       plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');
    
       % Determine the endpoints of the longest line segment
       len = norm(lines(k).point1 - lines(k).point2);
       if ( len > max_len)
          max_len = len;
          xy_long = xy;
       end
    end
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
end
