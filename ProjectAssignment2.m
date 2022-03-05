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
%%
figure, hold on
while true 
    % Edge detector
    %Takes raw image and isolates the line
    % 
    % Brandon's task, should try out the other camera
    image_compressed = receive(image_sub);
    image_compressed.Format = 'bgr8; jpeg compressed bgr8';
    rotatedImage = imrotate(readImage(image_compressed), 180);
    %eliminate the upper half of the image
    cutImage = imcrop(rotatedImage, [0, 240, 640, 480]);

    lines = IsolateGroundLines(cutImage);
    PlotGroundLines(cutImage, lines);
    BestFitLine(lines);
    %DetermineRoughlyParallelLines(cutImage, lines);
    %clf;
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
%%
function lines = IsolateGroundLines(image)
    cannyImage = edge(rgb2gray(image), 'canny');
    subplot(211); imshow(image)
    subplot(212); imshow(cannyImage)
    [H, T, R] = hough(cannyImage, 'Theta', 0:0.5:80);
    P = houghpeaks(H,20, 'Theta', 0:0.5:80);
    lines = houghlines(cannyImage,T,R,P,'FillGap', 30, 'MinLength', 80);
    
end

function PlotGroundLines(image, lines)
    imshow(image), hold on
    max_len = 0;
    for k = 1:length(lines)
       xy = [lines(k).point1; lines(k).point2];
       plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
    
       % Plot beginnings and ends of lines
       plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
       plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');
    
       % Determine the endpoints of the longest line segment
%        len = norm(lines(k).point1 - lines(k).point2);
%        if ( len > max_len)
%          max_len = len;
%          xy_long = xy;
%        end
    end  
end

function DetermineRoughlyParallelLines(colorImage, lines)
    %filledInImage = colorImage;  %what to return if nothing is found
    figure, imshow(colorImage), hold on
    for j = 1:length(lines)
        for k = 1:length(lines)
            if j ~= k
                if lines(j).theta >= (lines(k).theta - 5) & lines(j).theta <= (lines(k).theta + 5)
                    %lines are roughly parallel, get average lines of two
                    averageLinePoint1x = round(mean(lines(j).point1(1), lines(k).point1(1)))
                    averageLinePoint1y = round(mean(lines(j).point1(2), lines(k).point1(2)))
                    averageLinePoint2x = round(mean(lines(j).point2(1), lines(k).point2(1)))
                    averageLinePoint2y = round(mean(lines(j).point2(2), lines(k).point2(2)))
                    %create line between two points
                    plot(averageLinePoint1x,averageLinePoint1y,'x','LineWidth',2,'Color','yellow');
                    plot(averageLinePoint2x,averageLinePoint2y,'x','LineWidth',2,'Color','red');
                end
            end
        end
    end
end

function BestFitLine(lines)
    xy = zeros;
    for k = 1:length(lines)
        xyNew = [lines(k).point1; lines(k).point2];
        if k == 1
            xy = xyNew;
        else
            xy = cat(1,xy,xyNew);
        end
    end
    xy;
    coefficients = polyfit(xy(:,1),xy(:,2), 1);
    y1 = polyval(coefficients,1);
    y2 = polyval(coefficients,640);
    hold on
    plot([1, 640],[y1, y2],'LineWidth',2,'Color','red');
end