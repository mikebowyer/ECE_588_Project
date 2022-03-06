ip_TurtleBot = '127.0.0.1';    
ip_Matlab = '127.0.0.1 ';      

setenv('ROS_MASTER_URI', strcat('http://', ip_TurtleBot,':11311'))
setenv('ROS_IP', ip_Matlab)

rosinit(ip_TurtleBot)
%%

TurtleBot_Topic.picam = "/camera/rgb/image_raw/compressed";
%TurtleBot_Topic.picam = "/raspicam_node/image/compressed";
image_sub = rossubscriber(TurtleBot_Topic.picam);
%%
%%% to display a continuously updating image from Pi camera
%%
close all
figure, hold on
while true 
    % Get, rotate, and crop image
    image = grabAndCleanUpImage(image_sub);
    % Hough Transform to Get Lines
    lines = IsolateGroundLines(image);
    PlotGroundLines(image, lines);
    % Getting Line of Best Fit from Hough Lines
    coefficients = BestFitLine(lines);
    PlotBestFitLine(coefficients)
    % Converting line of best fit coeffiencts to intercept and slope
    %intercept, theta = convertLineOfBestFit(coefficients)
    % Control the boto
    %sendControlMsg(intercept, theta)
end
     
%%

function image = grabAndCleanUpImage(image_sub)
    image_compressed = receive(image_sub);
    image_compressed.Format = 'bgr8; jpeg compressed bgr8';
    rotatedImage = imrotate(readImage(image_compressed), 0);
    %eliminate the upper half of the image
    image = imcrop(rotatedImage, [0, 240, 640, 480]);
end
function lines = IsolateGroundLines(image)
    cannyImage = edge(rgb2gray(image), 'canny');
    subplot(221); imshow(image)
    subplot(222); imshow(cannyImage)
    subplot(223); imshow(image)
    subplot(224); imshow(image)
    [H, T, R] = hough(cannyImage, 'Theta', -45:0.5:45);
    P = houghpeaks(H,20, 'Theta', 0:0.5:80);
    lines = houghlines(cannyImage,T,R,P,'FillGap', 15, 'MinLength', 50);
    
end

function PlotGroundLines(image, lines)
    imshow(image), hold on
    max_len = 0;
    for k = 1:length(lines)
       xy = [lines(k).point1; lines(k).point2];
       subplot(223);plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
    
       % Plot beginnings and ends of lines
       subplot(223);plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
       subplot(223);plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');
    
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

function coefficients = BestFitLine(lines)
    coefficients = 0;
    if length(lines) ~= 0
        xy = zeros;
        for k = 1:length(lines)
            xyNew = [lines(k).point1; lines(k).point2];
            if k == 1
                xy = xyNew;
            else
                xy = cat(1,xy,xyNew);
            end
        end
        coefficients = polyfit(xy(:,1),xy(:,2), 1);
    end
end

function PlotBestFitLine(coefficients)
    y1 = polyval(coefficients,1);
    y2 = polyval(coefficients,640);
    hold on
    subplot(224);plot([1, 640],[y1, y2],'LineWidth',2,'Color','red');
end

%

function sendControlMsg(intercept, theta)

end

% Kay's addition functions

%% Function Definitions

% returns angle in degrees of robot path line 
function theta = convertLineOfBestFit(coefficients) 
    theta = 0; % initialize angle variable
    vect1 = [1 coefficients(1)]; % create a vector based on the line equation slope
    vect2 = [1 0]; % create horizontal line vector 
    dp = dot(vect1, vect2);
    
    % compute vector lengths
    length1 = sqrt(sum(vect1.^2));
    length2 = sqrt(sum(vect2.^2));
    
    % obtain the smaller angle of intersection in degrees
    theta = acos(dp/(length1*length2))*180/pi;

end

% returns offset via x-intercept with hottom horizontal edge of image
function x_intercept = findBottonIntercept(coefficients)

    x_intercept = 0;
    x = [0:1:250];
    % plot the line of robot path given coefficents
    y = x*coefficients(1) + coefficients(2);
    %hold on, subplot(2,1,2) 
    %plot(y,x)
    %xlim([0 600])
    %ylim([0 250])
    x_intercept = y(1); %first index value 

end

% returns negative pixel offset value from x_intercept if path line is left
% of robot current position
% returns positive pizel offset value from x_intercept if path line is
% right of robot current posiont
function centerOffset = imageCenterOffsetFinder(width)

    %half width is center of image
    centerOffset = -30; %hard code dummy value


end

% return number of pixels of cropped image used for path prediction
function [height, width] = imageSizeFinder(image)
    
    imageSize = size(image);
    height = imageSize(1); % number of pixels vertically of cropped image
    width = imageSize(2); % number of pixels horizontally of cropped image
    
end