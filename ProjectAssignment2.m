%ip_TurtleBot = '127.0.0.1';    
%ip_Matlab = '127.0.0.1 ';  

ip_TurtleBot = '10.0.1.57';    
ip_Matlab = '10.0.1.54'; 

setenv('ROS_MASTER_URI', strcat('http://', ip_TurtleBot,':11311'))
setenv('ROS_IP', ip_Matlab)

rosinit(ip_TurtleBot)
%% Setup Topic Subscriptions and Publications
% Subscriptions
%TurtleBot_Topic.picam = "/camera/rgb/image_raw/compressed";
TurtleBot_Topic.picam = "/raspicam_node/image/compressed";
image_sub = rossubscriber(TurtleBot_Topic.picam);
% Publications
[cmd_vel_pub,twist_msg] = rospublisher('/cmd_vel','geometry_msgs/Twist');

%% Run Main Robot Control Loop
close all
%figure('units','normalized','outerposition',[0 0 1 1]), hold on
figure
while true 
    % Get, rotate, and crop image
    image = grabAndCleanUpImage(image_sub);
    %image = grabAndCleanUpImageGreen(image_sub);
    [img_height, img_width]= imageSizeFinder(image);
    
    %get only green line and get its boundary
    greenPoints = GetGreenPoints(image);
    boundaries = findBoundaries(greenPoints, image);
    boundaryBWImage = ConvertBoundariesToBW(image, boundaries);
    lines = VerticalHoughWithBoundaryLines(boundaryBWImage);
    %verticalalLines = VerticalHoughWithBoundaryLines(boundaryBWImage);
    %horizontalLines = HorizontalHoughWithBoundaryLines(boundaryBWImage);
    %lines = [verticalalLines, horizontalLines];

    % Hough Transform to Get Lines
    %lines = IsolateGroundLines(image);
    %lines = IsolateGroundLinesGreen(image);
    PlotGroundLines(image, lines);

    % Getting Line of Best Fit from Hough Lines
    if length(lines) == 0
        twist_msg.Linear.X = .0;
        for k = 1:10
            twist_msg.Linear.Z = -.2;
            send(cmd_vel_pub,twist_msg);
        end        
        continue
    end
    lineBestFitPoints = BestFitLineAvg(lines);

    % Converting line of best fit to intercept and slope
    [theta, intercept, extent_points] = calcBestFitLineInfo(lineBestFitPoints, img_height);
    
    
    % Control the boto
    twist_msg = calcCmdVelMsg(intercept, theta, twist_msg, img_width);
    PlotInterceptTheta(extent_points, intercept,theta, image, twist_msg.Angular.Z)
    send(cmd_vel_pub,twist_msg);
end
     
%%
function [theta, intercept, points] = calcBestFitLineInfo(lineBestFitPoints, img_height)
    rise  = lineBestFitPoints(1,2) - lineBestFitPoints(2,2);
    run = lineBestFitPoints(1,1) - lineBestFitPoints(2,1);
    slope = rise / run;

    theta = rad2deg(atan(slope));
    if theta > 0
        theta = 90 - theta;
    else
        theta = -90 - theta;
    end

    mx_b_intercept = lineBestFitPoints(1,2) - slope * lineBestFitPoints(1,1);

    y1 = 1;
    x1 = (1 -mx_b_intercept) / slope;
    y2 = img_height;
    x2 = (img_height -mx_b_intercept) / slope;
    
    intercept = x2;
    points = [x1 y1; x2 y2];

end
function PlotInterceptTheta(lineBestFitPoints, intercept,theta, image, z)
    subplot(224);
    imshow(image);
    hold on; 
    plot(lineBestFitPoints(:,1),lineBestFitPoints(:,2),'LineWidth',2,'Color','red');
    title("Intercept Pixel: " + ceil(intercept) + "   Theta: " + theta + " Z: " + z)
end

function image = grabAndCleanUpImage(image_sub)
    image_compressed = receive(image_sub);
    image_compressed.Format = 'bgr8; jpeg compressed bgr8';
    rotatedImage = imrotate(readImage(image_compressed), 180);
    %eliminate the upper half of the image
    image = imcrop(rotatedImage, [0, 360, 640, 480]);
end

function binaryImage = grabAndCleanUpImageGreen(image_sub)
    image_compressed = receive(image_sub);
    image_compressed.Format = 'bgr8; jpeg compressed bgr8';
    rotatedImage = imrotate(readImage(image_compressed), 180);
    %eliminate the upper half of the image
    image = imcrop(rotatedImage, [0, 240, 640, 480]);
    %isolate only green portion
    greenImage = image(:,:,2);
    thresholdUpper = 200/255;
    thresholdLower = 40/255;
    binaryImage = imbinarize(greenImage, thresholdLower);
    %binaryImage = uint8(round(190*imbinarize(greenImage, thresholdUpper).*(1-imbinarize(greenImage, thresholdLower))+10));
end



function lines = IsolateGroundLines(image)
    cannyImage = edge(rgb2gray(image), 'canny');
    subplot(221); imshow(image)
    subplot(222); imshow(cannyImage)
    [H, T, R] = hough(cannyImage, 'Theta', -60:0.5:60);
    P = houghpeaks(H, 4, 'Theta', -60:0.5:60);
    lines = houghlines(cannyImage,T,R,P,'FillGap', 50, 'MinLength', 80);    
end

function lines = IsolateGroundLinesGreen(image)
    cannyImage = edge(image, 'canny');
    subplot(221); imshow(image)
    subplot(222); imshow(cannyImage)
    [H, T, R] = hough(cannyImage, 'Theta', -60:0.5:60);
    P = houghpeaks(H, 4, 'Theta', -60:0.5:60);
    lines = houghlines(cannyImage,T,R,P,'FillGap', 50, 'MinLength', 80);    
end

function PlotGroundLines(image, lines)
    
    subplot(223); imshow(image); hold on;
    for k = 1:length(lines)
       xy = [lines(k).point1; lines(k).point2];
       subplot(223);plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
    
       % Plot beginnings and ends of lines
       %subplot(223);plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
       %subplot(223);plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');
    
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
                    averageLinePoint1x = round(mean(lines(j).point1(1), lines(k).point1(1)));
                    averageLinePoint1y = round(mean(lines(j).point1(2), lines(k).point1(2)));
                    averageLinePoint2x = round(mean(lines(j).point2(1), lines(k).point2(1)));
                    averageLinePoint2y = round(mean(lines(j).point2(2), lines(k).point2(2)));
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
        coefficients = polyfit(xy(:,1),-xy(:,2), 1);
    end
end

function points = BestFitLineAvg(lines)
    startPoints = [];
    endPoints = [];
    
    if length(lines) ~= 0
        for k = 1:length(lines)
            startPoints = [startPoints ; lines(k).point1];
            endPoints = [endPoints ; lines(k).point2];
        end

        startPointX = mean(startPoints(:,1));
        startPointY = mean(startPoints(:,2));

        endPointX = mean(endPoints(:,1));
        endPointY = mean(endPoints(:,2));

        points = [startPointX startPointY; endPointX endPointY];

    end
end


function twist_out = calcCmdVelMsg(intercept_pixel, theta, twist_in, img_width)
    twist_out = twist_in;
    

    ratio_intercept_from_img_center = (intercept_pixel - (img_width/2)) / (img_width/2);
    
    twist_out.Linear.X = .01;
    
    max_turn_z_val = .1;
    intercept_part = -.5*((max_turn_z_val) * ratio_intercept_from_img_center)
    theta_part = -.5*max_turn_z_val * (-theta/90)
    twist_out.Angular.Z = theta_part + intercept_part;

    if abs(twist_out.Angular.Z) > 0.2
        twist_out.Angular.Z = 0.2 * sign(twist_out.Angular.Z)
    end
    
    %twist_out.Angular.Z = max_turn_z_val * (-theta/90) * (-(1 - ratio_intercept_from_img_center)*1.5);
 
end

% returns angle in degrees of robot path line 
function theta = convertLineOfBestFit(coefficients) 
    theta = rad2deg(atan(-coefficients(1)))
    if theta > 0
        theta = 90 - theta
    else
        theta = -90 - theta
    end
%     theta = 0; % initialize angle variable
%     vect1 = [1 coefficients(1)]; % create a vector based on the line equation slope
%     vect2 = [1 0]; % create horizontal line vector 
%     dp = dot(vect1, vect2);
%     
%     % compute vector lengths
%     length1 = sqrt(sum(vect1.^2));
%     length2 = sqrt(sum(vect2.^2));
%     
%     % obtain the smaller angle of intersection in degrees
%     theta = acos(dp/(length1*length2))*180/pi;

end

% returns offset via x-intercept with hottom horizontal edge of image
function pixels_from_img_center = findInterceptFromImgCenter(coefficients, img_height, img_width)
    y = img_height;
    x_intercept = (y - coefficients(2)) / coefficients(1);
    pixels_from_img_center = x_intercept - (img_width /2);

end

% return number of pixels of cropped image used for path prediction
function [height, width] = imageSizeFinder(image)
    
    imageSize = size(image);
    height = imageSize(1); % number of pixels vertically of cropped image
    width = imageSize(2); % number of pixels horizontally of cropped image
end

function greenPoints = GetGreenPoints(originalImage)
    redLowerBound =40;% 
    redUpperBound =120;%
    greenLowerBound =70;% 
    greenUpperBound =100;%
    blueLowerBound =15;% 
    blueUpperBound =45;% 
    greenPoints = redUpperBound>=originalImage(:,:,1) & originalImage(:,:,1)>=redLowerBound & originalImage(:,:,2)<=greenUpperBound & originalImage(:,:,2)>=greenLowerBound & originalImage(:,:,3)<=blueUpperBound & originalImage(:,:,3)>=blueLowerBound;
    subplot(222); imshow(greenPoints)
end

function BWBoundaryImage = ConvertBoundariesToBW(image, boundaries)
    %create empty image
    [rows, columns, numberOfColorChannels] = size(image);
    BWBoundaryImage = zeros(rows, columns, 1);
    for k=1:length(boundaries)
        %boundaryRows = boundaries{1,k}(:,1);
        %boundaryColumns = boundaries{1,k}(:,2);
        for j=1:length(boundaries{1,k})
            BWBoundaryImage(boundaries{1,k}(j,1),boundaries{1,k}(j,2), 1)=1;
        end        
    end  
    %subplot(224); imshow(BWBoundaryImage); title('BWBoundaryImage')
end

function boundaries = findBoundaries(BW, rawImage)
    [B,L,N,A] = bwboundaries(BW);
    boundaries = {};
    %subplot(224); imshow(rawImage); hold on; 
    subplot(221); imshow(rawImage); hold on; 
    % Loop through object boundaries  
    for k = 1:N 
        % Boundary k is the parent of a hole if the k-th column 
        % of the adjacency matrix A contains a non-zero element 
        if (nnz(A(:,k)) > 0) 
            boundary = B{k};
            %boundaries is array of arrays containing boundary points
            boundaries = [boundaries, boundary];
             plot(boundary(:,2),... 
                boundary(:,1),'r','LineWidth',1); 
%             % Loop through the children of boundary k 
%             for l = find(A(:,k))' 
%                 boundary = B{l}; 
%                 plot(boundary(:,2),... 
%                     boundary(:,1),'g','LineWidth',2); 
%             end 
        end 
    end
end 

function lines = VerticalHoughWithBoundaryLines(boundaries)
    [H, T, R] = hough(boundaries, 'Theta', -60:0.5:60);
    P = houghpeaks(H, 4, 'Theta', -60:0.5:60);
    lines = houghlines(boundaries,T,R,P,'FillGap', 50, 'MinLength', 30);  
end

function lines = HorizontalHoughWithBoundaryLines(boundaries)
    [H, T, R] = hough(boundaries, 'Theta', -90:0.5:89);
    P = houghpeaks(H, 4, 'Theta', -90:0.5:89);
    lines = houghlines(boundaries,T,R,P,'FillGap', 50, 'MinLength', 30);  
end