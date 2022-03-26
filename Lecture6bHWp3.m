clear all;
rosshutdown;
ip_TurtleBot = '10.0.1.57';    
ip_Matlab = '10.0.1.54'; 

setenv('ROS_MASTER_URI', strcat('http://', ip_TurtleBot,':11311'))
setenv('ROS_IP', ip_Matlab)

rosinit(ip_TurtleBot)
TurtleBot_Topic.picam = "/raspicam_node/image/compressed";
image_sub = rossubscriber(TurtleBot_Topic.picam);
laser_sub = rossubscriber('/scan');
%% 

while true 
    % Get, rotate, and crop image
    image = grabAndCleanUpImage(image_sub);
    subplot(221); imshow(image);
    scan_data = receive(laser_sub);
    subplot(222); plot(scan_data);
    slicedData = getFrontFacingScanData(scan_data);
    subplot(223); scatter(slicedData(:,1),slicedData(:,2));
    xlim([-0.5, 0.5])
    ylim([0, 2])
    overlayDistance(slicedData, image);
end

%% 

function image = grabAndCleanUpImage(image_sub)
    image_compressed = receive(image_sub);
    image_compressed.Format = 'bgr8; jpeg compressed bgr8';
    image = imrotate(readImage(image_compressed), 180);
    
    %eliminate the upper half of the image
%     image = imcrop(rotatedImage, [0, 360, 640, 480]);
end

function slicedData = getFrontFacingScanData(scan_data)
    data = readCartesian(scan_data);
    %camera horizonal FOV is roughly 50 degrees total
    slicedData = [];
    for j=1:length(data)
        if data(j,2) ~= 0 && data(j,1) > 0
            if atan(abs(data(j,1)/data(j,2))) > 1.1
                slicedData = [slicedData, [data(j,2); data(j,1)-0.06]]; %0.06 is the offset between camera and lidar
            end
        end
    end
    slicedData = transpose(slicedData);
end

function overlayDistance(slicedData, originalImage)    
    farPoint = 1.0; %red
    mediumPoint = 0.5; %green
    closePoint = 0.2; %blue
    imageSize = size(originalImage);
    imageWidth = imageSize(2);
    imageMidpoint = round(imageSize(1)/2);
    newSlicedData = convertSlicedDataToPixels(slicedData, imageWidth);
    c = 'gray';
    subplot(224); imshow(originalImage);
    hold on;
    for j=1:length(newSlicedData)
        %map to the image plane
        if newSlicedData(j,2) >= farPoint
            c = 'red';
        elseif newSlicedData(j,2) < farPoint && newSlicedData(j,2) >= mediumPoint
            c = 'green';
        elseif newSlicedData(j,2) < mediumPoint && newSlicedData(j,2) > closePoint
            c = 'blue';
        elseif newSlicedData(j,2) <= closePoint
            c = 'magenta';
        end
        scatter(newSlicedData(j,1),imageMidpoint,[],c)
    end
end

function newSlicedData = convertSlicedDataToPixels(slicedData, imageWidth)
    minX = min(slicedData(:,1));
    maxX = max(slicedData(:,1));
    xRange = maxX - minX;
    newSlicedData = [];
    for j=1:length(slicedData)
        xRangeRatio = (slicedData(j,1)+abs(minX))/xRange;
        pixelNumber = round(imageWidth*xRangeRatio);
        newSlicedData = [newSlicedData, [pixelNumber;slicedData(j,2)]];
    end
    newSlicedData=transpose(newSlicedData);
end