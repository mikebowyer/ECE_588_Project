
%%to display sample image for test
clear
clc
image_sub = imread('TestRawImage.jpeg');
%imshow(image_sub)

%%
%figure, hold on
    % ------ Get, rotate, and crop image
    image = imcrop(image_sub, [0, 240, 680, 480]);
    imshow(image)
    % ------ Hough Transform to Get Lines
    cannyImage = edge(rgb2gray(image), 'canny');
    subplot(221); imshow(image)
    subplot(222); imshow(cannyImage)
    subplot(223); imshow(image)
    subplot(224); imshow(image)
    [H, T, R] = hough(cannyImage, 'Theta', -30:0.5:30);
    P = houghpeaks(H,20, 'Theta', 0:0.5:80);
    lines = houghlines(cannyImage,T,R,P,'FillGap', 15, 'MinLength', 50);

    %PlotGroundLines(image, lines);
    max_len = 0;
    for k = 1:length(lines)
       xy = [lines(k).point1; lines(k).point2];
       subplot(223);plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
    
       % Plot beginnings and ends of lines
       subplot(223);plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
       subplot(223);plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');
    end

    % Getting Line of Best Fit from Hough Lines
    %-->coefficients = BestFitLine(lines);
    %-->PlotBestFitLine(coefficients)
    % Converting line of best fit coeffiencts to intercept and slope
    %-->intercept, theta = convertLineOfBestFit(coefficients)
    % Control the bot
    %-->sendControlMsg(intercept, theta)
      
%% Function Definitions

function image = grabAndCleanUpImage(image_sub)
    %image_compressed = image_sub;
    %image_compressed = 'bgr8; jpeg compressed bgr8';
    %rotatedImage = imrotate((image_compressed), 0);
    %eliminate the upper half of the image
    image = imcrop(image_sub, [0, 240, 680, 480]);
end

function lines = IsolateGroundLines(image)
    cannyImage = edge(rgb2gray(image), 'canny');
    subplot(221); imshow(image)
    subplot(222); imshow(cannyImage)
    subplot(223); imshow(image)
    subplot(224); imshow(image)
    [H, T, R] = hough(cannyImage, 'Theta', -30:0.5:30);
    P = houghpeaks(H,20, 'Theta', 0:0.5:80);
    lines = houghlines(cannyImage,T,R,P,'FillGap', 15, 'MinLength', 50);
    
end

function PlotGroundLines(image, lines)
    %imshow(image), hold on
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

function [intercept, theta] = convertLineOfBestFit(coefficients)

intercept = 0;
theta = 0;

end

function sendControlMsg(intercept, theta)

end

