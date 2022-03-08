%% crop test
clear;  clc;

%% determine angle and left/right offset of robot
clear;  clc;
% coefficients of line of best fit (example value for test)
coefficients = [-2, 187];

pathAngle = convertLineOfBestFit(coefficients);

%% test robot path line overlay with image
overlayImage = imread('TestRawImage.jpeg');
subplot(2,1,1), imshow(overlayImage); 
subplot(2,1,2), xlabel('image width'), ylabel('image height')
x_intercept = findBottonIntercept(coefficients);

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
    hold on, subplot(2,1,2) 
    plot(y,x)
    xlim([0 600])
    ylim([0 250])
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
