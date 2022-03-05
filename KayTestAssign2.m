%% Kay's Test code for project assignment
% Load Processed Image from Brandon's section
clear
clc

TestImage = imread('TestImage_fromBrandon.jpeg');
imshow(TestImage)

text(size(TestImage,2),size(TestImage,1)+15,'Image courtesy Brandon Hagans',...
     'FontSize',7,'HorizontalAlignment','right');

line([300 328],[85 103],'color',[1 1 0]);
line([268 255],[85 140],'color',[1 1 0]);

text(150,72,'Measure the angle between line','Color','y',...
     'FontWeight', 'bold');

%% Step 2: Extract Region of Interest from Image
start_row = 34;
start_col = 208;

croppedImage = TestImage(start_row:163, start_col:400, :);

% Store (X,Y) offsets for later use; subtract 1 so that each offset will
% correspond to the last pixel before the region of interest
offsetX = start_col-1;
offsetY = start_row-1;

%% Step 3: Threshold the Image (already completed by Brandon)

%% Step 4: Find Initial Point on Each Boundary

%% Step 5: Trace Boundaries

%% Step 7: Find Angle of Intersection

%% Step 8: Find the Point of Intersection

%% Step 9: Plot & Return values from step 7 & 8


