classdef LineFollower
    %UNTITLED5 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        image_topic = "/camera/rgb/image_raw/compressed";
        %image_topic  = "/raspicam_node/image/compressed";
        lastDirection = 'unknown';
        EdgePlot;
        ImgFig;
    end

    methods

        function obj = LineFollower()
            obj.ImgFig = figure('Position',[0, 550, 960, 450]);
            obj.EdgePlot = imshow([]);
        end

        function [lin_vel, ang_vel] = CalcLineFollowVels(obj, image_compressed)
            image = obj.CleanUpImage(image_compressed);
            [img_height, img_width]= obj.imageSizeFinder(image);
            greenPoints = obj.GetGreenPoints(image);
            sufficientPixels = obj.CheckIfSufficientPixels(greenPoints, img_height*img_width, 0.05);
            boundaries = obj.findBoundaries(greenPoints, image);
            boundaryBWImage = obj.ConvertBoundariesToBW(image, boundaries);
            lines = obj.VerticalHoughWithBoundaryLines(boundaryBWImage);
            %horizontalLines = obj.HorizontalHoughWithBoundaryLines(boundaryBWImage);
            %lines = [verticalLines, horizontalLines];
            imageWithLines = boundaryBWImage;
            
            for k=1:length(lines)
                xy = [lines(k).point1; lines(k).point2];
%                 line(obj.EdgePlot, xy(:,1), xy(:,2),'LineWidth',2,'Color','green')       
                imageWithLines = insertShape(imageWithLines,'Line',[xy(:,1) xy(:,2)],'LineWidth',5, 'Color','cyan');
            end
            
            %direction = obj.GetDirectionFromHorizontalLines(img_width, horizontalLines, obj.lastDirection);

            % Getting Line of Best Fit from Hough Lines
            if (length(lines) == 0) || ~sufficientPixels
                lin_vel = .01;
                ang_vel = 0;
                return
            end
            lineBestFitPoints = obj.BestFitLineAvg(lines);
            imageWithLines = insertShape(imageWithLines,'Line',[lineBestFitPoints(:,1) lineBestFitPoints(:,2)],'LineWidth',5, 'Color','red');
            set(obj.EdgePlot, 'CData', imageWithLines);
            % Converting line of best fit to intercept and slope
            [theta, intercept_pixel, extent_points] = obj.calcBestFitLineInfo(lineBestFitPoints, img_height);
            [lin_vel, ang_vel] = obj.calcCmdVelMsg(intercept_pixel, theta, img_width);
            %obj.lastDirection = direction;
            set(obj.ImgFig,'Name', strcat('Line following: line angle:  ', string(theta), ', line intercept:  ', string(intercept_pixel),'(horizontal pixels)'));
        end

        function img_out = CleanUpImage(obj,image_compressed)
            image_compressed.Format = 'bgr8; jpeg compressed bgr8';
            rotatedImage = imrotate(readImage(image_compressed), 180);
            %eliminate the upper half of the image
            img_out = imcrop(rotatedImage, [0, 360, 640, 480]);
        end
        function [height, width] = imageSizeFinder(obj, image)
            imageSize = size(image);
            height = imageSize(1); % number of pixels vertically of cropped image
            width = imageSize(2); % number of pixels horizontally of cropped image
        end
        function greenPoints = GetGreenPoints(obj, originalImage)
            redLowerBound =40;% 
            redUpperBound =120;%
            greenLowerBound =90;% 
            greenUpperBound =130;%
            blueLowerBound =30;% 
            blueUpperBound =60;% 
            greenPoints = redUpperBound>=originalImage(:,:,1) & originalImage(:,:,1)>=redLowerBound & originalImage(:,:,2)<=greenUpperBound & originalImage(:,:,2)>=greenLowerBound & originalImage(:,:,3)<=blueUpperBound & originalImage(:,:,3)>=blueLowerBound;
            %subplot(222); imshow(greenPoints)
        end
        function sufficientPixels = CheckIfSufficientPixels(obj, greenPoints, numPixel, percent)
            greenPixelsPercent = sum(sum(greenPoints))/numPixel;
            sufficientPixels = false;
            if greenPixelsPercent >= percent;
                sufficientPixels = true;
            end
        end
        function boundaries = findBoundaries(obj, BW, rawImage)
            [B,L,N,A] = bwboundaries(BW);
            boundaries = {};
            %subplot(224); imshow(rawImage); hold on; 
%             subplot(221); imshow(rawImage); hold on; 
            % Loop through object boundaries  
            for k = 1:N 
                % Boundary k is the parent of a hole if the k-th column 
                % of the adjacency matrix A contains a non-zero element 
                if (nnz(A(:,k)) > 0) 
                    boundary = B{k};
                    %boundaries is array of arrays containing boundary points
                    boundaries = [boundaries, boundary];
%                      plot(boundary(:,2),... 
%                         boundary(:,1),'r','LineWidth',1); 
        %             % Loop through the children of boundary k 
        %             for l = find(A(:,k))' 
        %                 boundary = B{l}; 
        %                 plot(boundary(:,2),... 
        %                     boundary(:,1),'g','LineWidth',2); 
        %             end 
                end 
            end
        end 
        function BWBoundaryImage = ConvertBoundariesToBW(obj, image, boundaries)
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
        function lines = VerticalHoughWithBoundaryLines(obj,boundaries)
            [H, T, R] = hough(boundaries, 'Theta', -60:0.5:60);
            P = houghpeaks(H, 4, 'Theta', -60:0.5:60);
            lines = houghlines(boundaries,T,R,P,'FillGap', 50, 'MinLength', 50);  
        end
        function lines = HorizontalHoughWithBoundaryLines(obj, boundaries)
            [H, T, R] = hough(boundaries, 'Theta',[-90:0.5:-80 80:0.5:89]);
            P = houghpeaks(H, 4, 'Theta', [-90:0.5:-80 80:0.5:89]);
            lines = houghlines(boundaries,T,R,P,'FillGap', 50, 'MinLength', 50);  
        end

        function direction = GetDirectionFromHorizontalLines(obj, imageWidth, horizontalLines, lastDirection)
            startPoints = [];
            endPoints = [];
            if length(horizontalLines) ~= 0
                for k = 1:length(horizontalLines)
                    startPoints = [startPoints ; horizontalLines(k).point1];
                    endPoints = [endPoints ; horizontalLines(k).point2];
                end
        
                startPointX = mean(startPoints(:,1));
                endPointX = mean(endPoints(:,1));
                averageX = (startPointX + endPointX)/2;
                
                if averageX >= imageWidth/2
                    direction = 'right';
                else
                    direction = 'left';
                end
            else
                direction = lastDirection;
            end
        end
        function points = BestFitLineAvg(obj, lines)
            startPoints = [];
            endPoints = [];
            points = [];
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
        function [theta, intercept, points] = calcBestFitLineInfo(obj, lineBestFitPoints, img_height)
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

        function [lin_vel, ang_vel] = calcCmdVelMsg(obj, intercept_pixel, theta, img_width)
            ratio_intercept_from_img_center = (intercept_pixel - (img_width/2)) / (img_width/2);
            
            lin_vel = .01;
            
            max_turn_z_val = .1;
            intercept_part = -.5*((max_turn_z_val) * ratio_intercept_from_img_center);
            theta_part = -.5*max_turn_z_val * (-theta/90);
            ang_vel = theta_part + intercept_part;
        
            if abs(ang_vel) > 0.2
                ang_vel = 0.2 * sign(ang_vel)
            end
            
            %twist_out.Angular.Z = max_turn_z_val * (-theta/90) * (-(1 - ratio_intercept_from_img_center)*1.5);
         
        end
        
        function PlotGroundLines(image, lines)
            
            %subplot(223); imshow(image); hold on;
            for k = 1:length(lines)
               xy = [lines(k).point1; lines(k).point2];
               subplot(223);plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
            end  
        end

%         function PlotInterceptTheta(lineBestFitPoints, intercept,theta, image, z)
%             subplot(224);
%             imshow(image);
%             hold on; 
%             plot(lineBestFitPoints(:,1),lineBestFitPoints(:,2),'LineWidth',2,'Color','red');
%             title("Intercept Pixel: " + ceil(intercept) + "   Theta: " + theta + " Z: " + z)
%         end
    end
end