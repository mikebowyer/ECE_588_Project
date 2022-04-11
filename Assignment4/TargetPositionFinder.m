classdef TargetPositionFinder
    %Finds the angle between the center of the camera and the target
    %

    properties
        cameraCenterColumnX = 320;
        cameraCenterRowY = 240;
        f=0.0036;
        pixelSize = 0.0000014;
        csize = 0.0000014; %pixel sizeis csize
        degreesPerPixel = 54/640;

        %% Plots for Debugging
        ImgFig
        EdgePlot
        EllipPlot
    end

    methods
        function obj = TargetPositionFinder()
            obj.ImgFig = figure('Position',[960, 0, 960, 540]);
            subplot(211);
            obj.EdgePlot = imshow([]);
            subplot(212);
            obj.EllipPlot = imshow([]);
        end

        function [xGlobal, yGlobal] = CalcTargetPosition(obj, ellipseXrelative,scan_data,odom_data)
            robotCurrentPosX = odom_data.Pose.Pose.Position.X;
            robotCurrentPosY = odom_data.Pose.Pose.Position.Y;
            offset = ellipseXrelative - obj.cameraCenterColumnX;
            targetAngle = round(-offset*obj.degreesPerPixel);
            rangeToTarget = 0;
            if targetAngle >= 0
                rangeToTarget = scan_data.Ranges(targetAngle+1);
            else
                rangeToTarget = scan_data.Ranges(length(scan_data.Ranges) + targetAngle);
            end
%             x = sin(deg2rad(targetAngle)) * rangeToTarget;
%             y = cos(deg2rad(targetAngle)) * rangeToTarget;
            %Next step, convert taret coordinates to global coordinates
            robotCurrentHeading = tan(robotCurrentPosY/robotCurrentPosX); %get the angle the robot is facing
            angleToTargetInGlobalCoords = robotCurrentHeading - rad2deg(targetAngle); %angle between straigh line path to target and global X axis
            xGlobal = rangeToTarget * cos(angleToTargetInGlobalCoords);
            yGlobal = rangeToTarget * sin(angleToTargetInGlobalCoords);
        end

        function [ellipseX, ellipseY] = FindTargetPixelCoords(obj, image_compressed)
            image_compressed.Format = 'bgr8; jpeg compressed bgr8';
            %image = readImage(image_compressed);
            image = imrotate(readImage(image_compressed), 180);
            targetImageHSV = rgb2hsv(image);
            targetImageSonly = targetImageHSV(:,:,2);
            %subplot(233); imshow(targetImageSonly)
            S_thresh = 0.3;
            HSV_Thresh_image = S_thresh>=targetImageSonly;
            [bw_Canny,threshOut] = edge(HSV_Thresh_image,'Canny', [0.00, 0.25],5); 
            %subplot(234); imshow(image)
            %hold on
            params.minMajorAxis = 15;
            params.maxMajorAxis = 200;
            params.numBest = 1;
            params.rotation = 90;
            params.rotationSpan = 10;
            
            ellipseX = NaN;
            ellipseY = NaN;
            % note that the edge (or gradient) image is used
            bestFits = ellipseDetection(bw_Canny, params);
            if bestFits(6) > 10
                ellipseX = bestFits(1,1); %column
                ellipseY = bestFits(1,2); %row
                % Update Images
                set(obj.EdgePlot, 'CData', bw_Canny);
                image = insertShape(image,'circle',[ellipseX ellipseY bestFits(1,3)],'LineWidth',5);
                set(obj.EllipPlot, 'CData', image);
            else
                return;
            end
            %imshow(image)
            %ellipse(bestFits(1,3),bestFits(1,4),bestFits(1,5)*pi/180,bestFits(1,1),bestFits(1,2),'r');    

            % Update Images
%             set(obj.EdgePlot, 'CData', bw_Canny);
%             image = insertShape(image,'circle',[ellipseX ellipseY bestFits(1,3)],'LineWidth',5);
%             set(obj.EllipPlot, 'CData', image);
        end
    end
end