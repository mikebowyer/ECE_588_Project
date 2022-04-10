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
    end

    methods
        function [xGlobal, yGlobal] = CalcTargetPosition(obj, ellipseXrelative,scan_data,robotCurrentPosX,robotCurrentPosY)
            offset = ellipseXrelative - cameraCenterColumnX;
            targetAngle = round(-offset*degreesPerPixel);
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
            rotatedImage = imrotate(readImage(image_compressed), 180);
            targetImageHSV = rgb2hsv(rotatedImage);
            targetImageSonly = targetImageHSV(:,:,2);
            subplot(233); imshow(targetImageSonly)
            S_thresh = 0.3;
            HSV_Thresh_image = S_thresh>=targetImageSonly;
            [bw_Canny,threshOut] = edge(HSV_Thresh_image,'Canny', [0.00, 0.25],5); 
            subplot(234); imshow(rotatedImage)
            hold on
            params.minMajorAxis = 15;
            params.maxMajorAxis = 200;
            params.numBest = 1;
            params.rotation = 90;
            params.rotationSpan = 10;
        
            % note that the edge (or gradient) image is used
            bestFits = ellipseDetection(bw_Canny, params);
            ellipseX = bestFits(1,1); %column
            ellipseY = bestFits(1,2); %row
            ellipse(bestFits(1,3),bestFits(1,4),bestFits(1,5)*pi/180,bestFits(1,1),bestFits(1,2),'r');    
        end
    end
end