classdef Camera < handle
    % CAMERA Example Camera class for RBE 3001 Lab 5
    %   You can add your image processing in this camera class,
    %   as well as any other functions related to the camera.
    
    properties
        % Flags
        DEBUG = false;
        POSE_PLOT = false;  
        DEBUG_BALLDETECTION = false;
        
        % Properties
        params;
        cam;
        cam_pose;
        cam_imajl;
        imgMask;
    end
    
    methods
        function self = Camera()
            % CAMERA Construct an instance of this class
            self.cam = webcam(2); % Get camera object
            self.params = self.calibrate(); % Run Calibration Function
            [self.cam_imajl, self.cam_pose] = self.getCameraPose();
            self.imgMask = ImageMask();
        end

        function shutdown(self)
            % SHUTDOWN shutdown script which clears camera variable
            clear self.cam;
        end
      
        function params = calibrate(self)
            % CALIBRATE Calibration function
            % This function will run the camera calibration, save the camera parameters,
            % and check to make sure calibration worked as expected
            % The calibrate function will ask if you are ready. To calibrate, you must press
            % any key, then the system will confirm if the calibration is successful

            % NOTE: This uses the camcalib.m file for camera calibration. If you have placed
            % your camera calibration script elsewhere, you will need to change the command below

            DEBUG = self.DEBUG;
            params = 0;
            try
                disp("Clear surface of any items, then press any key to continue");
                pause;
                camcalib; % Change this if you are using a different calibration script
                params = cameraParams;
                disp("This is REEE Frogg:")
                disp("Camera calibration complete!");
            catch exception
                msg = getReport(exception);
                disp(msg)
                disp("No camera calibration file found. Plese run camera calibration");
            end          
        end
        
        % Returns an undistorted camera image
        function img = getImage(self)
            raw_img =  snapshot(self.cam);
            [img, new_origin] = undistortFisheyeImage(raw_img, self.params.Intrinsics, 'OutputView', 'full');
        end

        
        function [newIs, pose] = getCameraPose(self)
            % GETCAMERAPOSE Get transformation from camera to checkerboard frame
            % This function will get the camera position based on checkerboard.
            % You should run this function every time the camera position is changed.
            % It will calculate the extrinsics, and output to a transformation matrix.
            % Keep in mind: this transformation matrix is a transformation from pixels
            % to x-y coordinates in the checkerboard frame!

            % There are a few debugging options included as well! Simply set POSE_PLOT
            % to true to show the checkerboard frame of reference on the picture!

            % 1. Capture image from camera
            raw_img =  snapshot(self.cam);
            % 2. Undistort Image based on params
            [img, newIs] = undistortFisheyeImage(raw_img, self.params.Intrinsics, 'OutputView', 'full');
            % 3. Detect checkerboard in the image
            [imagePoints, boardSize] = detectCheckerboardPoints(img);
            % 4. Compute transformation
            [R, t] = extrinsics(imagePoints, self.params.WorldPoints, newIs);
            
            pose = [   R,    t';
                    0, 0, 0, 1];
                
            if self.POSE_PLOT
                
                figure(10)
                newImgPoints = [];
                c = 1;
                for i = 0:25:175
                    for j = 0:25:125
                        newImgPoints(c,1) = i;
                        newImgPoints(c,2) = j;
                        newImgPoints(c,3) = 0;
                        c = c + 1;
                    end
                end
                
                newGridPoints = worldToImage(newIs, R, t, newImgPoints);

                axesPoints = worldToImage(newIs, R, t, [0 0 0; 0 50 0; 50 0 0]);

                x1 = [axesPoints(1, 1), axesPoints(2, 1)]';
                y1 = [axesPoints(1, 2), axesPoints(2, 2)]';
                
                img = insertText(img, [x1(2), y1(2)], 'Y Axis', 'TextColor', 'green', 'FontSize', 18);
                x2 = [axesPoints(1, 1), axesPoints(3, 1)]';
                y2 = [axesPoints(1, 2), axesPoints(3, 2)]';
                
                img = insertText(img, axesPoints(3, 1:2), 'X Axis', 'TextColor', 'red', 'FontSize', 18);
                hold on 
                imshow(img)
                title('Undistorted Image with checkerboard axes');
                viscircles(newGridPoints, ones(length(newGridPoints),1)*5);
                hold off
                
                line(x1, y1, 'linewidth', 5, 'Color', 'green');
                line(x2, y2, 'linewidth', 5, 'Color', 'red');
                
            end     
        end
        
        % Convert a pixel coordinate [x, y] into a position relative to the
        % base frame (F0) on the robot. Returns a 4x1 vector, where the
        % first three elements correspond to the X, Y, and Z coordinates in
        % millimeters. Note that this only works on bare checkerboard, use
        % the `ballCoordinates()` function for use with the balls.
        function P = convertPxToRobot(self, pixelXY)
            % Transformation from robot base frame to checkerboard origin
            T0_Checker = [0, 1,  0,   50; 
                          1, 0,  0, -100; 
                          0, 0, -1,    0; 
                          0, 0,  0,    1];
                      
            checkerPoints = pointsToWorld(self.params.Intrinsics, ...
                self.cam_pose(1:3,1:3), self.cam_pose(1:3,4), pixelXY);
            
            % Format as position "vector" (last column in HT matrix)
            checkerPos = [checkerPoints(1); checkerPoints(2); 0; 1];
            
            P = T0_Checker * checkerPos;
        end
        
        % Takes a snapshot of the workspace, manipulates the image, and
        % detects the balls on the checkerboard. Returns the color index 
        % (see the helper function `detectBallColor`)
        % and the centroid position of each ball (pixel coordinates).
        function B = detectBalls(self)
            raw_img = snapshot(self.cam); % Take a snapshot
            hsv_img = rgb2hsv(raw_img); % Convert RGB to HSV
            blur_img = imgaussfilt(raw_img, 1.0);
            
            [BW, RGBMaskedImg] = self.imgMask.createMask(blur_img);
            %figure; imshowpair(raw_img, BW, 'montage');
            
            % Detect circles within a certain radius range
            [centers, radii] = imfindcircles(BW, [20, 45]);
            %viscircles(centers, radii, 'Color', 'b');
            
            % Detect color of each ball
            ballColors = [];
            colorNames = [];
            for i = 1:length(centers)
                colorIndex = self.detectBallColor(hsv_img, round(centers(i, :)));
                ballColors(end+1,:) = colorIndex;
                %colorNames(end+1,:) = self.getColorName(colorIndex);
            end
            
            % Display a labeled image
            labeled_BW = insertObjectAnnotation(raw_img, 'circle', ...
                [centers radii], ballColors, 'LineWidth', 3);
            figure; imshow(labeled_BW);
            
            B = [ballColors, centers, radii]
        end
        
        % Helper function to detect the color of a ball at a certain
        % position (in pixels) for the red, green, yellow, and orange balls
        % provided. Takes in an HSV image and the *integer* pixel position
        % (preferably the centroid). Looks at the pixel and its surrounding
        % neighbors. Returns the color indexed in the same order mentioned
        % (i.e 1 = red) and 0 if it isn't one of the four colors.
        function C = detectBallColor(self, HSV, pxPos)
            % Separate HSV image into respective channels
            imgH = HSV(:,:,1); imgS = HSV(:,:,2); imgV = HSV(:,:,3);
            
            % Define Hue thresholds for each ball color, scaled to [0,1]
            % TODO: add S and V constraints if needed
            redH_Bounds = [340/360, 355/360];
            greenH_Bounds = [60/360, 75/360];
            yellowH_Bounds = [45/360, 55/360];
            orangeH_Bounds = [20/360, 35/360];
            
            % Take a 3x3 array of pixels
            % Note: MATLAB indexes the channel arrays backwards (y, x). Dammit.
            patch = [imgH(pxPos(2)-1,pxPos(1)-1), imgH(pxPos(2)-1,pxPos(1)), imgH(pxPos(2)-1,pxPos(1)+1);
                       imgH(pxPos(2),pxPos(1)-1),   imgH(pxPos(2),pxPos(1)),   imgH(pxPos(2),pxPos(1)+1);
                     imgH(pxPos(2)+1,pxPos(1)-1), imgH(pxPos(2)+1,pxPos(1)), imgH(pxPos(2)+1,pxPos(1)+1)];
            
            % Iterate through pixel patch and check for each color
            rsum = 0; gsum = 0; ysum = 0; osum = 0;
            for i = 1:9
                if patch(i) >= redH_Bounds(1) && patch(i) <= redH_Bounds(2)
                    rsum = rsum + 1;
                end
                if patch(i) >= greenH_Bounds(1) && patch(i) <= greenH_Bounds(2)
                    gsum = gsum + 1;
                end
                if patch(i) >= yellowH_Bounds(1) && patch(i) <= yellowH_Bounds(2)
                    ysum = ysum + 1;
                end
                if patch(i) >= orangeH_Bounds(1) && patch(i) <= orangeH_Bounds(2)
                    osum = osum + 1;
                end
            end
            
            C = 0;
            
            % Reject the analysis if < 60% of the pixels are not red.
            ravg = rsum / 9; 
            if ravg > 0.6
                C = 1;
            end
            
            % Reject the analysis if < 60% of the pixels are not green.
            gavg = gsum / 9; 
            if gavg > 0.6
                C = 2;
            end
            
            % Reject the analysis if < 60% of the pixels are not yellow.
            yavg = ysum / 9; 
            if yavg > 0.6
                C = 3;
            end
            
            % Reject the analysis if < 60% of the pixels are not orange.
            oavg = osum / 9; 
            if oavg > 0.6
                C = 4;
            end

        end
        
        % Helper function to get the name of the colors detected by the
        % image processor. Returns the string name.
        % 1 = "red"; 2 = "green"; 3 = "yellow"; 4 = "orange"; 0 = "N/A"
        function S = getColorName(self, colorIndex)
            names = {'red', 'green', 'yellow', 'orange', 'N/A'};
            switch colorIndex
                case 0
                    S = names{5};
                case 1
                    S = names{1};
                case 2
                    S = names{2};
                case 3
                    S = names{3};
                case 4
                    S = names{4};
                otherwise
                    S = names{5};
            end
        end
        
        % Returns a 4x1 vector, where the first three elements correspond 
        % to the X, Y, and Z coordinates in millimeters. This position is
        % relative to the base frame of the robot (F0).
        function P = ballCoordinate(self, imagePoints)
            %imagePoints, specified as an 1-by-2 matrix containing [x, y]
            %coordinates of image point.
            
            % Transformation from robot base frame to checkerboard origin
            T0_Checker = [0, 1,  0,   50; 
                          1, 0,  0, -100; 
                          0, 0, -1,    0; 
                          0, 0,  0,    1];
            
            cVector = [100; 150; 259]; %Camera translation vector with respect to 0,0,0
            %cVector needs to be verified.
            r = 12.5; % Ball Radius in mm (old: 11.6)
            
            virtualPoint = pointsToWorld(self.params.Intrinsics, ...
                self.cam_pose(1:3,1:3), self.cam_pose(1:3,4), imagePoints);
            
            % (virtualPoint - worldPoint)/ActualPoint = ballHeight / cameraHeight
            % ActualPoint = (virtualPoint - worldPoint)/ (ballHeight / cameraHeight)
            % because similar triangles
            vp = transpose([virtualPoint(1,:), 0]);
            ballToCamera = cVector - vp;
            
            % Coordinates relative to the origin on the checkerboard,
            % adjusted for the projection error.
            worldPoints = [(ballToCamera(1)/ballToCamera(3)*r + vp(1)); 
                           (ballToCamera(2)/ballToCamera(3)*r + vp(2)); 
                           -r; 1];
                       
            P = T0_Checker * worldPoints;
            
        end
    end
end
