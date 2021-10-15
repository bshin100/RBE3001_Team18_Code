% Auto-generated by cameraCalibrator app on 06-Oct-2021
%-------------------------------------------------------


% Define images to process
imageFileNames = {'/home/beshin/RBE3001_Matlab18/camera_calibration/Image5.png',...
    '/home/beshin/RBE3001_Matlab18/camera_calibration/Image6.png',...
    '/home/beshin/RBE3001_Matlab18/camera_calibration/Image9.png',...
    '/home/beshin/RBE3001_Matlab18/camera_calibration/Image10.png',...
    '/home/beshin/RBE3001_Matlab18/camera_calibration/Image11.png',...
    '/home/beshin/RBE3001_Matlab18/camera_calibration/Image23.png',...
    '/home/beshin/RBE3001_Matlab18/camera_calibration/Image26.png',...
    '/home/beshin/RBE3001_Matlab18/camera_calibration/Image27.png',...
    '/home/beshin/RBE3001_Matlab18/camera_calibration/Image30.png',...
    '/home/beshin/RBE3001_Matlab18/camera_calibration/Image31.png',...
    '/home/beshin/RBE3001_Matlab18/camera_calibration/Image37.png',...
    '/home/beshin/RBE3001_Matlab18/camera_calibration/Image64.png',...
    '/home/beshin/RBE3001_Matlab18/camera_calibration/Image65.png',...
    '/home/beshin/RBE3001_Matlab18/camera_calibration/Image68.png',...
    '/home/beshin/RBE3001_Matlab18/camera_calibration/Image69.png',...
    '/home/beshin/RBE3001_Matlab18/camera_calibration/Image70.png',...
    '/home/beshin/RBE3001_Matlab18/camera_calibration/Image71.png',...
    '/home/beshin/RBE3001_Matlab18/camera_calibration/Image72.png',...
    '/home/beshin/RBE3001_Matlab18/camera_calibration/Image73.png',...
    '/home/beshin/RBE3001_Matlab18/camera_calibration/Image75.png',...
    '/home/beshin/RBE3001_Matlab18/camera_calibration/Image76.png',...
    '/home/beshin/RBE3001_Matlab18/camera_calibration/Image77.png',...
    '/home/beshin/RBE3001_Matlab18/camera_calibration/Image79.png',...
    '/home/beshin/RBE3001_Matlab18/camera_calibration/Image80.png',...
    };
% Detect checkerboards in images
[imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imageFileNames);
imageFileNames = imageFileNames(imagesUsed);

% Read the first image to obtain image size
originalImage = imread(imageFileNames{1});
[mrows, ncols, ~] = size(originalImage);

% Generate world coordinates of the corners of the squares
squareSize = 25;  % in units of 'millimeters'
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Calibrate the camera using fisheye parameters
[cameraParams, imagesUsed, estimationErrors] = estimateFisheyeParameters(imagePoints, worldPoints, ...
    [mrows, ncols], ...
    'EstimateAlignment', false, ...
    'WorldUnits', 'millimeters');

% View reprojection errors
h1=figure; showReprojectionErrors(cameraParams);

% Visualize pattern locations
h2=figure; showExtrinsics(cameraParams, 'CameraCentric');

% Display parameter estimation errors
displayErrors(estimationErrors, cameraParams);

% For example, you can use the calibration data to remove effects of lens distortion.
undistortedImage = undistortFisheyeImage(originalImage, cameraParams.Intrinsics);

% See additional examples of how to use the calibration data.  At the prompt type:
% showdemo('MeasuringPlanarObjectsExample')
% showdemo('StructureFromMotionExample')
