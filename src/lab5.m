%%
% RBE 3001 Lab 5 example code!
% Developed by Alex Tacescu (https://alextac.com)
%%
clc;
clear;
clear java;
format short

%% Flags
DEBUG = false;
STICKMODEL = false;
DEBUG_CAM = false;

SETUP_CAM = false;

%% Setup
vid = hex2dec('16c0');
pid = hex2dec('0486');

if DEBUG
    disp(vid);
    disp(pid);
end

javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java;
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

robot = Robot(myHIDSimplePacketComs);
tp = Traj_Planner(robot);

if SETUP_CAM
    cam = Camera();
    cam.DEBUG = DEBUG_CAM;
    save('camera_var.mat', 'cam');
else
    load('camera_var.mat');
end

%% Place Poses per color (Task Space)
red_place =     [100; 150; 45];
green_place =   [100; -150; 45];
orange_place =  [0; 150; 45];
yellow_place =  [0; -150; 45];

%% Main Loop
try
    % Set up camera
%     if cam.params == 0
%         error("No camera parameters found!");
%     end
%     cam.cam_pose = cam.getCameraPose();
    
%     targetPxPoint = [618, 337];
%     cam.convertPxToRobot(targetPxPoint)

    disp('Calibration complete, please place balls and press any key');
    pause;
    
    % Detect any balls on the checkerboard
    ballData = cam.detectBalls();
    ballColors = ballData(:, 1); % Colors of the balls (numerical indicies)
    ballCents = ballData(:, 2:3); % Centroids of the balls (px coordinates)
    
    ballPosition = [];
    for i = 1:length(ballColors)
        ballPosition(end+1, :) = cam.ballCoordinate(ballCents(i, :));
    end
    
    % Pick up a yellow ball
    [targRow, targCol] = find(ballColors == 3);
    yellowInitial = ballPosition(targRow, 1:3); % Get the 1x3 position vector
    yellowInitial = transpose(yellowInitial); % Format as 3x1
    yellowInitAbove = yellowInitial;
    yellowInitAbove(3) = yellowInitAbove(3) + 10; % Add 10mm above ball
    
    intTime = 2.0;
    intPoints = 1000;
    loopTimer = zeros(5,1); 
    
    % Define paths
    neutralPos = [86.6025; 0; 245]; % Just a starting point (joints: 0,0,-30)
    
    pathArray = [];
    pathArray(:,:,1) = tp.linear_traj(neutralPos, yellowInitAbove, intTime, true);
    pathArray(:,:,2) = tp.linear_traj(yellowInitAbove, yellowInitial, intTime, true);
    pathArray(:,:,3) = tp.linear_traj(yellowInitial, neutralPos, intTime, true);
    pathArray(:,:,4) = tp.linear_traj(neutralPos, yellow_place, intTime, true);
    
    robot.openGripper();
    robot.interpolate_jp([0, 0, -30], 1500);
    pause(2);
    tic;
    
    loopTimer(1) = toc;
    path = pathArray(:,:,1);
    while (toc-loopTimer(1) < intTime*1.1)
        robot.interpolateLinearTraj(path, toc-loopTimer(1), intTime, intPoints, true);
    end
     
    loopTimer(2) = toc;
    path = pathArray(:,:,2);
    while (toc-loopTimer(2) < intTime*1.1)
        robot.interpolateLinearTraj(path, toc-loopTimer(2), intTime, intPoints, true);
    end
    
    robot.closeGripper();
    pause(0.5);
    
    loopTimer(3) = toc;
    path = pathArray(:,:,3);
    while (toc-loopTimer(3) < intTime*1.1)
        robot.interpolateLinearTraj(path, toc-loopTimer(3), intTime, intPoints, true);
    end
    
    loopTimer(4) = toc;
    path = pathArray(:,:,4);
    while (toc-loopTimer(4) < intTime*1.1)
        robot.interpolateLinearTraj(path, toc-loopTimer(4), intTime, intPoints, true);
    end
    
    robot.openGripper();
    pause(0.5);
    robot.interpolate_jp([0, 0, -30], 1500);
    pause(2);
    
    
    % Pick up a green ball
    [targRow, targCol] = find(ballColors == 2);
    greenInitial = ballPosition(targRow, 1:3); % Get the 1x3 position vector
    greenInitial = transpose(greenInitial); % Format as 3x1
    greenInitAbove = greenInitial;
    greenInitAbove(3) = greenInitAbove(3) + 10; % Add 10mm above ball
    
    pathArray(:,:,1) = tp.linear_traj(neutralPos, greenInitAbove, intTime, true);
    pathArray(:,:,2) = tp.linear_traj(greenInitAbove, greenInitial, intTime, true);
    pathArray(:,:,3) = tp.linear_traj(greenInitial, neutralPos, intTime, true);
    pathArray(:,:,4) = tp.linear_traj(neutralPos, green_place, intTime, true);
    
    
    loopTimer(1) = toc;
    path = pathArray(:,:,1);
    while (toc-loopTimer(1) < intTime*1.1)
        robot.interpolateLinearTraj(path, toc-loopTimer(1), intTime, intPoints, true);
    end
     
    loopTimer(2) = toc;
    path = pathArray(:,:,2);
    while (toc-loopTimer(2) < intTime*1.1)
        robot.interpolateLinearTraj(path, toc-loopTimer(2), intTime, intPoints, true);
    end
    
    robot.closeGripper();
    pause(0.5);
    
    loopTimer(3) = toc;
    path = pathArray(:,:,3);
    while (toc-loopTimer(3) < intTime*1.1)
        robot.interpolateLinearTraj(path, toc-loopTimer(3), intTime, intPoints, true);
    end
    
    loopTimer(4) = toc;
    path = pathArray(:,:,4);
    while (toc-loopTimer(4) < intTime*1.1)
        robot.interpolateLinearTraj(path, toc-loopTimer(4), intTime, intPoints, true);
    end
    
    robot.openGripper();
    pause(0.5);
    robot.interpolate_jp([0, 0, -30], 1500);
    pause(2);
    
    
    
    % Pick up a orange ball
    [targRow, targCol] = find(ballColors == 4);
    orangeInitial = ballPosition(targRow, 1:3); % Get the 1x3 position vector
    orangeInitial = transpose(orangeInitial); % Format as 3x1
    orangeInitAbove = orangeInitial;
    orangeInitAbove(3) = orangeInitAbove(3) + 10; % Add 10mm above ball
    
    pathArray(:,:,1) = tp.linear_traj(neutralPos, orangeInitAbove, intTime, true);
    pathArray(:,:,2) = tp.linear_traj(orangeInitAbove, orangeInitial, intTime, true);
    pathArray(:,:,3) = tp.linear_traj(orangeInitial, neutralPos, intTime, true);
    pathArray(:,:,4) = tp.linear_traj(neutralPos, orange_place, intTime, true);
    
    
    loopTimer(1) = toc;
    path = pathArray(:,:,1);
    while (toc-loopTimer(1) < intTime*1.1)
        robot.interpolateLinearTraj(path, toc-loopTimer(1), intTime, intPoints, true);
    end
     
    loopTimer(2) = toc;
    path = pathArray(:,:,2);
    while (toc-loopTimer(2) < intTime*1.1)
        robot.interpolateLinearTraj(path, toc-loopTimer(2), intTime, intPoints, true);
    end
    
    robot.closeGripper();
    pause(0.5);
    
    loopTimer(3) = toc;
    path = pathArray(:,:,3);
    while (toc-loopTimer(3) < intTime*1.1)
        robot.interpolateLinearTraj(path, toc-loopTimer(3), intTime, intPoints, true);
    end
    
    loopTimer(4) = toc;
    path = pathArray(:,:,4);
    while (toc-loopTimer(4) < intTime*1.1)
        robot.interpolateLinearTraj(path, toc-loopTimer(4), intTime, intPoints, true);
    end
    
    robot.openGripper();
    pause(0.5);
    robot.interpolate_jp([0, 0, -30], 1500);
    pause(2);
    
    
    
    % Pick up a red ball
    [targRow, targCol] = find(ballColors == 1);
    redInitial = ballPosition(targRow, 1:3); % Get the 1x3 position vector
    redInitial = transpose(redInitial); % Format as 3x1
    redInitAbove = redInitial;
    redInitAbove(3) = redInitAbove(3) + 10; % Add 10mm above ball
    
    pathArray(:,:,1) = tp.linear_traj(neutralPos, redInitAbove, intTime, true);
    pathArray(:,:,2) = tp.linear_traj(redInitAbove, redInitial, intTime, true);
    pathArray(:,:,3) = tp.linear_traj(redInitial, neutralPos, intTime, true);
    pathArray(:,:,4) = tp.linear_traj(neutralPos, red_place, intTime, true);
    
    
    loopTimer(1) = toc;
    path = pathArray(:,:,1);
    while (toc-loopTimer(1) < intTime*1.1)
        robot.interpolateLinearTraj(path, toc-loopTimer(1), intTime, intPoints, true);
    end
     
    loopTimer(2) = toc;
    path = pathArray(:,:,2);
    while (toc-loopTimer(2) < intTime*1.1)
        robot.interpolateLinearTraj(path, toc-loopTimer(2), intTime, intPoints, true);
    end
    
    robot.closeGripper();
    pause(0.5);
    
    loopTimer(3) = toc;
    path = pathArray(:,:,3);
    while (toc-loopTimer(3) < intTime*1.1)
        robot.interpolateLinearTraj(path, toc-loopTimer(3), intTime, intPoints, true);
    end
    
    loopTimer(4) = toc;
    path = pathArray(:,:,4);
    while (toc-loopTimer(4) < intTime*1.1)
        robot.interpolateLinearTraj(path, toc-loopTimer(4), intTime, intPoints, true);
    end
    
    robot.openGripper();
    pause(0.5);
    robot.interpolate_jp([0, 0, -30], 1500);
    pause(2);
    
    pause(3);
    
catch exception
    fprintf('\n ERROR!!! \n \n');
    disp(getReport(exception));
    disp('Exited on error, clean shutdown');
end

%% Shutdown Procedure
robot.shutdown()
%cam.shutdown()
