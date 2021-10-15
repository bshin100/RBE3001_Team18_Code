% RBE 3001 - Lab 3
% Execution Code for Lab 3, Part 5-8 trajectory planning

clear
clear java
clear classes;

vid = hex2dec('16c0');
pid = hex2dec('0486');

disp (vid);
disp (pid);

javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

% Create a PacketProcessor object to send data to the nucleo firmware
pp = Robot(myHIDSimplePacketComs);
tp = Traj_Planner(pp);
gr = Graphing(pp);

try
    tic % Start stopwatch timer
  
    % Joint angles of each vertex on the triangle
    triPoint1 = [85; 80; -20]; 
    triPoint2 = [10; 40; -30];
    triPoint3 = [-30; 70; -10];
    trianglePoints = [triPoint1 triPoint2 triPoint3 triPoint1];
  
    % Task space coordinates of the triangle vertices
    triPos1 = [12.9410; 147.9158; 25.7623]; 
    triPos2 = [160.2869; 28.2629; 154.2396];
    triPos3 = [124.6810; -71.9846; 42.5995];
    triangle = [triPos1 triPos2 triPos3 triPos1];
    
    % Initialize some storage matrices
    jointAngles = [];
    trajectory = [];
    xyzVelocity = [];
    xyzAccel = [];
    recordedTimes = [];  
    
    intTime = 3.0;
    intPoints = 1000;
    loopTimer = zeros(5,1); 
    
    %pp.servo_jp([0,0,0]); % Send to home position
    %pause(1);
    
    %% Part 5: Cubic Trajectory Triangle
   
%     % Run the interpolation for every vertex 1 -> 2 -> 3 -> 1
%     for i = 1:4
%         loopTimer(i) = toc; % Begin timer
%         path = tp.cubic_calc(intTime, trianglePoints(:, i));
%         
%         % Run condition based on setpoint acheivement and a timeout
%         % failsafe.
%         while (toc-loopTimer(i) < intTime*1.1) && ...
%             (pp.checkInterpolationStatus(trianglePoints(:,i), true) ~= true)
%         
%             pp.interpolateJointsCubic(path, toc-loopTimer(i), intTime, intPoints); % Non-blocking
% 
%             % Poll for current values
%             jointReading = pp.measured_js(true, false); 
%             tipReading = pp.measured_cp();
%             timeReading = toc * 1000; % ms
% 
%             % Append to storage matrices
%             jointAngles(end+1,:) = jointReading(1, :);
%             trajectory(end+1,:) = transpose(tipReading(1:3, 4));
%             recordedTimes(end+1) = timeReading;
% 
%             % Derive velocity and acceleration (but make sure theres enough
%             % data).
%             if size(recordedTimes, 2) > 10
%                 instVelocity = pp.readDerivative(recordedTimes(end-10), ...
%                     timeReading, trajectory(end-10, :), ...
%                     transpose(tipReading(1:3,4)));
%                 xyzVelocity(end+1,:) = instVelocity;
%             end
% 
%             if size(xyzVelocity, 1) > 10
%                 instAccel = pp.readDerivative(recordedTimes(end-10), ...
%                     timeReading, instVelocity, xyzVelocity(end-10, :));
%                 xyzAccel(end+1,:) = instAccel;
%             end 
%         end
%         
%     end
%     
%     gr.plotTip(recordedTimes, trajectory);
%     gr.plotTip3D(trajectory);
%     gr.plotJoints(recordedTimes, jointAngles);
%     gr.plotKinematics(recordedTimes, trajectory, xyzVelocity, xyzAccel);

    %% Part 6: Linear Trajectory Triangle
    
%     % Define paths
%     pathArray = [];
%     pathArray(:,:,1) = tp.linear_traj(triPos1, triPos2, intTime, false);
%     pathArray(:,:,2) = tp.linear_traj(triPos2, triPos3, intTime, false);
%     pathArray(:,:,3) = tp.linear_traj(triPos3, triPos1, intTime, false);
%     
%     % Run the interpolation for every vertex 1 -> 2 -> 3 -> 1
%     for i = 1:3
%         loopTimer(i) = toc; % Begin timer
%         path = pathArray(:,:,i);
%         
%         % Run condition based on setpoint acheivement and a timeout
%         % failsafe.
%         while (toc-loopTimer(i) < intTime*1.1) && ...
%             (pp.checkInterpolationStatus(triangle(:,i+1), false) ~= true)
%         
%             pp.interpolateLinearTraj(path, toc-loopTimer(i), intTime, intPoints, false);
% 
%             % Poll for current values
%             jointReading = pp.measured_js(true, true);
%             jointVel = jointReading(2, :);
%             tipReading = pp.measured_cp();
%             timeReading = toc * 1000; % ms
% 
%             % Append to storage matrices
%             jointAngles(end+1,:) = jointReading(1, :);
%             trajectory(end+1,:) = transpose(tipReading(1:3, 4));
%             recordedTimes(end+1) = timeReading;
% 
%             % Derive velocity and acceleration (but make sure theres enough
%             % data).
%             if size(recordedTimes, 2) > 10
%                 instVelocity = pp.readDerivative(recordedTimes(end-10), ...
%                     timeReading, trajectory(end-10, :), ...
%                     transpose(tipReading(1:3,4)));
%                 xyzVelocity(end+1,:) = instVelocity;
%             end
% 
%             if size(xyzVelocity, 1) > 10
%                 instAccel = pp.readDerivative(recordedTimes(end-10), ...
%                     timeReading, instVelocity, xyzVelocity(end-10, :));
%                 xyzAccel(end+1,:) = instAccel;
%             end 
%         end
%         
%     end
%     
%     gr.plotTip(recordedTimes, trajectory);
%     gr.plotTip3D(trajectory);
%     gr.plotJoints(recordedTimes, jointAngles);
%     gr.plotKinematics(recordedTimes, trajectory, xyzVelocity, xyzAccel);

    %% Part 7: Quintic Trajectory in Task Space

    % Define paths
    pathArray = [];
    pathArray(:,:,1) = tp.linear_traj(triPos1, triPos2, intTime, true);
    pathArray(:,:,2) = tp.linear_traj(triPos2, triPos3, intTime, true);
    pathArray(:,:,3) = tp.linear_traj(triPos3, triPos1, intTime, true);
    
    % Run the interpolation for every vertex 1 -> 2 -> 3 -> 1
    for i = 1:3
        loopTimer(i) = toc; % Begin timer
        path = pathArray(:,:,i);
        
        % Run condition based on setpoint acheivement and a timeout
        % failsafe.
        while (toc-loopTimer(i) < intTime*1.1) && ...
            (pp.checkInterpolationStatus(triangle(:,i+1), false) ~= true)
        
            pp.interpolateLinearTraj(path, toc-loopTimer(i), intTime, intPoints, true);

            % Poll for current values
            jointReading = pp.measured_js(true, true);
            jointVel = jointReading(2, :);
            tipReading = pp.measured_cp();
            timeReading = toc * 1000; % ms

            % Append to storage matrices
            jointAngles(end+1,:) = jointReading(1, :);
            trajectory(end+1,:) = transpose(tipReading(1:3, 4));
            recordedTimes(end+1) = timeReading;

            % Derive velocity and acceleration (but make sure theres enough
            % data).
            if size(recordedTimes, 2) > 10
                instVelocity = pp.readDerivative(recordedTimes(end-10), ...
                    timeReading, trajectory(end-10, :), ...
                    transpose(tipReading(1:3,4)));
                xyzVelocity(end+1,:) = instVelocity;
            end

            if size(xyzVelocity, 1) > 10
                instAccel = pp.readDerivative(recordedTimes(end-10), ...
                    timeReading, instVelocity, xyzVelocity(end-10, :));
                xyzAccel(end+1,:) = instAccel;
            end 
        end
        
    end
    
    gr.plotTip(recordedTimes, trajectory);
    gr.plotTip3D(trajectory);
    gr.plotJoints(recordedTimes, jointAngles);
    gr.plotKinematics(recordedTimes, trajectory, xyzVelocity, xyzAccel);

    %% EC1: Interesting Trajectory - Banana
%     banL = [101; 121; 55];
%     banL1 = [122; 80; 105];
%     banMid = [126; 5; 122];
%     banR1 = [107; -41; 111];
%     banR = [82; -80; 77];
%     bananaTop = [banL banL1 banMid banR1 banR];
%     
%     banL_L = [101; 121; 15];
%     banL1_L = [122; 80; 75];
%     banMid_L = [126; 5; 82];
%     banR1_L = [107; -41; 71];
%     banR_L = [82; -80; 37];
%     bananaLower = [banL_L banL1_L banMid_L banR1_L banR_L];
%     
%     for i = 1:4
%         path = tp.linear_traj(bananaTop(:,i), bananaTop(:,i+1), intTime, true);
%         pp.interpolateLinearTraj(path, intTime, intPoints, true);
%     end
%     
%     for i = 4:-1:1
%         path = tp.linear_traj(bananaTop(:,i), bananaTop(:,i-1), intTime, true);
%         pp.interpolateLinearTraj(path, intTime, intPoints, true);
%     end
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

toc