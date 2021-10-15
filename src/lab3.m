% RBE 3001 - Lab 3
% Execution Code for Lab 3 Part 3

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
model = Model(pp);

try
    tic % Start stopwatch timer
  
    %triPoint1 = [85 80 -20];
    %triPoint2 = [10 40 -30];
    %triPoint3 = [-30 70 -10];
  
    triPos1 = [12.9410; 147.9158; 25.7623];
    triPos2 = [160.2869; 28.2629; 154.2396];
    triPos3 = [124.6810; -71.9846; 42.5995];
    triangle = [triPos1 triPos2 triPos3 triPos1];
    
    loopTimer = zeros(5,1); 
    recordedTimes = zeros(700, 1); % Store timestamps over duration
    jointAngles = zeros(700, 3); % Store recorded joint angles (theta1, theta2, theta3)
    trajectory = zeros(700, 3); % Store recorded tip position (x, y, z) relative to base
    
    j = 1;
    for i = 1:4
        calcJoint = pp.ik3001(triangle(:, i)); % Generate joint angles from XYZ
        
        loopTimer(i) = toc; % Begin timer
        pp.interpolate_jp(calcJoint.', 2000);
        
        while toc-loopTimer(i) < 2.0
             jointReading = pp.measured_js(true, false); % Poll for current values
             tipReading = pp.measured_cp();
             timeReading = toc * 1000; % ms

             jointAngles(j,:) = jointAngles(j,:) + jointReading(1, :); % Append to storage matrices
             trajectory(j,:) = trajectory(j,:) + transpose(tipReading(1:3, 4));
             recordedTimes(j) = recordedTimes(j) + timeReading;
             
             j = j+1;
             pause(0.01); % 10ms interval
        end
    end
    
    % Store recorded data in a CSV, where the first column is time, next 3 columns are joint values, and the next 3 are the tip position.
    jointsAndPositions = [recordedTimes jointAngles trajectory]; % Horizontal concat
    writematrix(jointsAndPositions, 'lab3_data.csv');
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

toc