% RBE3001 - Laboratory 2
% Executable code for motion planning in joint space (part 8)

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
passiveModeller = Model(pp);
try
    tic

    % Move arm in a planar triangle with the base joint at 0 and record joint angles and
    % trajectory into a CSV log file.
    
    trianglePoints = [0 0 0; 0 45 45; 0 65 -30; 0 0 0];
    loopTimer = zeros(5,1); 

    recordedTimes = zeros(500, 1); % Store timestamps over duration
    jointAngles = zeros(500, 3); % Store recorded joint angles (theta1, theta2, theta3)
    trajectory = zeros(500, 3); % Store recorded tip position (x, y, z) relative to base
    
    j = 1;
    
    for i = 1:4
        loopTimer(i) = toc; % Begin timer
        pp.interpolate_jp(trianglePoints(i,:), 2000);
        
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
    writematrix(jointsAndPositions, 'lab2_joint-pos_data.csv');

catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

pp.shutdown()

%toc