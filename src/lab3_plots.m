% RBE 3001 - Lab 3
% Plotting code for part 3 of lab 3.

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
graph = Graphing(pp);

jointPosData = readmatrix('../Data/Lab3/lab3_data.csv');
timeData = jointPosData(:, 1);
joint1Data = jointPosData(:, 2);
joint2Data = jointPosData(:, 3);
joint3Data = jointPosData(:, 4);
tipX = jointPosData(:, 5);
tipY = jointPosData(:, 6);
tipZ = jointPosData(:, 7);

try
    tic % Start stopwatch timer
    
    graph.plotJoints(timeData, [joint1Data joint2Data joint3Data]);
    graph.plotTip(timeData, [tipX tipY tipZ]);
    
    % 3D plot for the path of the arm in task space
    figure
    plot3(tipX, tipY, tipZ)
    hold on
    title('Tip Path in Task Space')
    xlabel('X Axis')
    ylabel('Y Axis')
    zlabel('Z Axis')
    hold off    
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

toc