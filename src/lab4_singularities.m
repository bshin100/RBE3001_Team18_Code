% RBE 3001 - Lab 4
% Execution Code for Lab 4 Part 6

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

try
    tic % Start stopwatch timer
    
    triPos1 = [85 80 -20];
    triPoint1 = [12.9410; 147.9158; 25.7623];
    singlePos = [0; 0; -90]; % Singularity position (arm all colinear upwards)
    singlePos2 = [0; 90; -90]; % Another singularity position
    
    singularityFK = pp.fk3001(singlePos);
    singlePoint = singularityFK(1:3, 4);
    
    intTime = 5;
    intPoints = 500;
    
    pp.servo_jp(triPos1);
    pause(1);
    
    path = tp.linear_traj(triPoint1, singlePoint, intTime, false);
    pp.interpolateLinearTraj(path, intTime, intPoints, false);

catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

toc