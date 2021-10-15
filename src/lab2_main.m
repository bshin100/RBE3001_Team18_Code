% RBE3001 - Laboratory 2
% Executable code for 3D plotting of the arm (parts 5-7)
% Moves the arm to 5 arbitrary points and updates a 3D plot representation
% of the robot arm in real-time.

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
    
    zeroPos = [0 0 0];
    caliPos = [-89 87.1 33.5];
    randPos1 = [40 60 -10];
    randPos2 = [10 30 10];
    
    index = 1;
    setpoints = [randPos1; randPos2; [69 30 22]; [49 18 29]; [32 77 10]];
    
    pp.interpolate_jp(zeroPos, 1000);
    pause(1)
    
    % This is quick and dirty code for part 7, sign-off 4
    tic
    pp.interpolate_jp(setpoints(1, :), 1000);
    while toc < 2    
        clf;
        measurement = pp.measured_js(true,false);
        measuredJoints = transpose(measurement(1,:));
        passiveModeller.plot_arm(measuredJoints);
        pause(0.1);
    end
    
    pp.interpolate_jp(setpoints(2, :), 1000);
    while toc < 4    
        clf;
        measurement = pp.measured_js(true,false);
        measuredJoints = transpose(measurement(1,:));
        passiveModeller.plot_arm(measuredJoints);
        pause(0.1);
    end
    
    pp.interpolate_jp(setpoints(3, :), 1000);
    while toc < 6    
        clf;
        measurement = pp.measured_js(true,false);
        measuredJoints = transpose(measurement(1,:));
        passiveModeller.plot_arm(measuredJoints);
        pause(0.1);
    end
    
    pp.interpolate_jp(setpoints(4, :), 1000);
    while toc < 8    
        clf;
        measurement = pp.measured_js(true,false);
        measuredJoints = transpose(measurement(1,:));
        passiveModeller.plot_arm(measuredJoints);
        pause(0.1);
    end
    
    pp.interpolate_jp(setpoints(5, :), 1000);
    while toc < 10    
        clf;
        measurement = pp.measured_js(true,false);
        measuredJoints = transpose(measurement(1,:));
        passiveModeller.plot_arm(measuredJoints);
        pause(0.1);
    end
    
    disp('Joint Setpoints');
    disp(pp.setpoint_js());
    disp('Setpoint FK');
    disp(pp.setpoint_cp());
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

pp.shutdown()

toc