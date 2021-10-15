% RBE3001 - Laboratory 1 
% Multi-axis joint-space control

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
try
    % Continuous readout of the servo positions
    %while true
    %    disp(pp.measured_js(true, false));
    %end
    
    pos1 = [-33.80 89.26 -76.60];
    pos2 = [0 29.50 45.20];
    pos3 = [0 69.34 -89.65];
    pos4 = [30.00 78.70 -6.13];
    
    pp.servo_jp([0 0 0]); % Send to Zero position
    pause(1);
    tic; % Start stopwatch timer
    %pp.interpolate_jp(pos4, 4000); % Move to position, 4s interpolation
    pp.servo_jp(pos4);
    
    % Continuously read current joint positions and timestamps
    recordedPositions = zeros(6000, 3);
    recordedTimes = zeros(6000, 1);
    timeDelta = zeros(6000, 1);
    cnt = 1;

    while (cnt < 6001) % Loop polling for duration of movement
        measurement = pp.measured_js(true, false); % Take measurements once
        timestamp = toc*1000;
        
        % Append to array variables
        recordedPositions(cnt,:) = recordedPositions(cnt,:) + measurement(1,:);
        recordedTimes(cnt) = recordedTimes(cnt) + timestamp;
        
        if (cnt > 1)
            delta = timestamp - recordedTimes(cnt-1);
            timeDelta(cnt) = timeDelta(cnt) + delta;
        else
            timeDelta(1) = recordedTimes(1); % Manually enter first delta
        end
        
        cnt = cnt + 1;
        %pause(0.01);
    end

    pause(2);
    pp.interpolate_jp([0 0 0], 2000);
    
    timePos = [recordedTimes recordedPositions];
    writematrix(timePos, 'lab1data_P5.csv'); % Export data to csv file

    % Plotting time!
    figure
    plot(recordedTimes, recordedPositions(:,1))
    hold on
    plot(recordedTimes, recordedPositions(:,2))
    plot(recordedTimes, recordedPositions(:,3))
    title('Motion Profile of Hephaestus Arm Joints, Position 4')
    xlabel('Time Elapsed (ms)')
    ylabel('Joint Position (degrees)')
    legend('Joint 1', 'Joint 2', 'Joint 3')
    hold off
  
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

toc
