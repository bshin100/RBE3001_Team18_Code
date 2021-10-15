% RBE3001 - Laboratory 1 
% Joint Reading & Plotting Script

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

    % Joint Reading & Plotting Script

    % Send to Zero position with no interpolation
    pp.servo_jp([0 0 0]);
    pause(2);

    % Set base angle to 45 deg, with 4s interpolation
    tic; % Start stopwatch timer
    pp.interpolate_jp([45 0 0], 4000);

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
    pp.servo_jp([0 0 0]);
    
    timePos = [recordedTimes recordedPositions];
    writematrix(timePos, 'lab1data.csv'); % Export data to csv file

    % Plotting time!
    figure
    plot(recordedTimes, recordedPositions(:,1))
    hold on
    plot(recordedTimes, recordedPositions(:,2))
    plot(recordedTimes, recordedPositions(:,3))
    title('Motion Profile of Hephaestus Arm Joints')
    xlabel('Time Elapsed (ms)')
    ylabel('Joint Position (degrees)')
    legend('Joint 1', 'Joint 2', 'Joint 3')
    hold off
    
    figure
    histogram(timeDelta)
    xlim([0.5 1.5]) % Limit domain of histogram to better see distribution
    title('Incremental Timesteps Histogram')
    xlabel('Timesteps (ms)')
    ylabel('Frequency')
    
    % Calculate statistical values for time interval data
    disp(mean(timeDelta));
    disp(median(timeDelta));
    disp(max(timeDelta));
    disp(min(timeDelta));
  
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

toc
