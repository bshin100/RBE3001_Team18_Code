% RBE 3001 - Lab 2
% Execution Code for the first four parts of lab 2

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
  tic % Start stopwatch timer
  
  zeroPos = [0 0 0];
  testPos2 = [40 60 -10];
  
  T_Stored = zeros(4,4,10); % Empty 3D array to store each T matrix
  
  for i = 1:10      % Repeat motion 10 times
  
      pp.servo_jp(zeroPos); % Start/Reset at Zero Position
      pause(1);
      pp.interpolate_jp(testPos2, 1000); % Interpolate to arbitrary position
      pause(1.5);
      pp.interpolate_jp(zeroPos, 2000); % Return to Zero Position
      pause(2.5);    % Await completion

      T_Stored(:,:,i) = pp.measured_cp(); % Store 4x4 HT matrix
  end
  
  % Plot
  ax = axes;
  % Define color order for the plots
  ax.ColorOrder = [1 0 0; 0 1 0; 0 0 1; 0 1 1; 1 0 1; 1 1 0; 0 0 0; ... 
      0.850 0.325 0.098; 0.929 0.694 0.125; 0.494 0.184 0.556];
  
  figure
  plot3(T_Stored(1,4,1), T_Stored(2,4,1), T_Stored(3,4,1), '.')
  hold on
  for j = 2:10 % Iterate through each page of the matrix and plot vector P
    plot3(T_Stored(1,4,j), T_Stored(2,4,j), T_Stored(3,4,j), '.')
    disp('meep');
  end
  
  legend('Trial 1','Trial 2','Trial 3','Trial 4','Trial 5', ...
      'Trial 6','Trial 7','Trial 8','Trial 9','Trial 10')
  xlabel('X Position w.r.t Base')
  ylabel('Y Position w.r.t Base')
  zlabel('Z Position w.r.t Base')
  title('Home Position Repeated 10 Times') 
  hold off
  
  targPos = [100.0; 0.0; 195.0];
  pVSum = zeros(3,1);
  deltaSqSum = 0;
  for k = 1:10
      pVector = [T_Stored(1,4,k); T_Stored(2,4,k); T_Stored(3,4,k)];
      pVSum = pVSum + pVector;
      
      delta = sqrt((pVector(1)-targPos(1))^2 + (pVector(2)-targPos(2))^2 + (pVector(3)-targPos(3))^2);
      deltaSqSum = deltaSqSum + delta^2;
  end
  pVSum = pVSum / 10;
  disp('Average Tip Position: ');
  disp(pVSum);
  
  RMS = sqrt(deltaSqSum/10);
  disp('Root Mean Square: ');
  disp(RMS);
  
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

toc