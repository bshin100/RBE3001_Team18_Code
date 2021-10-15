% RBE 3001 - Lab 4
% Execution Code for Lab 4

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
model = Model();

try
    tic % Start stopwatch timer
    
    singlePos = [0; 0; -90]; % Singularity position (arm all colinear upwards)
    singlePos2 = [0; 90; -90]; % Another singularity position

    % Task space coordinates of the triangle vertices
    triPos1 = [12.9410; 147.9158; 25.7623];
    triPos2 = [160.2869; 28.2629; 154.2396];
    triPos3 = [124.6810; -71.9846; 42.5995];
    triangle = [triPos1 triPos2 triPos3 triPos1];
    
    % Initialize some storage matrices
    jointAngles = [];
    jointVelocity = [];
    trajectory = [];
    recordedTimes = [];
    jacobianVels = [];
    magXYZVel = [];
    jacobiandets = [];
    
    intTime = 3.0;
    intPoints = 1000;
    loopTimer = zeros(5,1); 
    
    % Define paths
    pathArray = [];
    pathArray(:,:,1) = tp.linear_traj(triPos1, triPos2, intTime, false);
    pathArray(:,:,2) = tp.linear_traj(triPos2, triPos3, intTime, false);
    pathArray(:,:,3) = tp.linear_traj(triPos3, triPos1, intTime, false);
    
    %pp.servo_jp([85, 80, -20]) % Send to first vertex
    %pause(1);
    
    % Run the interpolation for every vertex 1 -> 2 -> 3 -> 1
    for i = 1:3
        loopTimer(i) = toc; % Begin timer
        path = pathArray(:,:,i);
        
        % Run condition based on setpoint acheivement and a timeout
        % failsafe.
        while (toc-loopTimer(i) < intTime*1.1) && ...
            (pp.checkInterpolationStatus(triangle(:,i+1), false) ~= true)
        
            pp.interpolateLinearTraj(path, toc-loopTimer(i), intTime, intPoints, false);

            % Poll for current values
            jointReading = pp.measured_js(true, true);
            jointVel = jointReading(2, :);
            tipReading = pp.measured_cp();
            timeReading = toc * 1000; % ms
            eeVel = pp.fdk3001(jointReading(1, :), jointVel.');
            magVel = sqrt(eeVel(1)^2 + eeVel(2)^2 + eeVel(3)^2);
            J = pp.jacob3001(jointReading(1, :));
            Jdet = det(J(1:3,:));

            % Append to storage matrices
            jointAngles(end+1,:) = jointReading(1, :);
            jointVelocity(end+1,:) = jointVel;
            trajectory(end+1,:) = transpose(tipReading(1:3, 4));
            recordedTimes(end+1) = timeReading;
            jacobianVels(end+1,:) = eeVel.';
            magXYZVel(end+1) = magVel;
            jacobiandets(end+1) = Jdet;
            
            % Plot 3D Stick Robot (disable for velocity plotting)
            %model.plot_arm(jointAngles(end, :), eeVel(1:3, :));
        end
        
    end
    
    gr.plotJacobVelocities(recordedTimes, jacobianVels, magXYZVel);
    
    % Subplot for part 6   
%     tipX = trajectory(:, 1);
%     tipY = trajectory(:, 2);
%     tipZ = trajectory(:, 3);
%     
%     figure
%     subplot(2,1,1)
%     view(3) % Force hold into 3D
%     hold on
%     grid on
%     plot3(tipX, tipY, tipZ)
%     title('Tip Path in Task Space')
%     xlabel('X Axis')
%     ylabel('Y Axis')
%     zlabel('Z Axis')
%     %hold off
%     colororder([0 0.4470 0.7410; 0.8500 0.3250 0.0980; 0.9290 0.6940 0.1250])
% 
%     subplot(2,1,2)
%     hold on
%     plot(recordedTimes, jacobiandets)
%     title('Determinant of Jacobian vs. Time')
%     xlabel('Time (ms)')
%     ylabel('Determinant Value')
%     hold off
%     colororder([0 0.4470 0.7410; 0.8500 0.3250 0.0980; 0.9290 0.6940 0.1250])
  
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

toc