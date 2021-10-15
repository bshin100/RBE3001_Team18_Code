% RBE 3001 - Lab 2
% Plotting code for the motion planning part of lab 2 (part 8)

jointPosData = readmatrix('../Data/Lab2/lab2_joint-pos_data.csv');

timeData = jointPosData(:, 1);
joint1Data = jointPosData(:, 2);
joint2Data = jointPosData(:, 3);
joint3Data = jointPosData(:, 4);
tipX = jointPosData(:, 5);
tipY = jointPosData(:, 6);
tipZ = jointPosData(:, 7);

% Create a plot with joint angles vs. time
figure
plot(timeData, joint1Data, timeData, joint2Data, timeData, joint3Data)
hold on
title('Joint Values vs. Time')
xlabel('Time (ms)')
ylabel('Joint Angle (deg)')
legend('Joint 1', 'Joint 2', 'Joint 3')
hold off

% Create a plot with tip positions (x and z axes) vs. time
figure
plot(timeData, tipX, timeData, tipZ)
hold on
title('Tip Positions vs. Time')
xlabel('Time (ms)')
ylabel('Tip Position (mm)')
legend('X Position', 'Z Position')
hold off

% 2D path for the trajectory in the x-z plane
figure
plot(tipX, tipZ)
hold on
plot(100.4, 196.9, 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'k')
plot(170.4, 78.79, 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'k')
plot(75.34, 66.93, 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'k')
title('X-Z Plane Trajectory of the Arm')
xlabel('X Position (mm)')
ylabel('Z Position (mm)')
hold off

% 2D path for the trajectory in the x-y plane
figure
plot(tipX, tipY)
hold on
plot(100.4, -0.4204, 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'k')
plot(170.4, -0.7163, 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'k')
plot(75.34, -0.3156, 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'k')
title('X-Y Plane Trajectory of the Arm')
xlabel('X Position (mm)')
ylabel('Y Position (mm)')
hold off


