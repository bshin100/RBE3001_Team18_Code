% Combine plots to compare repeatability in identical trials

data1 = readmatrix('../Data/lab1data_P5_4.csv');
data2 = readmatrix('../Data/lab1data_P5_4_noint.csv');
%data3 = readmatrix('../Data/lab1data_noint_3.csv');

data_times = data1(:,1);
data1_joint1 = data1(:,2); 
data1_joint2 = data1(:,3); 
data1_joint3 = data1(:,4); 
data2_joint1 = data2(:,2);
data2_joint2 = data2(:,3); 
data2_joint3 = data2(:,4); 
%data3_joint1 = data3(:,2);

% Plotting time!
figure
plot(data_times, data1_joint1)
hold on
plot(data_times, data1_joint2)
plot(data_times, data1_joint3)
plot(data_times, data2_joint1)
plot(data_times, data2_joint2)
plot(data_times, data2_joint3)
%plot(data_times, data3_joint1)
title('Combined Motion Profile of Joints, Position 4')
xlabel('Time Elapsed (ms)')
ylabel('Joint Position (degrees)')
legend('Joint 1 - Interpolation', 'Joint 2 - Interpolation', ...
    'Joint 3 - Interpolation', 'Joint 1 - No Interpolation', ...
    'Joint 2 - No Interpolation', 'Joint 3 - No Interpolation')
hold off