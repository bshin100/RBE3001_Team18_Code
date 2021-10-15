classdef Graphing < handle
    
    properties
        robot; % Instantiate a Robot class
    end
    
    methods
        function self = Graphing(robot)
            self.robot = robot;
        end
  
        % Create a plot with joint angles vs. time
        % Takes in an array of joint values, with the columns corresponding
        % to each joint. 
        function plotJoints(self, timeData, jointArray)
            figure
            plot(timeData, jointArray(:, 1), timeData, jointArray(:, 2), ...
                timeData, jointArray(:, 3))
            hold on
            title('Joint Values vs. Time')
            xlabel('Time (ms)')
            ylabel('Joint Angle (deg)')
            legend('Joint 1', 'Joint 2', 'Joint 3')
            hold off
            colororder([0 0.4470 0.7410; 0.8500 0.3250 0.0980; 0.9290 0.6940 0.1250])
        end
        
        % Create a plot with tip positions vs. time
        % Takes in an array of tip positions with the columns corresponding
        % to each axis (i.e. x, y, and z).
        function plotTip(self, timeData, positionArray)
            figure
            plot(timeData, positionArray(:, 1), timeData, positionArray(:, 2), ...
                timeData, positionArray(:, 3))
            hold on
            title('Tip Positions vs. Time')
            xlabel('Time (ms)')
            ylabel('Tip Position (mm)')
            legend('X Position', 'Y Position', 'Z Position')
            hold off
            colororder([0 0.4470 0.7410; 0.8500 0.3250 0.0980; 0.9290 0.6940 0.1250])
        end
        
        % Create a plot with tip positions plotted in task space.
        % Takes in an array of tip positions with the columns corresponding
        % to each axis (i.e. x, y, and z).
        function plotTip3D(self, positionArray)
            
            tipX = positionArray(:, 1);
            tipY = positionArray(:, 2);
            tipZ = positionArray(:, 3);
            
            figure
            view(3) % Force hold into 3D
            hold on
            grid on
            plot3(tipX, tipY, tipZ)
            title('Tip Path in Task Space')
            xlabel('X Axis')
            ylabel('Y Axis')
            zlabel('Z Axis')
            hold off
            colororder([0 0.4470 0.7410; 0.8500 0.3250 0.0980; 0.9290 0.6940 0.1250])
        end
        
        % Creates a figure with subplots for position, velocity, and
        % acceleration with respect to time.
        % Takes in the time array, and all the respective data arrays.
        function plotKinematics(self, timeData, positionArray, ...
                velocityArray, accelArray)
            
            % Measure lengths of velocity and acceleration arrays since
            % they're typically shorter than the timeData array.
            velLen = length(velocityArray);
            accLen = length(accelArray);
            
            figure
            subplot(3,1,1)
            hold on
            plot(timeData, positionArray(:, 1), timeData, ...
                positionArray(:, 2), timeData, positionArray(:, 3))
            title('Tip Positions vs. Time')
            xlabel('Time (ms)')
            ylabel('Tip Position (mm)')
            legend('X Position', 'Y Position', 'Z Position')
            hold off
            colororder([0 0.4470 0.7410; 0.8500 0.3250 0.0980; 0.9290 0.6940 0.1250])
            
            subplot(3,1,2)
            hold on
            plot(timeData(1:velLen), velocityArray(:,1), timeData(1:velLen),...
                velocityArray(:,2), timeData(1:velLen), velocityArray(:,3))
            title('Tip Velocity vs. Time')
            xlabel('Time (ms)')
            ylabel('Tip Velocity (mm/s)')
            legend('X Velocity', 'Y Velocity', 'Z Velocity')
            hold off
            colororder([0 0.4470 0.7410; 0.8500 0.3250 0.0980; 0.9290 0.6940 0.1250])
            
            subplot(3,1,3)
            hold on
            plot(timeData(1:accLen), accelArray(:,1), timeData(1:accLen), ...
                accelArray(:,2), timeData(1:accLen), accelArray(:,3))
            title('Tip Acceleration vs. Time')
            xlabel('Time (ms)')
            ylabel('Tip Acceleration (mm/s^2)')
            legend('X Acceleration', 'Y Acceleration', 'Z Acceleration')
            hold off
            colororder([0 0.4470 0.7410; 0.8500 0.3250 0.0980; 0.9290 0.6940 0.1250])
        end
        
        % Creates a plot of the end-effector linear, angular, and magnitude
        % velocities with respect to time.
        % Takes in the time array, Jacobian velocities (nx6), and the
        % magnitude of linear velocity.
        function plotJacobVelocities(self, timeData, jacobianVels, magVel)
            figure
            subplot(3,1,1)
            hold on
            plot(timeData, jacobianVels(:, 1), timeData, ...
                jacobianVels(:, 2), timeData, jacobianVels(:, 3))
            title('Tip Linear Velocity vs. Time')
            xlabel('Time (ms)')
            ylabel('Tip Velocity (mm/s)')
            legend('X Velocity', 'Y Velocity', 'Z Velocity')
            hold off
            colororder([0 0.4470 0.7410; 0.8500 0.3250 0.0980; 0.9290 0.6940 0.1250])
            
            subplot(3,1,2)
            hold on
            plot(timeData, jacobianVels(:, 4), timeData, ...
                jacobianVels(:, 5), timeData, jacobianVels(:, 6))
            title('Tip Angular Velocity vs. Time')
            xlabel('Time (ms)')
            ylabel('Tip Velocity (deg/s)')
            legend('X Velocity', 'Y Velocity', 'Z Velocity')
            hold off
            colororder([0 0.4470 0.7410; 0.8500 0.3250 0.0980; 0.9290 0.6940 0.1250])
            
            subplot(3,1,3)
            hold on
            plot(timeData, magVel)
            title('Magnitude of Tip Linear Velocity vs. Time')
            xlabel('Time (ms)')
            ylabel('Tip Velocity (mm/s)')
            legend('Velocity Magnitude')
            hold off
            colororder([0 0.4470 0.7410; 0.8500 0.3250 0.0980; 0.9290 0.6940 0.1250])
        end
    end
end
