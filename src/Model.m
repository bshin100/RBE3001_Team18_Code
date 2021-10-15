classdef Model < handle
    
    properties
        %robot; % Instantiate a robot class inside Model
    end
    
    methods
        function self = Model()
        end
        
        % Takes in a 3x1 array q of joint values and plots a stick model of
        % the arm showing all frames, joints, and links.
        function plot_arm(self, q, eeLinVel)
            
            % Pull calculated intermediate transformation matrices from the
            % Robot class.     
            T0_1 = self.robot.fkInter(q, 1);
            T0_2 = self.robot.fkInter(q, 2);
            T0_3 = self.robot.fkInter(q, 3);
            T0_4 = self.robot.fk3001(q);
            
            % Extract the position vectors
            P0_1 = T0_1(1:3, 4);
            P0_2 = T0_2(1:3, 4);
            P0_3 = T0_3(1:3, 4);
            P0_4 = T0_4(1:3, 4);
            
            clf;
            
            % Plot links and joints in 3D
            ax = axes;
            ax.ColorOrder = [0 0 1];
            
            %figure(2)
            % Start with origin at base frame, plot joint positions
            plot3(0, 0, 0, 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'k')
            hold on
            plot3(T0_1(1,4), T0_1(2,4), T0_1(3,4), 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'k')
            plot3(T0_2(1,4), T0_2(2,4), T0_2(3,4), 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'k')
            plot3(T0_3(1,4), T0_3(2,4), T0_3(3,4), 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'k')
            plot3(T0_4(1,4), T0_4(2,4), T0_4(3,4), 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'k')
            
            % Plot links between joints
            plot3([0 P0_1(1) P0_2(1) P0_3(1) P0_4(1)], [0 P0_1(2) P0_2(2) P0_3(2) P0_4(2)], ...
                [0 P0_1(3) P0_2(3) P0_3(3) P0_4(3)], '-k', 'LineWidth', 2)
            
            % Plot frames
            lineLength = 30; % Define lengths of the axis lines
            
            % Frame 0
            fr0_x = plot3([0 lineLength], [0 0], [0 0], '-r', 'LineWidth', 1);
            fr0_y = plot3([0 0], [0 lineLength], [0 0], '-g', 'LineWidth', 1);
            fr0_z = plot3([0 0], [0 0], [0 lineLength], '-b', 'LineWidth', 1);
            
            % Frame 1
            plot3([0 lineLength], [0 0], [P0_1(3) P0_1(3)], '-r', 'LineWidth', 1)
            plot3([0 0], [0 lineLength], [P0_1(3) P0_1(3)], '-g', 'LineWidth', 1)
            plot3([0 0], [0 0], [P0_1(3) P0_1(3)+lineLength], '-b', 'LineWidth', 1)
            
            % Frame 2
            plot3([P0_2(1) P0_2(1)+T0_2(1,1)*lineLength], [P0_2(2) P0_2(2)+T0_2(2,1)*lineLength], [P0_2(3) P0_2(3)+T0_2(3,1)*lineLength], '-r', 'LineWidth', 1)
            plot3([P0_2(1) P0_2(1)+T0_2(1,2)*lineLength], [P0_2(2) P0_2(2)+T0_2(2,2)*lineLength], [P0_2(3) P0_2(3)+T0_2(3,2)*lineLength], '-g', 'LineWidth', 1)
            plot3([P0_2(1) P0_2(1)+T0_2(1,3)*lineLength], [P0_2(2) P0_2(2)+T0_2(2,3)*lineLength], [P0_2(3) P0_2(3)+T0_2(3,3)*lineLength], '-b', 'LineWidth', 1)
            
            % Frame 3
            plot3([P0_3(1) P0_3(1)+T0_3(1,1)*lineLength], [P0_3(2) P0_3(2)+T0_3(2,1)*lineLength], [P0_3(3) P0_3(3)+T0_3(3,1)*lineLength], '-r', 'LineWidth', 1)
            plot3([P0_3(1) P0_3(1)+T0_3(1,2)*lineLength], [P0_3(2) P0_3(2)+T0_3(2,2)*lineLength], [P0_3(3) P0_3(3)+T0_3(3,2)*lineLength], '-g', 'LineWidth', 1)
            plot3([P0_3(1) P0_3(1)+T0_3(1,3)*lineLength], [P0_3(2) P0_3(2)+T0_3(2,3)*lineLength], [P0_3(3) P0_3(3)+T0_3(3,3)*lineLength], '-b', 'LineWidth', 1)
            
            % Frame 4
            plot3([P0_4(1) P0_4(1)+T0_4(1,1)*lineLength], [P0_4(2) P0_4(2)+T0_4(2,1)*lineLength], [P0_4(3) P0_4(3)+T0_4(3,1)*lineLength], '-r', 'LineWidth', 1)
            plot3([P0_4(1) P0_4(1)+T0_4(1,2)*lineLength], [P0_4(2) P0_4(2)+T0_4(2,2)*lineLength], [P0_4(3) P0_4(3)+T0_4(3,2)*lineLength], '-g', 'LineWidth', 1)
            plot3([P0_4(1) P0_4(1)+T0_4(1,3)*lineLength], [P0_4(2) P0_4(2)+T0_4(2,3)*lineLength], [P0_4(3) P0_4(3)+T0_4(3,3)*lineLength], '-b', 'LineWidth', 1)      
           
            
            % Plot Velocity of End-Effector
            plot3([T0_4(1,4) T0_4(1,4)+eeLinVel(1)], [T0_4(2,4) T0_4(2,4)+eeLinVel(2)], [T0_4(3,4) T0_4(3,4)+eeLinVel(3)], '-m', 'LineWidth', 1.5)
            
            % Formatting
            title('3D Plot of the Robot')
            xlabel('X Position (mm)')
            ylabel('Y Position (mm)')
            zlabel('Z Position (mm)')
            grid on
            axis([-200 200 -200 200 0 400])
            legend([fr0_x fr0_y fr0_z], {'X Unit Vector','Y Unit Vector','Z Unit Vector'})
            hold off
            
            drawnow
        end
        
    end
end