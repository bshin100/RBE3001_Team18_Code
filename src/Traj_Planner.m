classdef Traj_Planner < handle
    
    properties
        robot;
    end

    methods
        function self = Traj_Planner(robot)
            self.robot = robot;
        end
        
        % Solves for a cubic polynomial trajectory between two via-points. 
        % Parameters include start and end times [t_start; t_end], 
        % start and end velocities, and start and end positions. 
        % The output is a 4x1 array with the coefficients of the polynomial.
        % Output is d, c, b, a
        function T = cubic_traj(self, t, vel, q)
            %q(t) = a0, a1*t + a2*t^2 + a3*t^3
            M = [1, t(1), t(1)^2, t(1)^3;
                 0, 1,  2*t(1), 3*t(1)^2;
                 1, t(2), t(2)^2, t(2)^3;
                 0,  1, 2*t(2), 3*t(2)^2;];

            given = [q(1); vel(1); q(2); vel(2);];
            T = inv(M)*given;
        end
        
        % Calculates the cubic trajectory for three joints. Takes in the
        % desired joint angles (3x1 vector) and total interpolation time 
        % (`timeDelta`) in seconds. Assumes vi and vf are 0.
        function T = cubic_calc(self, timeDelta, q)
            theta1 = q(1);
            theta2 = q(2);
            theta3 = q(3);
            
            % Measure current position for a starting point
            currJoints = self.robot.measured_js(true, false);
            currTheta1 = currJoints(1,1);
            currTheta2 = currJoints(1,2);
            currTheta3 = currJoints(1,3);
            
            % Calculate joint angle paths
            j1Path = self.cubic_traj([0; timeDelta], [0; 0], [currTheta1; theta1]);
            j2Path = self.cubic_traj([0; timeDelta], [0; 0], [currTheta2; theta2]);
            j3Path = self.cubic_traj([0; timeDelta], [0; 0], [currTheta3; theta3]);
            
            T = [j1Path j2Path j3Path]; % Output 4x3 array
        end
        
        % THIS IS NOT LINEAR INTERPOLATION, WHOEVER DESIGNED THIS LAB IS
        % CONFUSING. THIS IS FOR A LINEAR TRAJECTORY (STRAIGHT LINES).
        % Calculates a cubic/quintic trajectory for three TASK space coordinates.
        % Takes in start and ending positions of the end-effector and
        % outputs the planned trajectory in the task space. Positions must
        % be formatted as 3x1 vectors.
        function T = linear_traj(self, startPos, endPos, timeDelta, quintic)
            % Calculate task space paths
            xPath = self.cubic_traj([0; timeDelta], [0; 0], [startPos(1); endPos(1)]);
            yPath = self.cubic_traj([0; timeDelta], [0; 0], [startPos(2); endPos(2)]);
            zPath = self.cubic_traj([0; timeDelta], [0; 0], [startPos(3); endPos(3)]);
            
            if quintic 
                xPath = self.quintic_traj([0; timeDelta], [0; 0], [startPos(1); endPos(1)]);
                yPath = self.quintic_traj([0; timeDelta], [0; 0], [startPos(2); endPos(2)]);
                zPath = self.quintic_traj([0; timeDelta], [0; 0], [startPos(3); endPos(3)]);
            end
            
            T = [xPath yPath zPath]; % Output 4x3 array (cubic) or 6x3 array (quintic)
        end
        
        % Solves for a quintic polynomial trajectory betweem two
        % via-points. Parameters include start and end times [t_start;
        % t_end], start and end velocities, and start and end positions.
        % The output is a 6x1 array with the coffeicients of the polynomial.
        function T = quintic_traj(self, t, vel, q)
            acc = [0,0];
            
            %q(t) = a0, a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
            M = [1, t(1),   t(1)^2,  t(1)^3,    t(1)^4,    t(1)^5;
                 0, 1,    2*t(1),  3*t(1)^2,  4*t(1)^3,  5*t(1)^4;
                 0, 0,    2,       6*t(1),   12*t(1)^2, 20*t(1)^3;
                 1, t(2),   t(2)^2,  t(2)^3,    t(2)^4,    t(2)^5;
                 0, 1,    2*t(2),  3*t(2)^2,  4*t(2)^3,  5*t(2)^4;
                 0, 0,    2,       6*t(2),   12*t(2)^2, 20*t(2)^3;];
             
            given = [q(1); vel(1); acc(1); q(2); vel(2); acc(2);];
            T = inv(M)*given;
        end
        
    end
end