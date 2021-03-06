classdef Robot < handle
    
    properties
        myHIDSimplePacketComs;
        pol;
        jointSetpointGoal;
        GRIPPER_ID = 1962;
        MOTOR_ID = 1848;
        MOTOR_ID_READ = 1910;
        VEL_ID = 1822;
    end
    
    methods
        
        %The is a shutdown function to clear the HID hardware connection
        function  shutdown(packet)
	    %Close the device
            packet.myHIDSimplePacketComs.disconnect();
        end
        
        % Create a packet processor for an HID device with USB PID 0x007
        function packet = Robot(dev)
            packet.myHIDSimplePacketComs=dev; 
            packet.pol = java.lang.Boolean(false);
        end
        
        %Perform a command cycle. This function will take in a command ID
        %and a list of 32 bit floating point numbers and pass them over the
        %HID interface to the device, it will take the response and parse
        %them back into a list of 32 bit floating point numbers as well
        function com = command(packet, idOfCommand, values)
                com= zeros(15, 1, 'single');
                try
                    ds = javaArray('java.lang.Double',length(values));
                    for i=1:length(values)
                        ds(i)= java.lang.Double(values(i));
                    end
                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    packet.myHIDSimplePacketComs.writeFloats(intid,  ds);
                    ret = 	packet.myHIDSimplePacketComs.readFloats(intid) ;
                    for i=1:length(com)
                       com(i)= ret(i).floatValue();
                    end
                catch exception
                    getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        
        function com = read(packet, idOfCommand)
                com= zeros(15, 1, 'single');
                try

                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    ret = 	packet.myHIDSimplePacketComs.readFloats(intid) ;
                    for i=1:length(com)
                       com(i)= ret(i).floatValue();
                    end
                catch exception
                  getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        
        function  write(packet, idOfCommand, values)
                try
                    ds = javaArray('java.lang.Double',length(values));
                    for i=1:length(values)
                        ds(i)= java.lang.Double(values(i));
                    end
                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    packet.myHIDSimplePacketComs.writeFloats(intid,  ds,packet.pol);

                catch exception
                    getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        
        % Specifies a position to the gripper
        function writeGripper(packet, value)
            try
                ds = javaArray('java.lang.Byte',length(1));
                ds(1)= java.lang.Byte(value);
                intid = java.lang.Integer(packet.GRIPPER_ID);
                packet.myHIDSimplePacketComs.writeBytes(intid, ds, packet.pol);
            catch exception
                getReport(exception)
                disp('Command error, reading too fast');
            end
        end
        
        % Opens the gripper
        function openGripper(packet)
            packet.writeGripper(180);
        end
        
        % Closes the gripper
        function closeGripper(packet)
            packet.writeGripper(0);
        end
        
        % Set position joint setpoint 
        % Takes in 1x3 array of joint values in deg to be sent to the
        % actuators and bypasses interpolation.
        function servo_jp(packet, positionArr)
            packet.jointSetpointGoal = positionArr;
            % Write a 15 float packet containing the joint values
            packet.write(packet.MOTOR_ID, [0; 0; positionArr.'; zeros(15, 1)] );
        end
        
        % Set position joint goal (with interpolation)
        % Takes in 1x3 array of joint values in deg to be sent to the
        % actuators with an interpolation time in ms.
        function interpolate_jp(packet, positionArr, interpolateTime)
            packet.jointSetpointGoal = positionArr;
            % Write a 15 float packet containing the joint values
            packet.write(packet.MOTOR_ID, [interpolateTime; 0; positionArr.'; zeros(15, 1)] );
        end
        
        % Measured joint state
        % Takes in booleans GETPOS and GETVEL and returns a 2x3
        % array for the current joint positions (deg)/velocities
        function returnVal = measured_js(packet, GETPOS, GETVEL)
            pos = zeros(1, 3);
            vel = zeros(1, 3);
            posData = packet.read(packet.MOTOR_ID_READ);
            velData = packet.read(packet.VEL_ID);

            if GETPOS
                pos = [posData(3) posData(5) posData(7)];
            end

            if GETVEL
                vel = [velData(3) velData(6) velData(9)];
            end

            returnVal = [pos; vel];

        end
        
        % Joint setpoint
        % Returns a 1x3 array containing current joint setpoint
        % positions in degrees. If interpolation is enabled, return
        % values will reflect the current intermediate setpoint.
        function setpoints = setpoint_js(packet)
            setpointData = packet.read(packet.MOTOR_ID_READ);
            setpoints = [setpointData(2) setpointData(4) setpointData(6)];
        end
        
        % Joint goal
        % Returns a 1x3 array containing end-of-motion joint
        % setpoint positions in degrees.
        function goal = goal_js(packet)
            % Read from stored value in Robot class
            goal = packet.jointSetpointGoal;
        end
        
        % Measured Cartesian position
        % Takes in data from `measured_js()` function and returns a 4x4
        % homogeneous transformation matrix for the current joint position.
        function T = measured_cp(packet)
            jointRaw = packet.measured_js(true, false); % 2x3 array
            jointVals = transpose(jointRaw(1,:)); % Format into 3x1 vector
            
            T = packet.fk3001(jointVals);         
        end
        
        % Cartesian position setpoint
        % Takes in data from `setpoint_js()` function and returns a 4x4
        % homogeneous transformation matrix for the current joint setpoints.
        % If interpolation is enabled, return values will reflect the 
        % current intermediate setpoint.
        function T = setpoint_cp(packet)
            setpoints = packet.setpoint_js();
            T = packet.fk3001(setpoints.');	
        end
        
        % Cartesian position goal
        % Takes in data from `goal_js()` and returns a 4x4 homogeneous
        % transformation matrix for the commanded end-of-motion joint
        % setpoint positions.
        function T = goal_cp(packet)
            goal = packet.goal_js();
            T = packet.fk3001(goal.');
        end
        
        % Takes in a cubic trajectory path (4x3 array) and interpolates the
        % joints to the setpoints and time interval defined by the path. 
        % Also requires a total interpolation time, and n via-points.
        % Must pass in the current time (toc) in the parameter.
        function interpolateJointsCubic(packet, path, time, totalTime, n)
            
            j1Coeff = path(:, 1);
            j2Coeff = path(:, 2);
            j3Coeff = path(:, 3);
            stepTime = totalTime / n; % Time for each step

            % Calculate via-points for each joint
            j1Setpoint = j1Coeff(4)*time^3 + j1Coeff(3)*time^2 + j1Coeff(2)*time + j1Coeff(1);
            j2Setpoint = j2Coeff(4)*time^3 + j2Coeff(3)*time^2 + j2Coeff(2)*time + j2Coeff(1);
            j3Setpoint = j3Coeff(4)*time^3 + j3Coeff(3)*time^2 + j3Coeff(2)*time + j3Coeff(1);

            lastTime = time;
            packet.servo_jp([j1Setpoint j2Setpoint j3Setpoint]); % Move!

            while (toc-lastTime < stepTime)
                % Do nothing until the stepTime has been elapsed
            end
            
        end
        
        % Takes in a cubic/quintic TASK space trajectory path to output
        % smooth, linear movement between setpoints. Also requires a total 
        % interpolation time, n via-points, and a boolean for quintic.
        % Must pass in the current time (toc) in the parameter.
        function interpolateLinearTraj(packet, path, time, totalTime, n, quintic)
            
            xPathCoeff = path(:, 1);
            yPathCoeff = path(:, 2);
            zPathCoeff = path(:, 3);
            stepTime = totalTime / n; % Time for each step

            % Calculate via-points in task-space
            xSetpoint = xPathCoeff(4)*time^3 + xPathCoeff(3)*time^2 + xPathCoeff(2)*time + xPathCoeff(1);
            ySetpoint = yPathCoeff(4)*time^3 + yPathCoeff(3)*time^2 + yPathCoeff(2)*time + yPathCoeff(1);
            zSetpoint = zPathCoeff(4)*time^3 + zPathCoeff(3)*time^2 + zPathCoeff(2)*time + zPathCoeff(1);

            if quintic
                xSetpoint = xPathCoeff(6)*time^5 + xPathCoeff(5)*time^4 ...
                    + xPathCoeff(4)*time^3 + xPathCoeff(3)*time^2 ...
                    + xPathCoeff(2)*time + xPathCoeff(1);
                ySetpoint = yPathCoeff(6)*time^5 + yPathCoeff(5)*time^4 ...
                    + yPathCoeff(4)*time^3 + yPathCoeff(3)*time^2 ...
                    + yPathCoeff(2)*time + yPathCoeff(1);
                zSetpoint = zPathCoeff(6)*time^5 + zPathCoeff(5)*time^4 ...
                    + zPathCoeff(4)*time^3 + zPathCoeff(3)*time^2 ...
                    + zPathCoeff(2)*time + zPathCoeff(1);
            end
            
            % Convert via-points into joint angles using IK
            jointSetpoints = packet.ik3001([xSetpoint; ySetpoint; zSetpoint]);

            lastTime = time;
            packet.servo_jp(jointSetpoints.'); % Move!
            
            while (toc-lastTime < stepTime)
                % Do nothing until the stepTime has been elapsed
            end
                              
        end
        
        % Checks for the completion of an interpolation motion defined by a
        % small tolerance. Takes in a joint/task space goal 3x1 vector (and 
        % corresponding boolean). Returns true (1) or false (0).
        function status = checkInterpolationStatus(packet, goal, jointSpace)
            tolerance = 0.05; % 5% tolerance
            if jointSpace
                currentPos = packet.measured_js(true, false);
                delta = abs(currentPos(1,:).' - goal);
                status = (delta / goal) < tolerance;
                status = all(status(:)); % Reduce to single boolean
            else
                currentPos = packet.measured_cp();
                delta = abs(currentPos(1:3,4) - goal);
                status = (delta / goal) < tolerance;
                status = all(status(:)); % Reduce to single boolean
            end    
        end
        
        % Takes 3 joint configurations in an 3x1 vector (i.e. joint angles) and
        % returns a 4x4 homogeneous transformation matrix for the tip frame with
        % respect to the base frame on the Hephaestus arm.
        function T = fk3001(packet, q)
            theta1 = q(1);
            theta2 = q(2);
            theta3 = q(3);

            % Scary transformation matrix, precalulated for runtime efficiency.
            T = [ cos((pi*theta1)/180)*cos((pi*(theta2 - 90))/180)*cos((pi*(theta3 + 90))/180) - cos((pi*theta1)/180)*sin((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180), - cos((pi*theta1)/180)*cos((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180) - cos((pi*theta1)/180)*cos((pi*(theta3 + 90))/180)*sin((pi*(theta2 - 90))/180), -sin((pi*theta1)/180), 100*cos((pi*theta1)/180)*cos((pi*(theta2 - 90))/180) + 100*cos((pi*theta1)/180)*cos((pi*(theta2 - 90))/180)*cos((pi*(theta3 + 90))/180) - 100*cos((pi*theta1)/180)*sin((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180);
                  sin((pi*theta1)/180)*cos((pi*(theta2 - 90))/180)*cos((pi*(theta3 + 90))/180) - sin((pi*theta1)/180)*sin((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180), - sin((pi*theta1)/180)*cos((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180) - sin((pi*theta1)/180)*cos((pi*(theta3 + 90))/180)*sin((pi*(theta2 - 90))/180),  cos((pi*theta1)/180), 100*sin((pi*theta1)/180)*cos((pi*(theta2 - 90))/180) + 100*sin((pi*theta1)/180)*cos((pi*(theta2 - 90))/180)*cos((pi*(theta3 + 90))/180) - 100*sin((pi*theta1)/180)*sin((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180);
                                                          - cos((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180) - cos((pi*(theta3 + 90))/180)*sin((pi*(theta2 - 90))/180),                                             sin((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180) - cos((pi*(theta2 - 90))/180)*cos((pi*(theta3 + 90))/180),                     0,                                                           95 - 100*cos((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180) - 100*cos((pi*(theta3 + 90))/180)*sin((pi*(theta2 - 90))/180) - 100*sin((pi*(theta2 - 90))/180);
                                                                                                                                                                            0,                                                                                                                                                             0,                     0,                                                                                                                                                                                                                          1];

        end
        

        % Takes in joint variables in 3x1 vector q and returns the 4x4
        % homogeneous transformation matrix from the base frame to the
        % numeric frame specified in n.
        function T = fkInter(packet, q, n)
            theta1 = q(1); theta2 = q(2);
            switch n
                case 1
                    % Transformation from frames 0 to 1
                    T = [ 1     0     0     0;
                          0     1     0     0;
                          0     0     1    55;
                          0     0     0     1];
                case 2
                    % Transformation from frames 0 to 2
                    T = [cos((pi*theta1)/180),  0, -sin((pi*theta1)/180),  0;
                         sin((pi*theta1)/180),  0,  cos((pi*theta1)/180),  0;
                                            0, -1,                     0, 95;
                                            0,  0,                     0,  1];
                case 3
                    % Transformation from frames 0 to 3
                    T = [cos((pi*theta1)/180)*cos((pi*(theta2 - 90))/180), -cos((pi*theta1)/180)*sin((pi*(theta2 - 90))/180), -sin((pi*theta1)/180), 100*cos((pi*theta1)/180)*cos((pi*(theta2 - 90))/180);
                         sin((pi*theta1)/180)*cos((pi*(theta2 - 90))/180), -sin((pi*theta1)/180)*sin((pi*(theta2 - 90))/180),  cos((pi*theta1)/180), 100*sin((pi*theta1)/180)*cos((pi*(theta2 - 90))/180);
                                             -sin((pi*(theta2 - 90))/180),                      -cos((pi*(theta2 - 90))/180),                     0,                 95 - 100*sin((pi*(theta2 - 90))/180);
                                                                        0,                                                 0,                     0,                                                    1];
                otherwise
                    disp('Error: n must be an integer between 0 and 3');
                    T = zeros(4,4);
            end
        end
        
        function T = lawOfCos(packet,a,b,c)
            D = (a^2 + b^2 - c^2)/(2*a*b);
            C = [sqrt(1-D^2), -sqrt(1-D^2)];
            T = [atan2(C(1),D), atan2(C(2),D)];
        end
        
        % Calculate instantaneous derivatives
        function T = readDerivative(packet, t0, t1, q0, q1)
            dxdt = (q1-q0)/(t1-t0);
            T = dxdt;
        end
        
        % Takes in a 3x1 task space position vector and returns a set of
        % corresponding joint angles.
        function T = ik3001(packet, p)
            x = p(1);
            y = p(2);
            z = p(3);
            L0 = 55;
            L1 = 40;
            L2 = 100;
            L3 = 100;
            r = sqrt(x^2 + y^2);
            theta1 = atan2(y,x);
            s = z - L1 - L0;
            alpha = atan2(s,r);
            l = sqrt(s^2+r^2);
            
            c = (2*l*L3);
            b = (l^2 + L3^2 - L2^2);
            a = sqrt(c^2 - b^2);
            
            beta = atan2(a, b);
            
            theta2 = (pi/2) - alpha - beta;
            
            c2 = 2*L2*L3;
            b2 = L2^2 + L3^2 - l^2;
            a2 = sqrt(c2^2 - b2^2);
            
            w = atan2(a2, b2);
            theta3 = (pi/2) - w;
            theta1 = rad2deg(theta1);
            theta2 = rad2deg(theta2);
            theta3 = rad2deg(theta3);
%             if(theta1 < -90 || theta1 > 90 || ...
%                theta2 < -45 || theta2 > 100 || ...
%                theta3 < -90 || theta3 > 63)
%                error("Outside of range");
%             end
            T = [theta1; theta2; theta3;];
        end
        
        % Takes in configuration q (all current joint angles, 3x1 vector) 
        % and returns the corresponding numeric 6x3 Jacobian matrix.
        function J = jacob3001(packet, q)
            detThreshold = 0.1;
            theta1 = q(1); theta2 = q(2); theta3 = q(3);
            
            % Closed-form pre-derived Jacobian for efficiency
            J = [-(5*pi*sin((pi*theta1)/180)*(cos((pi*(theta2 + theta3))/180) + sin((pi*theta2)/180)))/9, -(5*pi*cos((pi*theta1)/180)*(sin((pi*(theta2 + theta3))/180) - cos((pi*theta2)/180)))/9, -(5*pi*sin((pi*(theta2 + theta3))/180)*cos((pi*theta1)/180))/9;
                  (5*pi*cos((pi*theta1)/180)*(cos((pi*(theta2 + theta3))/180) + sin((pi*theta2)/180)))/9, -(5*pi*sin((pi*theta1)/180)*(sin((pi*(theta2 + theta3))/180) - cos((pi*theta2)/180)))/9, -(5*pi*sin((pi*(theta2 + theta3))/180)*sin((pi*theta1)/180))/9;
                                                                                                       0,                      -(5*pi*(cos((pi*(theta2 + theta3))/180) + sin((pi*theta2)/180)))/9,                      -(5*pi*cos((pi*(theta2 + theta3))/180))/9;
                                                                                                       0,                                                                   -sin((pi*theta1)/180),                                          -sin((pi*theta1)/180);
                                                                                                       0,                                                                    cos((pi*theta1)/180),                                           cos((pi*theta1)/180);
                                                                                                       1,                                                                                       0,                                                              0];
           Jdet = det(J(1:3,:));
           if (abs(Jdet) < detThreshold)
               errordlg('uh oh forg', 'Error');
               J = 0; % Return 0 for the Jacobian if singularity
               packet.shutdown();
               error('Error: Singularity approached.');
           end
                                                                                                   
        end
        
        % Takes in configuration q (all current joint angles, 3x1 vector) 
        % and a 3x1 vector of the instantaneous joint velocities and
        % returns the 6x1 matrix of linear and angular velocities.
        function T = fdk3001(packet, q, qdot)       
            J = packet.jacob3001(q);           
            T = J * qdot;
        end
        
    end
end
