% RBE 3001 - Lab 2
% Matrix Transformation Functions

% This file is just a function holder and is not actively used - the 
% necessary functions are re-implemented in Robot.m.
% This file is kept as reference and for generation of slow 
% symbolic calculations.

% NOTE: DIMENSIONS NEED TO BE IN MM AND DEG

syms theta1 theta2 theta3
T0_1 = dh2fk([0 55 0 0])
T0_2 = dh2fk([0 55 0 0; theta1 40 0 -90])
T0_3 = dh2fk([0 55 0 0; theta1 40 0 -90;(theta2-90) 0 100 0])

% Takes in a 1x4 array corresponding to one row of a DH parameter table
% and generates the associated intermediate transformation and returns
% a corresponding symbolic 4x4 homogeneous transformation matrix.
function T = dh2mat(q)
	theta = q(1); % Separate input array into variables
	d = q(2);
    a = q(3);
	alpha = q(4);
    %syms theta d a alpha; % Disable this when piping into `dh2fk()`
    
	ctheta = cosd(theta); % Precalculate basic functions for faster runtime
	stheta = sind(theta);
	calpha = cosd(alpha);
	salpha = sind(alpha);
    T = [ctheta -stheta*calpha  stheta*salpha a*ctheta; 
         stheta  ctheta*calpha -ctheta*salpha a*stheta; 
         0 salpha calpha d; 0 0 0 1];
end

% Takes in an nx4 array corresponding to the n rows of the full DH
% parameter table then generates a corresponding symbolic 4x4 homogeneous
% transformation matrix for the composite transformation.
function T = dh2fk(q)
    
    syms T_Accum T_loop
    T_Accum = eye(4);
    inputNumRows = size(q,1);
    
    % Cumulatively calculate the frame transformations from 1 to n
    for n = 1:inputNumRows  % Loop bounded by num rows of input q array
        T_loop =  dh2mat([q(n,1) q(n,2) q(n,3) q(n,4)]);
        T_Accum = T_Accum * T_loop; % Multiply Matrices
    end
    % TODO: Pull intermediate transformations for the 3D stick model
    
    T = T_Accum;
end

% Takes n joint configurations in an nx1 vector (i.e. joint angles) and
% returns a 4x4 homogeneous transformation matrix for the tip frame with
% respect to the base frame.
function T = fk3001(q)
    % Generate symbolic transformation matrix to then hardcode below for 
    % faster operation time.   
    %{
    syms theta1 theta2 theta3
    dh_table = [0 55 0 0; theta1 40 0 -90; ...
       (theta2-90) 0 100 0; (theta3+90) 0 100 0];
    T = dh2fk(dh_table);
    %}
    
    theta1 = q(1);
    theta2 = q(2);
    theta3 = q(3);
    
    % Scary transformation matrix
    T = [ cos((pi*theta1)/180)*cos((pi*(theta2 - 90))/180)*cos((pi*(theta3 + 90))/180) - cos((pi*theta1)/180)*sin((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180), - cos((pi*theta1)/180)*cos((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180) - cos((pi*theta1)/180)*cos((pi*(theta3 + 90))/180)*sin((pi*(theta2 - 90))/180), -sin((pi*theta1)/180), 100*cos((pi*theta1)/180)*cos((pi*(theta2 - 90))/180) + 100*cos((pi*theta1)/180)*cos((pi*(theta2 - 90))/180)*cos((pi*(theta3 + 90))/180) - 100*cos((pi*theta1)/180)*sin((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180);
          sin((pi*theta1)/180)*cos((pi*(theta2 - 90))/180)*cos((pi*(theta3 + 90))/180) - sin((pi*theta1)/180)*sin((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180), - sin((pi*theta1)/180)*cos((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180) - sin((pi*theta1)/180)*cos((pi*(theta3 + 90))/180)*sin((pi*(theta2 - 90))/180),  cos((pi*theta1)/180), 100*sin((pi*theta1)/180)*cos((pi*(theta2 - 90))/180) + 100*sin((pi*theta1)/180)*cos((pi*(theta2 - 90))/180)*cos((pi*(theta3 + 90))/180) - 100*sin((pi*theta1)/180)*sin((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180);
                                                  - cos((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180) - cos((pi*(theta3 + 90))/180)*sin((pi*(theta2 - 90))/180),                                             sin((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180) - cos((pi*(theta2 - 90))/180)*cos((pi*(theta3 + 90))/180),                     0,                                                           95 - 100*cos((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180) - 100*cos((pi*(theta3 + 90))/180)*sin((pi*(theta2 - 90))/180) - 100*sin((pi*(theta2 - 90))/180);
                                                                                                                                                                    0,                                                                                                                                                             0,                     0,                                                                                                                                                                                                                          1];
 
end


