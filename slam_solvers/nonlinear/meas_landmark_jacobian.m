% MEAS_LANDMARK_JACOBIAN
% 16-833 Spring 2019 - *Stub* Provided
% Compute the Jacobian of the measurement function
%
% Arguments: 
%     rx    - robot's x position
%     ry    - robot's y position
%     lx    - landmark's x position
%     ly    - landmark's y position
%
% Returns:
%     H     - Jacobian of the measurement fuction
%
function H = meas_landmark_jacobian(rx, ry, lx, ly)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% Your code goes here %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
var = (lx-rx)^2 + (ly-ry)^2;
% Jaccobian of Z wrt landmark Rx,Ry
H_measure_pose = [(ly-ry)/var,      -(lx-rx)/var,    -(ly-ry)/var, (lx-rx)/var;
                -(lx-rx)/sqrt(var), -(ly-ry)/sqrt(var),(lx-rx)/sqrt(var), (ly-ry)/sqrt(var)];
% Jaccobian of Z wrt landmark Lx,Ly
H_measure_land =[-(ly-ry)/var     ,       (lx-rx)/var;
                (lx-rx)/sqrt(var), (ly-ry)/sqrt(var)];
            
H=[H_measure_pose,H_measure_land];
