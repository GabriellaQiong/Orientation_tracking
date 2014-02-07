function [rpy] = rot2rpy(R)
% ROT2RPY() converts rotation matrix to raw pitch yaw angles
%
% Written by Qiong Wang at University of Pennsylvania
% 02/06/2014

% R = [ cos(yaw)*cos(pitch) - sin(roll)*sin(yaw)*sin(pitch), 
%     cos(pitch)*sin(yaw) + cos(yaw)*sin(roll)*sin(pitch), -cos(roll)*sin(pitch)]
% 
%     [ -cos(roll)*sin(yaw),           cos(roll)*cos(yaw),              sin(roll)]
% 
%     [ cos(yaw)*sin(pitch) + cos(pitch)*sin(roll)*sin(yaw), 
%     sin(yaw)*sin(pitch) - cos(yaw)*cos(pitch)*sin(roll),  cos(roll)*cos(pitch)]

roll  = asin(R(2,3));
yaw   = atan2(-R(2,1)/cos(roll),R(2,2)/cos(roll));
pitch = atan2(-R(1,3)/cos(roll),R(3,3)/cos(roll));
rpy   = [roll, pitch, yaw];