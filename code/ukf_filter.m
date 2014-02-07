function [q, rots] = ukf_filter(acc, vel, P, Q, R, ts)
% UKF_FILTER() fuses the data from both accelerometer and gyroscope and
% give estimation of orientation through Uncented Kalman Filter
%
% Written by Qiong Wang at University of Pennsylvania
% 02/07/2014
%
% INPUT (N is the number of data points)
% acc, vel -- 3 x N acceleration and angular velocity
% P, Q, R  -- 3 x 3 covariance matrix
%             P : state covariance matrix              (time t)
%             Q : process noise matrix                 (time t -> time t+1)
%             R : measurement/observation noise matrix (time t measure)
% ts       -- 1 x N time stamp
%
% OUTPUT
% q        -- N x 4 quaternion matrix
% rots     -- 3 x 3 x N rotation matrix

% Initialize
datNum  = size(acc, 2);
q       = zeros(datNum, 4);
rots    = zeros(3, 3, datNum);
deltaT  = ts - [ts(1), ts(1 : end - 1)];
s       = [1 0 0 0];                        % State vector


end