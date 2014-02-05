function [tsNew] = sync_time(ts1, ts2)
% SYNC_TIME() synchronize time from different sensors
% Written by Qiong Wang at University of Pennsylvania
% 02/04/2014

% Compute the differences
tsDiff = ts1 - ts2;

% Find the smallest difference point
tsNew  = ;

% Sensor 1 to sensor 2

% Sensor 2 to sensor 1

end