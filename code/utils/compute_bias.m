function [bias, sensitivity] = compute_bias(imuVals)
% COMPUTE_BIAS() computes the static bias of imu values
%
% Written by Qiong Wang at University of Pennsylvania
% 02/06/2014

bias        = zeros(6, 1);
bias(1 : 3) = mean(imuVals(1 : 3, 1 : 150), 2);
bias(4 : 6) = mean(imuVals(4 : 6, 1 : 150), 2);
 
% Vref        = 3300;
% sensitivity = 400;
% sensitivity = Vref / 1023 / sensitivity * bias(3);
end