function [gyroNew, R] = parse_gyro(gyroRaw, bias, ts)
% PARSE_ACC() parse the raw data from gyrometer 
% Written by Qiong Wang at University of Pennsylvania
% 02/05/2014

% Parse different structure
if size(gyroRaw, 1) == 6
    gyroRaw = gyroRaw(4 : 6, :);
end

if size(bias, 1) == 6
    bias = bias(4 : 6, :);
end

% Parameters
Vref        = 3300;
sensitivity = 0.001;
scale       = Vref / 1023 / sensitivity * pi / 180;

% Compute acceleration
gyroNew   = scale .* bsxfun(@minus, gyroRaw, bias);
factorMat = [0 1 0; 0 0 1; 1 0 0];
gyroNew   = factorMat * gyroNew;

% Compute the rotation matrix using quaternion
velNorm = sqrt(sum(gyroNew .* gyroNew, 1));
deltaT  = ts - [ts(1), ts(1 : end - 1)];
angle   = velNorm .* deltaT;
axis    = bsxfun(@rdivide, gyroNew, velNorm); 
qDelta  = [cos(angle / 2)' transpose(bsxfun(@times, axis, sin(angle / 2)))];
q0      = [1 0 0 0];
q       = quatnormalize(quatmultiply(q0, qDelta));
R       = quat2dcm(q);
end