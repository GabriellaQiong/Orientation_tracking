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
sensitivity = 3.3;
scale       = Vref / 1023 / sensitivity * pi / 180;
datNum      = size(gyroRaw, 2);

% Compute acceleration
gyroNew   = scale .* bsxfun(@minus, gyroRaw, bias);
factorMat = [0 1 0; 0 0 1; - 1 0 0];
gyroNew   = factorMat * gyroNew;

% Compute the rotation matrix using quaternion
velNorm = sqrt(sum(gyroNew.^2, 1));
deltaT  = ts - [ts(1), ts(1 : end - 1)];
angle   = velNorm .* deltaT;
axis    = bsxfun(@rdivide, gyroNew, velNorm); 
qDelta  = [cos(angle / 2)' transpose(bsxfun(@times, axis, sin(angle / 2)))];
q       = [1 0 0 0];
R       = zeros(3, 3, datNum);

% Note: Gyroscope outputs the rotation according to body frame the rotation
%       should be incremented
for i = 1 : datNum
    q          = quatnormalize(quatmultiply(q, qDelta(i, :)));
    R(:, :, i) = quat2dcm(q)';
end

end