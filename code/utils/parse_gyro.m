function gyroNew = parse_gyro(gyroRaw, bias)
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
sensitivity = 100;
scale       = Vref / 1023 / sensitivity * pi / 180;

% Compute acceleration
gyroNew   = scale .* bsxfun(@minus, gyroRaw, bias);
factorMat = [0 1 0; 0 0 1; 1 0 0];
gyroNew   = factorMat * gyroNew; 
end