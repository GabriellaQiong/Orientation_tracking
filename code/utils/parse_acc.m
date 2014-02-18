function [accNew, R] = parse_acc(accRaw, bias)
% PARSE_ACC() parse the raw data from accelerometer 
% Written by Qiong Wang at University of Pennsylvania
% 02/05/2014

% Parse different structure
if size(accRaw, 1) == 6
    accRaw = accRaw(1 : 3, :);
end

if size(bias, 1) == 6
    bias = bias(1 : 3, :);
end

% Parameters
Vref        = 3300;
sensitivity = 300;                         % Higher --> more accurate
scale       = Vref / 1023 / sensitivity;   % Wrong in reference datasheet
datNum      = size(accRaw, 2);

% Compute acceleration
accNew = scale .* bsxfun(@minus, accRaw, bias);
accNew = bsxfun(@times, accNew, [-1; -1; 1]);
accNew = bsxfun(@rdivide, accNew, sqrt(sum(accNew .* accNew, 1)));

% Compute the rotation matrix
% Note: only two angles need for the rotation set no yaw and the rotation
%       matrix refers mainly from Mellinger's rot2rpy
R  = zeros(3, 3, datNum);
sr = - accNew(2, :);       cr = sqrt(1 - sr .* sr);
sp = - accNew(1, :) ./ cr; cp = accNew(3, :) ./ cr;

R(1, 1, :) = cp;
R(1, 2, :) = sr .* sp;
R(1, 3, :) = - accNew(1, :);
R(2, 2, :) = cr;
R(2, 3, :) = sr;
R(3, 1, :) = sp;
R(3, 2, :) = - cp .* sr;
R(3, 3, :) = accNew(3, :);
end