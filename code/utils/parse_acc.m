function accNew = parse_acc(accRaw, bias)
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
sensitivity = 400;
scale       = Vref / 1023 / sensitivity;   % Wrong in reference datasheet

% Compute acceleration
accNew = scale .* bsxfun(@minus, accRaw, bias);
accNew = bsxfun(@times, accNew, [-1; -1; 1]); 
end