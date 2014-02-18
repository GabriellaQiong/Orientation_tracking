function W = quat2mat(q)
% QUAT2MAT() converts quaternion to matrix
% 
% Written by Qiong Wang at University of Pennsylvania
% 02/07/2014
%
% INPUT
% q -- N x 4

W           = transpose(bsxfun(@rdivide, q(:, 2 : 4), sqrt(1 - q(:, 1).^2)));
W(isnan(W)) = 0;
end