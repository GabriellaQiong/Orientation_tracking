function q = mat2quat(W)
% MAT2QUAT() converts matrix to quaternion
% 
% Written by Qiong Wang at University of Pennsylvania
% 02/07/2014
%
% INPUT & OUTPUT
% W -- 3 x N
% q -- N x 4 

angle = sqrt(sum(W.^2, 1));
axis  = bsxfun(@rdivide, W, angle);
q     = [cos(angle/2); bsxfun(@times, axis, sin(angle/2))]';
end