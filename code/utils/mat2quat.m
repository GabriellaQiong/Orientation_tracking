function q = mat2quat(W)
% MAT2QUAT() converts matrix to quaternion
% 
% Written by Qiong Wang at University of Pennsylvania
% 02/07/2014
%
% INPUT & OUTPUT
% W -- 3 x N or M x N
% q -- N x 4 

angle       = sqrt(sum(W(1 : 3, :).^2, 1));
axis        = bsxfun(@rdivide, W(1 : 3, :), angle);
q           = [cos(angle/2); bsxfun(@times, axis, sin(angle/2))]';
q(isnan(q)) = 0;
end