function [q, rots] = ukf_estimate_7_state(acc, vel, P, Q, R, ts)
% UKF_ESTIMATE() fuses the data from both accelerometer and gyroscope and
% give estimation of orientation through Uncented Kalman Filter
%
% Written by Qiong Wang at University of Pennsylvania
% 02/07/2014
% Refer to Kraft's paper: http://hep1.physik.uni-bonn.de/fileadmin/Publications/Mqube/Kraft_FusionPaperWeb.pdf
%
% INPUT (N is the number of data points)
% acc, vel -- 3 x N acceleration and angular velocity
% P, Q, R  -- 6 x 6 covariance matrix
%             P : state covariance matrix              (time t)
%             Q : process noise matrix                 (time t -> time t+1)
%             R : measurement/observation noise matrix (time t measure)
% ts       -- 1 x N time stamp
%
% OUTPUT
% q        -- N x 4 quaternion matrix
% rots     -- 3 x 3 x N rotation matrix

% Initialize
n       = 6;
datNum  = size(acc, 2);
q       = zeros(datNum, 4);
rots    = zeros(3, 3, datNum);
x       = [1 0 0 0 0 0 0]';
w       = [0 0 0]';
thresh  = 1e-3;      
maxIter = 100;

% Precompute
deltaT  = ts - [ts(1), ts(1 : end - 1)];

% UKF filter
for i = 1: datNum
%     fprintf('Processing data point # %d...\n', i);
    % Sigma Points (P -- 6 x 6, n = 6, 12 sigma points)
%     P  = P + 1e-10 * eye(6);
    S  = chol(P + Q);
    W1 = S * sqrt(2 * n);
    W  = [W1, -W1];
    
    % Quaternion Sigma Points
    X(:, 1 : 4) = mat2quat(W);         % 12 x 4
    X(:, 5 : 7) = W(4 : 6, :)';
    
    % Transformation of Sigma Points
    wNorm  = sqrt(sum(w.^2, 1));
    angle  = wNorm * deltaT(i);
    axis   = w / wNorm;
    qDelta = [cos(angle / 2) axis' * sin(angle / 2)];
    q0     = quatmultiply(x(1 : 4)', X(:, 1 : 4));
    
    % Find Transformed Sigma Points Y
    Y(:, 1 : 4) = quatnormalize(quatmultiply(q0, qDelta));
    Y(:, 5 : 7) = bsxfun(@plus, X(:, 5 : 7), w');
    
    % Computation of Mean (Gradient Descent)
    iter    = 0;
    s       = x(1 : 4)';
    errNorm = 1;   
    
    while (errNorm >= thresh && iter < maxIter)
        errQuat = quatmultiply(Y(:, 1 : 4), quatinv(s));
        errVec  = quat2mat(errQuat);
        errMean = mean(errVec, 2);
        errQuat = mat2quat(errMean);
        errNorm = sqrt(sum(errMean.^2, 1));
        s       = quatmultiply(errQuat, s);
        iter    = iter + 1;
    end
    
    % Computation of Covariance
    % (1) A Priori State Vector Covariance
    x_(1 : 4, :) = s';
    x_(5 : 7, :) = mean(Y(:, 5 : 7), 1)';
    Wp           = [errVec', Y(:, 5 : 7)];
    P_           = cov(Wp);                        % cov() between columns
    
    % (2) Measurement Estimate Covariance
    Z(1 : 3, :) = quat2mat(quatnormalize(quatmultiply(Y(:, 1 : 4), quatmultiply([0 0 0 1], quatinv(Y(:, 1 : 4))))));
    Z(4 : 6, :) = Y(:, 5 : 7)';
    
    z_  = mean(Z, 2);
    z   = [acc(:, i); vel(:, i)];
    v   = z - z_;    
    Pzz = cov(Z');
    Pvv = Pzz + R;
    
    % (3) Cross correlation matrix
    Pxz  = zeros(6);
    Zabs = bsxfun(@minus, Z, z_);
    for j = 1 : 2 * n
        Pxz = Pxz + Wp(j, :)' * Zabs(:, j)';
    end
    Pxz = Pxz / 2 / n;
    
    % Kalman Gain and Update
    K        = Pxz / Pvv;
    measure  = K * v;
    x(1 : 4) = transpose(quatmultiply(x_(1 : 4)', mat2quat(measure)));
    x(5 : 7) = x_(5 : 7) + measure(4 : 6);
    P        = P_ - K * Pvv * K';
%     P        = P_;
    w        = x(5 : 7);
    
    % Record data
    q(i, :)       = x(1 : 4)';
    rots(:, :, i) = quat2dcm(q(i, :));
end