% Run Script for Project 2 Orientation Tracking, ESE 650
% Written by Qiong Wang at University of Pennsylvania
% 02/04/2014

%% Clear up
clear all;
close all;
clc;

%% Initialize
check   = false;                     % Whether to do sanity check
stitch  = false;                      % Whether to stitch the images
verbose = false;                     % Whether to show the details
thresh  = 0.1;                       % Threshold for time difference                       
P       = 0.17 * diag(rand(3,1));    % State covariance matrix
Q       = 0.2 * diag([1 1 1]);     % Process noise covariance matrix
R       = 0.5 * diag([1 1 1]);       % Measurement noise covariance matrix

% Parameters for seven state
% P       = 0.001 * eye(6);       % State covariance matrix
% Q       = diag([ones(3, 1) * 0.0001; ones(3, 1) * 0.00011]);       % Process noise covariance matrix
% R       = 0.1 * eye(6);       % Measurement noise covariance matrix

%% Path
scriptDir = fileparts(mfilename('fullpath'));
dataDir   = '/home/qiong/ese650_data/project2/';
imuDir    = fullfile(dataDir, '/imu/');
viconDir  = fullfile(dataDir, '/vicon/');
camDir    = fullfile(dataDir, '/cam/');
outputDir = fullfile(scriptDir, '../results');
if ~exist(outputDir, 'dir')
    mkdir(outputDir); 
end
addpath(genpath('../code'));

%% Load data
dataIdx = input('Please choose one dataset: ');
load(fullfile(imuDir, ['imuRaw', num2str(dataIdx)]));
tsImu   = ts;
load(fullfile(viconDir, ['viconRot', num2str(dataIdx)]));
tsVicon = ts;
if dataIdx < 3 || dataIdx == 13
    load(fullfile(camDir, ['cam', num2str(dataIdx)]));
    tsCam   = ts;
end

%% Parse data and sanity check
bias        = compute_bias(vals);
[acc, Racc] = parse_acc(vals, bias);
[vel, Rvel] = parse_gyro(vals, bias, tsImu);

if check
    check_plot;
end

%% Orientation Tracking
% UKF filter
[q, rotsUKF] = ukf_estimate(acc, vel, P, Q, R, tsImu);
h = plot_data(rots, rotsUKF, tsVicon, tsImu, outputDir, verbose);
fig_save(h, fullfile(outputDir, ['dataset' num2str(dataIdx) '_4state']), 'pdf');

%% Image Mosaicing
if ~stitch
   return; 
end

tVideo = zeros(numel(tsCam), 1);
for i= 1 : numel(tsCam)
    [~, idx]  = min(abs(tsCam(i)-tsImu));
    tVideo(i) = idx;
end

% Record each frame
h        = figure(100);
imMosaic = zeros(2000, 1000, 3); 
for i = 1 : 3 : numel(tVideo)
    fprintf('Processing image # %d ... \n', i);
    img = cam(:, :, :, i);
    if abs(tsCam(i)-tsImu(tVideo(i))) < thresh
        [yaw, pitch, roll] = dcm2angle(Rvel(:, :, tVideo(i)));
%         tf    = projective2d(rotsUKFnew(:, :, tVideo(i)) / rotsUKFnew(:, :, tVideo(i - 1)));
%         imDes = imwarp(img,tf);
        img = double(imrotate(img, roll * 180 / pi));
        dr  = 250 * tan(-pitch);
        dc  = 320 * sin(yaw);
        nr  = round(1000 - round(size(img, 1) / 2) + dr);
        nc  = round(500 - round(size(img, 2) / 2) + dc);
        if (nr >= 1 && nr <= size(imMosaic,1) - size(img, 1) + 1 && nc >= 1 && nc <= size(imMosaic,2) - size(img,2) + 1)
            imMosaic(nr : nr + size(img,1) - 1, nc : nc + size(img,2) - 1, :) = ...
            uint8(((1 - (img > 0)).*imMosaic(nr : nr + size(img,1) - 1, nc: nc + size(img, 2) - 1,:)...
            + (img > 0) .* img(:,:,:)));
            imshow(uint8(imMosaic)); axis image; axis off; drawnow;
        end
    end
end
