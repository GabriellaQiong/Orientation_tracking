% Run Script for Project 2 Orientation Tracking, ESE 650
% Written by Qiong Wang at University of Pennsylvania
% 02/04/2014

%% Clear up
clear all;
clc;

%% Initialize
check   = true;                  % Whether to do sanity check
verbose = false;                 % Whether to show the details
P       = eye(3);                % State covariance matrix
Q       = eye(3);                % Process noise covariance matrix
R       = eye(3);                % Measurement noise covariance matrix

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
dataIdx = input('Please choose one dataset (1 ~ 9): ');
load(fullfile(imuDir, ['imuRaw', num2str(dataIdx)]));
tsImu   = ts;
load(fullfile(viconDir, ['viconRot', num2str(dataIdx)]));
tsVicon = ts;
load(fullfile(camDir, ['cam', num2str(dataIdx)]));
tsCam   = ts;

%% Parse data and sanity check
bias        = compute_bias(vals);
[acc, Racc] = parse_acc(vals, bias);
[vel, Rvel] = parse_gyro(vals, bias, tsImu);

if check
    check_plot;
end

%% Orientation Tracking
% UKF filter
