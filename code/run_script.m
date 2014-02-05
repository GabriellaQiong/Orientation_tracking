% Run Script for Project 2 Orientation Tracking, ESE 650
% Written by Qiong Wang at University of Pennsylvania
% 02/04/2014

%% Clear up
clear all;
clc;

%% Initialize
verbose = true;                  % Whether to show the details

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
addpath(genpath('code'));

%% Load data
dataIdx = input('Please choose one dataset (1 ~ 9): ');
load(fullfile(imuDir, ['imuRaw', num2str(dataIdx)]));
tsImu   = ts;
load(fullfile(viconDir, ['viconRot', num2str(dataIdx)]));
tsVicon = ts;
load(fullfile(camDir, ['cam', num2str(dataIdx)]));
tsCam   = ts;

%% Parse data


%% Orientation Tracking

% UKF filter