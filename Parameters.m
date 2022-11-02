clear all
close all
clc
%global d focalLength principalPoint imageSize  reprojectionMatrix
d=0.2;       %meters between left and right camera
global disparityswitch points3Dswitch ORBswitch 
disparityswitch=1;
points3Dswitch=1;
ORBswitch=0;
% Stereo camera parameters , for real cameras I can use Using the Stereo
% Camera Calibrator App and Export Camera Parameters
focalLength    = [1109, 1109]; % In pixels
principalPoint = [640, 360];   % In pixels [x, y]
imageSize      = [720, 1280];  % In pixels [mrows, ncols]
baseline       = d;          % In meters

reprojectionMatrix = [1, 0, 0, -principalPoint(1); 
    0, 1, 0, -principalPoint(2);
    0, 0, 0, focalLength(1);
    0, 0, 1/baseline, 0];


% In this example, the images are already undistorted and rectified. In a general workflow, 
% uncomment the following code to undistort and rectify the images.
% currILeft     = undistortImage(currILeft, intrinsics);
% currIRight    = undistortImage(currIRight, intrinsics);
% stereoParams  = stereoParameters(intrinsics, intrinsics, eye(3), [-baseline, 0 0]);
% [currILeft, currIRight] = rectifyStereoImages(currILeft, currIRight, stereoParams, 'OutputView','full');
