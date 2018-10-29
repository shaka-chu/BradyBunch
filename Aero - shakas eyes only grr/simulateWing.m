% AERO3260 - Aerodynamics 1 - Assignment 2 2018
% Author SID: 460369684
% Question Number: 3
%
% Script Description: 
% This script performs simulations in Xfoil for all selected airfoils
% subset from the UIUC database

function [polar] = simulateWing(Re,M,AoA)

    % Load airfoil file
    fileName = 'naca65415.txt';
    fid = fopen(fileName,'r');

    % Get x and y coordinates ('xfoil.m' can read the filename directly,
    % but this causes issues if there are headers on occasion)
    coords = textscan(fid,'%f %f','HeaderLines',1);

    % Close all files so Matlab doesn't get confused
    fclose('all');

    % x and y coordinates
    xyMat = [coords{1} coords{2}];

    % Run 'Xfoil Interface Updated' code, written by Gus Brown and updated
    % by Louis Edelman
    polar = xfoil(xyMat,AoA,Re,M,'oper iter 200','ppar n 250');
end