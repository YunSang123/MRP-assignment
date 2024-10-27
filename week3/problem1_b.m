% Z-Y-X Euler angle

% clear; clc;

R23 = Rot('z',90)*Rot('y',180)*Rot('x',0);
t23 = transpose([0 4 0]);

T23 = [R23 t23;
       0 0 0 1];