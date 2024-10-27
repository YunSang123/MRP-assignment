% Z-Y-X Euler angle

% clear; clc;

R01 = Rot('z',180)*Rot('y',0)*Rot('x',0);
t01 = transpose([2 0 0]);
R12 = Rot('z',0)*Rot('y',90)*Rot('x',150);
t12 = transpose([0 0 1]);

T01 = [R01 t01;
       0 0 0 1];

T12 = [R12 t12;
       0 0 0 1];

T02 = T01 * T12;