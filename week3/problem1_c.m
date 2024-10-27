% X-Y-Z Euler angle

% clear; clc;

R03 = Rot('x',90)*Rot('y',-60)*Rot('z',0);
t03 = transpose([0 2*sqrt(3) 1]);

T03 = [R03 t03;
       0 0 0 1];