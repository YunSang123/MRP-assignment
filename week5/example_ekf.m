% Mobile Robot Mapping lecture 5: EKF example

clear all; close all;

DTOR = pi/180;
RTOD = 180/pi;

% robot starts at the origin = world frame
x0 = [0 0 0]'; S0=diag([0.1^2, 0.1^2, (0.01*DTOR)^2]);

% robot moves 
Q = diag([0.1,0.1,1*DTOR].^2);
w = mvnrnd([0,0 0]',Q,1)';
u0 = [1.9 4.1 -pi/4]';

% make a prediction
[x1_p,J] = head2tail_2d(x0, u0);
x1_true = x1_p + w;
S1_p = J(:,1:3)*S0*J(:,1:3)'+Q;

X_estim = [x0; x1_p];
X_true = [x0; x1_true];

%% P1(a) below is code for update with meas 1 change this for meas 3
% measurement #1 z = x1
%z1 = 2; R1 = 1;
%H1 = [1 0 0];
z3 = TODO
R3 = TODO
H3 = TODO

% predicted measurement from predicted values
%z_minus = x1_p(1)
z_minus = TODO

% Kalman update
K = TODO
x1 = TODO
S1 = TODO

% plot
figure(1);
plot_mobile_robot (X_estim, '2d')
hold on
plot(x1_true(1),x1_true(2),'kx'); % true postion black
plot(x1_p(1),x1_p(2),'yo'); % estimated by robot motion yellow
plot_ellipse (x0(1:2),S0(1:2,1:2),'r'); % uncertainty at origin
plot_ellipse (x1_p(1:2),S1_p(1:2,1:2),'r'); % uncertainty after move
X_estim = [x0; x1];
plot_mobile_robot (X_estim, '2d')
hold on
plot(x1(1),x1(2),'g*'); % update pose after Kalman filter
plot_ellipse (x1(1:2),S1(1:2,1:2),'g'); % uncertainty after Kalman filter

x1_update_z3 = x1
S1_update_z3 = S1
%% P1(b) update with all meas 1,2 and 3
z1 = 2; R1 = 1;
z2 = 4; R2 = 1;
z3 = 5; R3 = 1;

% measurement
z = TODO
R = TODO
H = TODO

% predicted measurement from predicted values
z_minus = TODO

% Kalman update
K = TODO
x1 = TODO
S1 = TODO

% plot
figure(2);
plot_mobile_robot (X_estim, '2d',2)
hold on
plot(x1_true(1),x1_true(2),'kx'); % true postion black
plot(x1_p(1),x1_p(2),'yo'); % estimated by robot motion yellow
plot_ellipse (x0(1:2),S0(1:2,1:2),'r'); % uncertainty at origin
plot_ellipse (x1_p(1:2),S1_p(1:2,1:2),'r'); % uncertainty after move
X_estim = [x0; x1];
plot(x1(1),x1(2),'g*'); % update pose after Kalman filter
plot_ellipse (x1(1:2),S1(1:2,1:2),'g'); % uncertainty after Kalman filter
x1
S1