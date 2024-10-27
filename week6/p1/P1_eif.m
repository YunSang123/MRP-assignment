% PS week 06
% P1. EIF measurement update

clear; clc;

DTOR = pi/180;
RTOD = 180/pi;

% robot starts at the origin = world frame
x0 = [0 0 0]'; S0=diag([0.1^2, 0.1^2, (0.01*DTOR)^2]);

%% robot moves 
Q = diag([0.1,0.1,1*DTOR].^2);
w = mvnrnd([0,0 0]',Q,1)';
u0 = [1.9 4.1 -pi/4]';

% make a prediction
[x1_p,J] = head2tail_2d(x0, u0);
x1_true = x1_p + w;
S1_p = J(:,1:3)*S0*J(:,1:3)'+Q;

X_estim = [x0; x1_p];
X_true = [x0; x1_true];

figure(1);
scale = 0.1;
plot_mobile_robot (X_estim, scale, '2d')
hold on
plot_ellipse (x0(1:2),S0(1:2,1:2),'r');
plot_ellipse (x1_p(1:2),S1_p(1:2,1:2),'r');
plot(x1_true(1),x1_true(2),'rx');
hold off;
disp(x1_p(1:3));

%% measurement update
% 
z1 = 2; R1 = 1; H1 = [1 0 0];
z2 = 4; R2 = 1; H2 = [0 1 0];
z3 = sqrt(2*2+4*4); R3 = 1;
d = sqrt(x1_p(1)*x1_p(1) + x1_p(2)*x1_p(2));
H3 = [x1_p(1)/d x1_p(2)/d 0];

z = [z1 z2 z3]';
H = [H1;H2;H3];
R = diag([R1 R2 R3]);

L1_p = inv(S1_p);
eta_p = L1_p*(J(:,1:3)*x0 + J(:,4:6)*u0);

L1 = H'*inv(R)*H + L1_p;
eta = H'*inv(R)*z + eta_p;

x1 = inv(L1)*eta;
S1_p = inv(L1);

figure(2);
X_estim = [x0; x1];
plot_mobile_robot (X_estim, scale, '2d')
hold on
plot_ellipse (x0(1:2),S0(1:2,1:2),'r');
plot_ellipse (x1_p(1:2),S1_p(1:2,1:2),'r');
plot(x1_true(1),x1_true(2),'rx');
disp(x1_p(1:3));
