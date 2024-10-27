% slam101 lec 08 Uncertainty Propagation
% Example in R. Smith 1990 paper

clear all; close all;

DTOR = pi/180;
RTOD = 180/pi;

% robot starts at the origin = world frame
xr = [0 0 0]'; % xr = initial pose of the robot
Sr = zeros(3,3); % Sr = Variance of the initial pose

% robot sees the object #1
x1 = [-1 1 0]'; % observation state about the object #1
R = diag([0.2^2, 0.2^2, (1*DTOR)^2]); % Uncertainty of x1
X = [xr; x1]; % State vector
Cov = [Sr zeros(3,3); % Covariance w.r.t X(xr; x1)
       zeros(3,3) R];

% robot moves
u = [1 1 0]'; % Command for robot to move to pose 1
Q = diag([0.15^2, 0.05^2, (1*DTOR)^2]); % Uncertainty of u
[yr, J] = head2tail_2d(xr,u); % yr = xr+u, J = J_oplus
Sy = J*[Sr zeros(3,3);zeros(3,3) Q]*transpose(J); % Sy = Variance of pose(1,1,0)

X = [yr; x1];
Cov = [Sy zeros(3,3);zeros(3,3) R];

% robot sees #2
z2 = [-1 1 0]'; % z2 = Observation about object #2
R = diag([0.1^2, 0.2^2, (1*DTOR)^2]); % Uncertainty for observation z2
[x2, J] = head2tail_2d(yr,z2);
J1 = J(1:3,1:3);
S2 = J*[Sy zeros(3,3);zeros(3,3) R]*transpose(J);

X = [yr;x1;x2];
Cov = [Sy zeros(3,3) Sy*transpose(J1);
       zeros(3,3) R zeros(3,3);
       J1*Sy zeros(3,3) S2]

plot_pose_with_cov_ellipse (X, Cov)

% original 2,2 = diag([0.2^2, 0.2^2, (1*DTOR)^2]), not R
% plot_pose_with_cov_ellipse (X, Cov)