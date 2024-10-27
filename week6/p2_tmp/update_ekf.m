function update_ekf (idx, dx, dy, sx2, sy2)

global pose cov

% landmark x and y value
landmark_idx = [4+2*(idx-1),5+2*(idx-1)];
Lx = pose(4+2*(idx-1));
Ly = pose(5+2*(idx-1));

% current robot (x,y,theta) estimate
x_current = pose(1:3);
x = x_current(1);
y = x_current(2);
t = x_current(3);

% measurement
z = [dx, dy]';
R = diag([sx2,sy2]);
if (isnan(Lx) && isnan(Ly))
    % initialize landmark
    Lx_init = 
    Ly_init = 
    pose(landmark_idx) = [Lx_init, Ly_init]';

else
    % update with landmark
    z_predict = 
    
    % measurement jacobian (2 x (3+2*10)
    H =
    
    % Kalman update equations
    K = 
    pose = 
    cov = 
end

