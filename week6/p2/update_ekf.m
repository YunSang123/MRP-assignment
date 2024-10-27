function update_ekf (idx, dx, dy, sx2, sy2)

global pose cov

% landmark x and y value
landmark_idx = [4+2*(idx-1),5+2*(idx-1)];
Lx = pose(4+2*(idx-1)); % x_position of landmark in state vector 
Ly = pose(5+2*(idx-1)); % y_position of landmark in state vector

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
    Lx_init = x + z(1)*cos(t) - z(2)*sin(t);
    Ly_init = y + z(1)*sin(t) + z(2)*cos(t);
    pose(landmark_idx) = [Lx_init, Ly_init]';
    cov(landmark_idx(1),landmark_idx(1)) = sx2;
    cov(landmark_idx(2),landmark_idx(2)) = sy2;
else
    % update with landmark
    delta = [Lx-x Ly-y]';
    q = delta'*delta;
    dtheta = atan2(delta(2),delta(1))-t;
    z_predict = [sqrt(q)*cos(dtheta) sqrt(q)*sin(dtheta)]';

    % measurement jacobian (2 x (3+2*10)
    h11 = -sqrt(q)/q*delta(1)*cos(dtheta)-sqrt(q)*sin(dtheta)*delta(2)/q;
    h12 = -sqrt(q)/q*delta(2)*cos(dtheta)+sqrt(q)*sin(dtheta)*delta(1)/q;
    h13 = sqrt(q)*sin(dtheta);
    h14 = sqrt(q)/q*delta(1)*cos(dtheta)+sqrt(q)/q*delta(2)*sin(dtheta);
    h15 = sqrt(q)/q*delta(2)*cos(dtheta)-sqrt(q)/q*delta(1)*sin(dtheta);
    h21 = -sqrt(q)/q*delta(1)*sin(dtheta)+sqrt(q)/q*delta(2)*cos(dtheta);
    h22 = -sqrt(q)/q*delta(2)*sin(dtheta)-sqrt(q)/q*delta(1)*cos(dtheta);
    h23 = -sqrt(q)*cos(dtheta);
    h24 = sqrt(q)/q*delta(1)*sin(dtheta)-sqrt(q)/q*delta(2)*cos(dtheta);
    h25 = sqrt(q)/q*delta(2)*sin(dtheta)+sqrt(q)/q*delta(1)*cos(dtheta);
    H = [h11 h12 h13 h14 h15;h21 h22 h23 h24 h25];

    F = [eye(3) zeros(3, 20);
         zeros(2,3) zeros(2, 2*idx-2) eye(2) zeros(2, 20-2*idx)];
    H = H*F;

    % Kalman update equations
    K = cov*H'*inv(H*cov*H'+R);
    pose = pose + K*(z - z_predict);
    cov = (eye(23) - K*H)*cov;
end