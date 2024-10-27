function propagate_ekf (dx, dy, dt, sx2, sy2, st2)

global pose cov

x_current = pose(1:3);
S_current = cov(1:3,1:3);

u = [dx,dy,dt]';
Q = diag([sx2,sy2,st2]);

% TODO what are
[x_next,J] = head2tail_2d(x_current, u);
S_next = J(:,1:3)*S_current*J(:,1:3)'+J(:,4:6)*Q*J(:,4:6)';

% plug in back to pose
pose(1:3) = x_next;
cov(1:3,1:3) = S_next;