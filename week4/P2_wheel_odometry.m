clc; clear; close all;

% system config
load('encoder.mat');
enc_cnt_L = enc_cnt(:,3);
enc_cnt_R = enc_cnt(:,4);

VEHICLE_TREAD = 1.65;
WHEEL_DIAMETER = 0.640;           
ENCODER_RESOLUTION = 2048;
DTOR = pi/180;

%% initial value
p0 = [0, 0, 0];
S0 = diag([0.1^2, 0.1^2, (0.02*DTOR)^2]);
Se = diag([0.02^2, 0.02^2]); % encoder uncertainty 0.02 

% we use cell for convinience
pose_global = cell(size(enc_cnt,1),1);
S_global = cell(size(enc_cnt,1),1);

pose_global{1} = p0;
S_global{1} = S0;


%% problem 1, 2, 3
for k = 2:size(enc_cnt,1)
    % calculate relative motion from wheel odometry
    dL = ((enc_cnt_L(k)-enc_cnt_L(k-1)) / ENCODER_RESOLUTION * WHEEL_DIAMETER * pi);
    dR = ((enc_cnt_R(k)-enc_cnt_R(k-1)) / ENCODER_RESOLUTION * WHEEL_DIAMETER * pi);
    dist = (dL+dR)/2;
    
    dth = (dR - dL) / VEHICLE_TREAD;
    dx = dist * cos (dth);
    dy = dist * sin (dth); 
      
    % robot starts from initial pose (global coordinate = initial
    % coordinate)
    % hint: pose_global kth pos represented from global coordinate
    %       S_jk should be calculated from Se (previous lecture)
    X_jk = [dx, dy, dth];
   J_odo = [0.5*cos(dth) 0.5*cos(dth); 0.5*sin(dth) 0.5*sin(dth); 1/VEHICLE_TREAD -1/VEHICLE_TREAD];
    S_jk = J_odo*Se*transpose(J_odo);
    
    X_0j = pose_global{k-1};
    S_0j = S_global{k-1};
    
    [X_0k, J] = head2tail_2d(X_0j,X_jk)
    S_0k = J*[S_0j zeros(3,3);zeros(3,3) S_jk]*transpose(J)
    pose_global{k} = X_0k';
    S_global{k} = S_0k;

end

x_0k = X_0k(1); % X_0k is the final state
y_0k = X_0k(2);
theta_0k = X_0k(3);

J_ominus = [-cos(theta_0k) -sin(theta_0k) x_0k*sin(theta_0k)-y_0k*cos(theta_0k);
            sin(theta_0k) -cos(theta_0k) x_0k*cos(theta_0k)+y_0k*sin(theta_0k);
            0 0 -1]
S_k0 = J_ominus * S_0k * J_ominus'

% incremental plot
figure(1); 
for k = 1:length(pose_global)
    % plot(pose_global, pose_global(:,2)); % plot trajector
    scale = 2;
    plot_triangle(pose_global{k}', scale);  hold on;% plot sample position
    
    if k == size(enc_cnt,1)
        plot_ellipse (pose_global{k}(1:2)',S_global{k}(1:2,1:2),'b'); % When final_state, plot uncertainty ellipse as blue line
    else
        plot_ellipse (pose_global{k}(1:2)',S_global{k}(1:2,1:2),'r'); % When final_state, plot uncertainty ellipse as red line
    end
    
%     axis equal; 
    xlabel('x'); ylabel('y');
    axis([-120 120 -10 225])
    grid on
    drawnow;
%     pause(0.02);
    
end
x = cellfun(@(c) c(1), pose_global);
y = cellfun(@(c) c(2), pose_global);

plot(x,y);