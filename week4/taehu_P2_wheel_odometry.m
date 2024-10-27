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
S0 = diag([0.1^2, 0.1^2, (0.02*DTOR)^2]); % 초기 위치의 공분산 행렬
Se = diag([0.02^2, 0.02^2]); % 엔코더의 불확실성 (예시)

% 상태 저장을 위한 변수 (셀 배열 사용)
pose_global = cell(size(enc_cnt,1),1);
S_global = cell(size(enc_cnt,1),1);

pose_global{1} = p0;
S_global{1} = S0;

%% problem 1, 2, 3
for k = 2:size(enc_cnt,1)
    % 휠 오도메트리로부터 상대적인 이동 계산
    dL = ((enc_cnt_L(k)-enc_cnt_L(k-1)) / ENCODER_RESOLUTION * WHEEL_DIAMETER * pi);
    dR = ((enc_cnt_R(k)-enc_cnt_R(k-1)) / ENCODER_RESOLUTION * WHEEL_DIAMETER * pi);
    dist = (dL + dR) / 2;
    
    dth = (dR - dL) / VEHICLE_TREAD; % 회전각 변화
    dx = dist * cos(dth);
    dy = dist * sin(dth);
    
    X_jk = [dx, dy, dth]';  % 상대적인 이동 변위
   
    % 오도메트리 야코비안 행렬 계산
    a = [cos(dth), 0;
         sin(dth), 0;
         0,        1];

    b = [1/2, 1/2;
         1/VEHICLE_TREAD, -1/VEHICLE_TREAD];
    
    J_odo = a * b;
   
    % 오도메트리 불확실성 갱신
    S_jk = J_odo * Se * J_odo';

    % 이전 로봇 상태 불러오기
    X_0j = pose_global{k-1};
    S_0j = S_global{k-1};

    % head2tail_2d 함수 사용하여 새로운 로봇 상태 계산
    [X_0k, J] = head2tail_2d(X_0j, X_jk);
    
    J_1 = J(1:3, 1:3);  % 필요한 상위 3x3 야코비안 추출
    
    % 공분산 행렬 갱신
    S_0k = J_1 * S_0j * J_1' + S_jk;  % 불확실성 전파

    % 새로운 상태 저장
    pose_global{k} = X_0k';
    S_global{k} = S_0k;
end

%% 최종 상태에서 본 초기 위치의 불확실성 계산
X_f = pose_global{end};  % 최종 로봇 상태
Cov_f = S_global{end};   % 최종 상태의 공분산
theta_f = X_f(3);        % 최종 로봇의 회전각

% 좌표계 변환 야코비안 (역변환)
J_ominus = [ cos(theta_f), sin(theta_f), 0;
            -sin(theta_f), cos(theta_f), 0;
             0,            0,          -1 ];

% 초기 상태를 최종 상태 기준으로 변환한 위치와 회전
X_fi = [-X_f(1)*cos(theta_f) - X_f(2)*sin(theta_f);
         X_f(1)*sin(theta_f) - X_f(2)*cos(theta_f);
        -X_f(3)]';

% 초기 상태에 대한 공분산을 최종 상태에서 본 관점으로 변환
Cov_fi = J_ominus * Cov_f * J_ominus';

% 결과 출력
disp('최종 로봇 포즈에서 본 초기 위치의 공분산:');
disp(Cov_fi);

% incremental plot
figure(1); 
for k = 1:length(pose_global)
    scale = 2;
    plot_triangle(pose_global{k}', scale);  hold on; % 로봇 위치 시각화
    
    if k == size(enc_cnt, 1)
        plot_ellipse(pose_global{k}(1:2)', S_global{k}(1:2, 1:2), 'b'); % 마지막 위치에서 파란색 표시
    else
        plot_ellipse(pose_global{k}(1:2)', S_global{k}(1:2, 1:2), 'r'); % 나머지는 빨간색 표시
    end
    
    xlabel('x'); ylabel('y');
    axis([-120 120 -10 225])
    grid on;
    drawnow;  
    pause(0.01);  
end
