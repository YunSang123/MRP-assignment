function [Rotation_matrix] = Rot(axis,angle)
%ROT 이 함수의 요약 설명 위치
%   자세한 설명 위치
if axis == 'z'
    Rotation_matrix = [cos(angle*pi/180) -sin(angle*pi/180) 0;
                       sin(angle*pi/180) cos(angle*pi/180) 0;
                       0 0 1]
elseif axis == 'y'
    Rotation_matrix = [cos(angle*pi/180) 0 sin(angle*pi/180);
                       0 1 0;
                       -sin(angle*pi/180) 0 cos(angle*pi/180)]
else
    Rotation_matrix = [1 0 0;
                       0 cos(angle*pi/180) -sin(angle*pi/180);
                       0 sin(angle*pi/180) cos(angle*pi/180)]
end