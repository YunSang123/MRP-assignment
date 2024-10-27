% Mobile Robot Mapping lecture 02: SVD example
% load point cloud data and display

clear all; close all;

load dat

n = size(dat,1);
dim = size(dat,2);

%% plot
figure(1)
plot3(dat(:,1), dat(:,2), dat(:,3),'.')
hold on;
axis equal


%% select a building
tall_idx = find(dat(:,3)>35);
dist_idx = find(dat(:,1)>450);
idx = intersect(tall_idx, dist_idx);

% we got building points
bldgpts = dat(idx,:);
plot3(bldgpts(:,1), bldgpts(:,2), bldgpts(:,3),'ro');

% Perform SVD on the building points
% [U, S, V] = svd(bldgpts - mean(bldgpts, 1), 0);

% The normal vector to the plane is the last column of V (smallest singular value)
% normal_vector = V(:,3);  % The third column of V is the normal vector

% we got building points
bldgpts = dat(idx,:);
% plot3(bldgpts(:,1), bldgpts(:,2), bldgpts(:,3),'ro');

% fit a plane
plane_center = median(bldgpts);  % You can also use mean(bldgpts) if preferred
plane_size = 20;

% plot_plane(normal_vector, median(bldgpts), plane_size);




% 
% % Normal vector to the plane
% A = normal_vector(1);
% B = normal_vector(2);
% C = normal_vector(3);
% 
% % Plane center (a point on the plane)
% D = -dot(normal_vector, plane_center);  % D = -(A*x_center + B*y_center + C*z_center)
% 
% % Calculate the distance from each point in the point cloud to the plane
% distances = abs(A * dat(:,1) + B * dat(:,2) + C * dat(:,3) + D) / sqrt(A^2 + B^2 + C^2);
% 
% % Display the error (distance) for each point
% figure;
% histogram(distances);
% title('Error (Distance) from Points to Plane');
% xlabel('Distance');
% ylabel('Number of Points');





