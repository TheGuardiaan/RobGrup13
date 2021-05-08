%% Point Cloud manipulation example
clear

load('PointCloud2Data.mat')

scatter3(pcd)

% read RGB
xrgb=reshape(pcd.readRGB(),640,480,3);
xrgb=permute(xrgb, [2 1 3]); % re-arrange from BGR to RGB
figure
imshow(xrgb)

% read XYZ format
xyz = pcd.readXYZ();
figure
scatter3(xyz(:,1),xyz(:,2), xyz(:,3))

xyzc = xyz(~isnan(xyz(:,3)),:); % remove NaN

%% kmeans for clustering

K = 20;
[idx,C,sumd] = kmeans(xyzc', K, 'Display', 'iter'); 
%C = C'

figure
scatter3(xyz(:,1),xyz(:,2), xyz(:,3), '.')
hold on
for i=1:K,
    plot3(C(1,i), C(2,i), C(3,i), 'r.')
end

figure
for i=1:K
    scatter3(xyzc(idx==i,1),xyzc(idx==i,2), xyzc(idx==i,3), '.')
    hold on
end


%% Plane fitting on single cluster

M = 6; % cluster id
figure
scatter3(xyzc(idx==M,1),xyzc(idx==M,2), xyzc(idx==M,3), '.')


P = xyzc(idx==M,:)';    % Plane fitting..
N = size(P,2); % number points

x0 = mean(P')

figure, scatter3(pcd)

% SIMILAR TO Plane_fitting_example_CORKE.m
P0 = P - repmat(x0', 1, N);
w = ones(1,N); % Weights - could be different..
J = (repmat(w,3,1).*P0)*P0';
J  % show J

[v,d] = eig(J)  % eigen-vec/values

n = v(:,1)'% eigenvector corresponding to lowest eigenvalue..
figure
scatter3(P(1,:), P(2,:), P(3,:), '.')
hold on
nvec = [x0 ; x0 + 0.5*n]; % normal vector for plotting (from mean to mean+normvec)
line(nvec(:,1), nvec(:,2), nvec(:,3), 'Color', 'r')

%% The alternative..

help pcfitplane



