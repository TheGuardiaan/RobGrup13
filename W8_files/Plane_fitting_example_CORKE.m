%% CORKE Plane fitting - sec. 14.3.8
clear

mydir = pwd;
cd('U:\Kurser_undervisning\ITROB2\CORKE_robotics_toolbox\rvctools') % ROB toolbox + Machine vision !
startup_rvc
cd(mydir)

%% Plane fitting
T = transl(1,2,3)*rpy2tr(pi/2, pi/8, 0); % rot.mat.
P = mkgrid(10, 1, T); % create points
N = size(P,2); % number points

P = P + 0.02*randn(size(P));
scatter3(P(1,:), P(2,:), P(3,:)), axis([0 4 0 4 0 4])

x0 = mean(P')
P0 = P - repmat(x0', 1, N); % remove mean

w = ones(1,N); % Weights - could be different..
J = (repmat(w,3,1).*P0)*P0';
J  % show J

[v,d] = eig(J)  % eigen-vec/values

n = v(:,1)'% eigenvector corresponding to lowest eigenvalue..
figure
scatter3(P(1,:), P(2,:), P(3,:)), axis([0 4 0 4 0 4])
hold on
nvec = [x0 ; x0 + 0.5*n]; % normal vector for plotting (from mean to mean+normvec)
line(nvec(:,1), nvec(:,2), nvec(:,3), 'Color', 'r')

