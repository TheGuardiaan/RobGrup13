% Examples on Distance transform and D-star
clear

mydir = pwd;
cd('U:\Kurser_undervisning\ITROB2\CORKE_robotics_toolbox\rvctools') % ROB toolbox + Machine vision !
startup_rvc
cd(mydir)

%% Initialization
load map1
start = [20, 10];
goal = [40, 40];

imagesc(map), colorbar
hold on, plot(start(1), start(2), 'r*'), text(start(1), start(2), 'START')
hold on, plot(goal(1), goal(2), 'ro'), text(goal(1), goal(2), 'GOAL')

% modify map
map2 = map;
map2(60:90, 40:50) = 1;
figure
imagesc(map2), colorbar
hold on, plot(start(1), start(2), 'r*'), text(start(1), start(2), 'START')
hold on, plot(goal(1), goal(2), 'ro'), text(goal(1), goal(2), 'GOAL')

%% Distance transform
dx = DXform(map2);
dx.plan(goal, 'animate') % planning phase iterations
dx.plot()

dx.query(start, 'animate') % query phase iterations

p = dx.query(start);
figure
dx.plot3d(p) % dist trans plot 3D

dx % dist trans class
dx.distancemap(start)

%% Dstar
ds = Dstar(map2);
tic, ds.plan(goal); toc
ds.plot()
ds.query(start, 'animate')


% modify map
map3 = ds.costmap(); 
subplot(211), imagesc(flipud(map3))
map3(25,60)  % NOTE = Inf
ds.modify_cost([1, 5; 20, 30], Inf); % modify cost - eg. "open a door"
subplot(212), imagesc(flipud(ds.costmap()))


% Re-planning
tic, ds.plan(); toc
ds.plot()
ds.query(start, 'animate')



