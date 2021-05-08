% Examples of PRM methods
clear

mydir = pwd;
cd('U:\Kurser_undervisning\ITROB2\CORKE_robotics_toolbox\rvctools') % ROB toolbox + Machine vision !
startup_rvc
cd(mydir)

%% Initialization
load map1
start = [20, 10];
goal = [40, 40];

% modify map
map2 = map;
map2(60:90, 40:50) = 1;
figure
imagesc(map2), colorbar
hold on, plot(start(1), start(2), 'r*'), text(start(1), start(2), 'START')
hold on, plot(goal(1), goal(2), 'ro'), text(goal(1), goal(2), 'GOAL')

%% Probabilistic Roadmap method

prm = PRM(map)
prm.plan('npoints', 100)  % planning
prm.plot() 

prm.query(start, goal)
prm.plot()

% unconnected problems (sometimes..)
goal2 = [2, 30];
prm.query(start, goal2)
prm.plot()

% rerun with more nodes - or just rerun..
prm2 = PRM(map);
prm2.plan('npoints', 300)  % planning
prm2.query(start, goal2)
prm2.plot()
