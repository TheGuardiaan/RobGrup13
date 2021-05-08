%% Example of Bug2 algorithm
clear

mydir = pwd;
cd('U:\Kurser_undervisning\ITROB2\CORKE_robotics_toolbox\rvctools') % ROB toolbox + Machine vision !
startup_rvc
cd(mydir)

%% Bug2

load map1
start = [20, 10];
goal = [40, 40];

imshow(map)

bug = Bug2(map)
bug.goal = goal;
bug.verbose = 1;
bug.query(start, goal, 'animate')


hold on, plot(start(1), start(2), 'r*'), text(start(1), start(2), 'START')
hold on, plot(goal(1), goal(2), 'ro'), text(goal(1), goal(2), 'GOAL')


