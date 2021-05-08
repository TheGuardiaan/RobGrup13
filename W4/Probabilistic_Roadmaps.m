clc
clear

%%
load exampleMaps.mat
map = binaryOccupancyMap(simpleMap,2);
    
%%
prm = mobileRobotPRM(map,250);
%show(prm)

%%
startLocation = [2 1];
endLocation = [12 12];

%%
path = findpath(prm,startLocation,endLocation);
%show(prm)


%%
rngState = rng;

rng(rngState);
prm.ConnectionDistance = 2;
path = findpath(prm,startLocation,endLocation);
%show(prm)
%%
update(prm)
path = findpath(prm,startLocation,endLocation);
%show(prm)
%%
rosshutdown

%%  
clear
clc
VmIp = 'http://192.168.174.128:11311';
MyIp = '192.168.0.28';


setenv('ROS_MASTER_URI',VmIp);
% assuming your own ip is 192.168.1.100
setenv('ROS_IP',MyIp);
rosinit(VmIp,'NodeHost',MyIp);

%rostopic list
   
robot = rospublisher('/mobile_base/commands/velocity');
odom = rossubscriber('/odom');
rossubscriber('/scan');

velmsg = rosmessage(robot);


%%
odomdata = receive(odom,3);
pose = odomdata.Pose.Pose;
x = pose.Position.X;
y = pose.Position.Y;
z = pose.Position.Z;

initpose = [x y z];

%% Path Planning
path = [1 1; 2 1; 3 1; 4 1; 5 1; 5 2; 5 3; 4 3; 3 3; 2 3; 1 3; 1 2];
%plot(path(:,1),path(:,2),'kx')

%% Initialisering
robotInitialLocation = path(1,:);
robotGoal = path(end,:);
initialOrientation = 0;

%% Opdatering af nuværende position/retning
robotCurrentPose = [robotInitialLocation initialOrientation]';

disp("robotCurrentPose");
    disp(robotCurrentPose);

%% Opsætning af robotmodel
%robot = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");
robotRadius = 0.4;
robot = ExampleHelperRobotSimulator('BuildingDefaultName',2);
robot.enableLaser(false);
robot.setRobotSize(robotRadius);
robot.showTrajectory(true);
robot.setRobotPose(robotCurrentPose);

%% Visualize the desired path
figure
plot(path(:,1), path(:,2),'k--d')
xlim([0 13])
ylim([0 13])

%% Robotkontrolleropsætning
controller = controllerPurePursuit;
controller.DesiredLinearVelocity = 0.6;
controller.MaxAngularVelocity = 2;
controller.LookaheadDistance = 0.5 ;
controller.Waypoints = path;
%% Opsætning af målområde
goalRadius = 0.1;
distanceToGoal = norm(robotInitialLocation - robotGoal);
%%
controlRate = rateControl(10);
while (distanceToGoal > goalRadius)

    velmsg = rosmessage(robot);
    velmsg.Angular.Z = 0.0;	% Angular velocity (rad/s)
    velmsg.Linear.X = 0.1; % Linear velocity (m/s), +forward,-reverse
    send(robot,velmsg); 
    
    [v, omega] = controller(robot.getRobotPose);

    drive(robot,v,omega)

    robotCurrentPose = robot.getRobotPose;

    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal);

    waitfor(controlRate);
end