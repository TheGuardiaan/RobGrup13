%%  
rosshutdown
clear
clc
VmIp = 'http://192.168.174.129:11311';
MyIp = '192.168.0.28';


setenv('ROS_MASTER_URI',VmIp);
% assuming your own ip is 192.168.1.100
setenv('ROS_IP',MyIp);
rosinit(VmIp,'NodeHost',MyIp);


%%
image = imread('Shannon00.jpg');
grayimage = rgb2gray(image);
bwimage = grayimage < 0.5;

map = binaryOccupancyMap(bwimage,21)
%Reslusion til 21 for at scalere 2D map til 3D map som cirka er 54m
show(map)

%%
%Calculate a simple path:
punkt_A = [0 0]; %Testpunkt
%punkt_A = [46.5 29];

punkt_B = [5.5 9.5];
punkt_C = [27.5 2.5];


path = findpathFunc(punkt_A, punkt_B, map);
controller = setController(path);


%path = findpathFunc(punkt_B, punkt_C);




%%
resetRobotPose = rospublisher('/mobile_base/commands/reset_odometry');
msg = rosmessage(resetRobotPose);
send(resetRobotPose, msg)

%Set new startpose - Virker ikke - Prøv med - "Localize TurtleBot Using Monte Carlo Localization"
%robotInitialLocation = punkt_A;
%initialOrientation = 0;

%GazeboInitialLocation = robotInitialLocation;
%GazeboCurrentPose = [robotInitialLocation initialOrientation];


%%
odom = rossubscriber('/odom');
odomdata = receive(odom,3); %modtag indenfor 3 sekunder -> ellers fejl
pose = odomdata.Pose.Pose;

x = pose.Position.X;
y = pose.Position.Y;

eulerOrientation = euler(quaternion([pose.Orientation.X pose.Orientation.Y pose.Orientation.Z pose.Orientation.W]), 'ZYX', 'frame');
theta = eulerOrientation(3);

GazeboInitialLocation = [x, y]
GazeboCurrentPose = [x, y, theta]  


%%
goalRadius = 0.1;
GazeboGoal = punkt_B
distanceToGoal = norm(GazeboInitialLocation - GazeboGoal');

%% Initialize the simulation loop
%%% Initialize the simulation loop
sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);

% Initialize the figure
%figure(2)

robotPub = rospublisher('/mobile_base/commands/velocity');

% Send message to robot i Gazebo
velmsg = rosmessage(robotPub);

velmsg.Angular.Z = 0;	% Angular velocity (rad/s)
velmsg.Linear.X = 1; % Linear velocity (m/s), +forward,-reverse
send(robotPub,velmsg);

while( distanceToGoal > goalRadius )
    
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(GazeboCurrentPose);    
    %[v, omega] = controller([x, y, theta]);
    
    %disp(v)
    %disp(omega)
    
    % Send message to robot i Gazebo
    velmsg = rosmessage(robotPub);

    velmsg.Angular.Z = omega;	% Angular velocity (rad/s)
    velmsg.Linear.X = v; % Linear velocity (m/s), +forward,-reverse
    send(robotPub,velmsg);

    % Get the robot's velocity using controller inputs
    %vel = derivative(robot, robotCurrentPose, [v omega]);
    
    % Update the current pose
    odom = rossubscriber('/odom');
    odomdata = receive(odom,3); %modtag indenfor 3 sekunder -> ellers fejl
    pose = odomdata.Pose.Pose;
    x = pose.Position.X;
    y = pose.Position.Y;       
           
    eulerOrientation = euler(quaternion([pose.Orientation.X pose.Orientation.Y pose.Orientation.Z pose.Orientation.W]), 'ZYX', 'frame');
    theta = eulerOrientation(3);
        
    GazeboCurrentPose = [x, y, theta]         
    %disp("x - y - theta")
    
    % Update the plot
    hold on
    
    % Plot the path of the robot as a set of transforms
    plot(x, y, 'g--X');
   
    xlim([0 62])
    ylim([0 30])
    
    % Re-compute the distance to the goal
    distanceToGoal = norm(GazeboCurrentPose(1:2) - GazeboGoal(:)');      
    
    disp("DistanceToGoal: " + distanceToGoal)
    waitfor(vizRate);
end

%Create a simple roadmap with 50 nodes
function path = findpathFunc(start_punkt, end_punkt, map)
    nodes = 250;
    prmSimple = mobileRobotPRM(map,nodes);

    path = findpath(prmSimple,start_punkt,end_punkt);
    figure(1)
    show(prmSimple)
        
end


function controllerReturn = setController(path)
    %Simulering i Gazebo
    controller = controllerPurePursuit;
    controller.Waypoints = path;
    controller.DesiredLinearVelocity = 0.2;
    controller.MaxAngularVelocity = 0.5;
    controller.LookaheadDistance = 0.4;
    
    controllerReturn = controller;
end