%PRM exercise mandatory

%Restart robot
rosshutdown
%% ROS Initialization
clear
clc
VmIp = 'http://192.168.174.128:11311';
MyIp = '192.168.0.28';


setenv('ROS_MASTER_URI',VmIp);
% assuming your own ip is 192.168.1.100
setenv('ROS_IP',MyIp);
rosinit(VmIp,'NodeHost',MyIp);

%%
%Load map
load exampleMaps.mat
map = binaryOccupancyMap(simpleMap,2);

%%
%Create a simple roadmap with 50 nodes
prmSimple = mobileRobotPRM(map,3);
figure(1)
show(prmSimple)

%Calculate a simple path:
startLocation = [2 1];
endLocation = [12 10];
path = findpath(prmSimple,startLocation,endLocation);

show(prmSimple)

%%
resetRobotPose = rospublisher('/mobile_base/commands/reset_odometry');
msg = rosmessage(resetRobotPose);
send(resetRobotPose, msg)

odom = rossubscriber('/odom');
odomdata = receive(odom,3); %modtag indenfor 3 sekunder -> ellers fejl
pose = odomdata.Pose.Pose;
x = pose.Position.X;
y = pose.Position.Y;

eulerOrientation = euler(quaternion([pose.Orientation.X pose.Orientation.Y pose.Orientation.Z pose.Orientation.W]), 'ZYX', 'frame');
theta = eulerOrientation(3);

GazeboInitialLocation = [x, y]

GazeboCurrentPose = [x, y, theta]  

GazeboGoal = endLocation

%%
%Simulering i Gazebo
controller = controllerPurePursuit;
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.3;
controller.MaxAngularVelocity = 1;
controller.LookaheadDistance = 0.4;

goalRadius = 0.1;
distanceToGoal = norm(GazeboInitialLocation - GazeboGoal');

%% Initialize the simulation loop
%%% Initialize the simulation loop
sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);

% Initialize the figure
%figure(2)

robotPub = rospublisher('/mobile_base/commands/velocity');

while( distanceToGoal > goalRadius )
       
    % Compute the controller outputs, i.e., the inputs to the robot
    %[v, omega] = controller(GazeboCurrentPose);    
    [v, omega] = controller([x, y, theta]);
    
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
    
    % Update the plot
    hold on
    
    % Plot the path of the robot as a set of transforms
    plot(x, y, 'g--X');
   
    xlim([0 14])
    ylim([0 14])
    
    % Re-compute the distance to the goal
    distanceToGoal = norm(GazeboCurrentPose(1:2) - GazeboGoal(:)');      
    
    disp(distanceToGoal)
    waitfor(vizRate);
end