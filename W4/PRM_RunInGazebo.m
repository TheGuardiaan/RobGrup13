%% Restart connection
rosshutdown
%% 
%------------ Connect to Robot i VM ---------------
%

clear
clc
VmIp = 'http://192.168.174.128:11311';
MyIp = '192.168.0.28';


setenv('ROS_MASTER_URI',VmIp);
% assuming your own ip is 192.168.1.100
setenv('ROS_IP',MyIp);
rosinit(VmIp,'NodeHost',MyIp);


robot = rospublisher('/mobile_base/commands/velocity');
velmsg = rosmessage(robot);
odom = rossubscriber('/odom'); %Scriber til topic for at for roboten's koordinater



%%
%Using the Path Following Controller Along with PRM
%If the desired set of waypoints are computed by a path planner, the path following controller can be used in the same fashion. First, visualize the map
load exampleMaps.mat;
map = binaryOccupancyMap(simpleMap);
%show(map)

%%
%You can compute the path using the PRM path planning algorithm. See  Path Planning in Environments of Different Complexity for details.
mapInflated = copy(map);
%inflate(mapInflated, robot.TrackWidth/2);
prm = robotics.PRM(mapInflated);
prm.NumNodes = 100;
prm.ConnectionDistance = 10;


%%
%Find a path between the start and end location. Note that the path will be different due to the probabilistic nature of the PRM algorithm.
startLocation = [4.0 3.0];
endLocation = [12.0 2.0];
path = findpath(prm, startLocation, endLocation);

%Display the inflated map, the road maps, and the final path.
figure(1)
show(prm);

%%
%Reset robot poisrion
ResetPostrion =  rospublisher('/mobile_base/commands/reset_odometry');
restmsg = rosmessage(ResetPostrion);
send(ResetPostrion, restmsg);


%Substriber til robot for at får kondinater
odom = rossubscriber('/odom');
odomdata = receive(odom,3); %modtag indenfor 3 sekunder -> ellers fejl
pose = odomdata.Pose.Pose;
x = pose.Position.X;
y = pose.Position.Y;
z = pose.Orientation.Z;
w = pose.Orientation.W;


eulerOrientation = euler(quaternion([pose.Orientation.X pose.Orientation.Y pose.Orientation.Z pose.Orientation.W]), 'ZYX', 'frame');
theta = eulerOrientation(3);

GazeboInitialLocation = [x,y];
GazeboInitialOrientation = theta;

GazeboCurrentPose = [GazeboInitialLocation GazeboInitialOrientation]';

GazeboGoal = endLocation;

%%
%Rob i Gazebo controller
controller = controllerPurePursuit;
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.6;
controller.MaxAngularVelocity = 1.5;
controller.LookaheadDistance = 0.8;

goalRadius = 5;
distanceToGoal = norm(GazeboInitialLocation - GazeboGoal);


%% Initialize the simulation loop
sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);

robotPub = rospublisher('/mobile_base/commands/velocity');

figure(2)

while( distanceToGoal > goalRadius )   
    % Compute the controller outputs, i.e., the inputs to the robot
    %[v, omega] = controller(GazeboCurrentPose);    
    [v, omega] = controller([x, y, theta]);
        
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
    %z_new = pose.Orientation.Z;
    %w_new = pose.Orientation.W;

    eulerOrientation = euler(quaternion([pose.Orientation.X pose.Orientation.Y pose.Orientation.Z pose.Orientation.W]), 'ZYX', 'frame');
    theta = eulerOrientation(3);
        
    GazeboCurrentPose = [x, y, theta];          
   
    % Re-compute the distance to the goal
    distanceToGoal = norm(GazeboCurrentPose(1:2) - GazeboGoal(:)');      
        
    disp("distanceToGoal: ");
        disp(distanceToGoal)
        
    disp("GazeboCurrentPose: ");
        disp(GazeboCurrentPose)
                   
        
    % Update the plot
    hold off

    % Plot the path of the robot as a set of transforms
    
    plot(x, y, 'x');

    xlim([0 25])
    ylim([0 25])
        
    waitfor(vizRate);
    %pause(.1);
end
