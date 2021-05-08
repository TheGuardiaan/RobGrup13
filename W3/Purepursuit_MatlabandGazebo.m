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
path = [1.00    0.00;
        2.00    0.00;
        2.00    5.00;
        0.00    5.00;
        0.00   1.00];
    
%robotInitialLocation = path(1,:);
robotGoal = path(end,:);  

initialOrientation = 0;


odomdata = receive(odom,3);
pose = odomdata.Pose.Pose;
x = pose.Position.X;
y = pose.Position.Y;
theta = pose.Orientation.Z;

test = euler(quaternion([pose.Orientation.X pose.Orientation.Y pose.Orientation.Z pose.Orientation.W]), 'ZYX', 'frame');
theta = test(3);

robotInitialLocation = [x, y]
robotCurrentPose = [robotInitialLocation initialOrientation]';


%%

controller = controllerPurePursuit;
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.6;
controller.MaxAngularVelocity = 2;
controller.LookaheadDistance = 5;


goalRadius = 0.1;
distanceToGoal = norm(robotInitialLocation - robotGoal);

%%% Initialize the simulation loop
sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);

% Initialize the figure
%figure

% Determine vehicle frame size to most closely represent vehicle with plotTransforms
%frameSize = robot_Sim.TrackWidth/0.8;

disp("goalRadius");
        disp(goalRadius);

%%
while( distanceToGoal > goalRadius )
    
    [v, w] = controller([x, y, theta]);

    velmsg.Angular.Z = w;   % Angular velocity (rad/s)
    velmsg.Linear.X = v;    % Linear velocity (m/s), +forward,-reverse

    send(robot,velmsg);
    
    odomdata = receive(odom,3);
    pose = odomdata.Pose.Pose;
    x = pose.Position.X;
    y = pose.Position.Y;
    theta = pose.Orientation.Z;

    test = euler(quaternion([pose.Orientation.X pose.Orientation.Y pose.Orientation.Z pose.Orientation.W]), 'ZYX', 'frame');
    theta = test(3);

    robotCurrentPose = controller([x, y ,theta]);

    
    
    
    disp("distanceToGoal");
        disp(distanceToGoal);    
        
    % Update the plot
    hold off

    % Plot the path of the robot as a set of transforms
    
    plot(x, y, 'x');
    xlim([0 25])
    ylim([0 25])

    pause(.1);
    
end
    
