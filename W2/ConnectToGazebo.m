clear
clc



%%
clear
clc

roslaunch turtlebot_gazebo turtlebot_world.launch


%% Define a set of waypoints 
clear
clc
%% for the desired path for the robot
path = [0.00    0.00;
        2.00    0.00;
        2.00    5.00;
        0.00    5.00;
        0.00    0.00;
        ];
%% Set the current location and the goal location of the robot as defined by the path.
robotInitialLocation = path(1,:);
robotGoal = path(end,:);

%% Assume an initial robot orientation (the robot orientation is the angle between the robot heading and the positive X-axis, measured counterclockwise).
initialOrientation = 0;
    
%% Define the current pose for the robot [x y theta]
robotCurrentPose = [robotInitialLocation initialOrientation]';

%%
%% Initialize the robot model and assign an initial pose. 
    %The simulated robot has kinematic equations for the motion of a two-wheeled differential drive robot. 
    %The inputs to this simulated robot are linear and angular velocities.
clc  

TrackWidth = 0.2;
VehicleInputs = 0.05;
VehicleSpeedHeadingRate = 0.2;
    
    
%robot = differentialDriveKinematics(TrackWidth, 1, VehicleInputs, VehicleSpeedHeadingRate);    
robot = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");


%% Visualize the desired path
figure
plot(path(:,1), path(:,2),'k--d')
xlim([0 13])
ylim([0 13])

%% Based on the path defined above and a robot motion model, you need a path following controller to drive the robot along the path. Create the path following controller using the controllerPurePursuit object.
controller = controllerPurePursuit;

%% Use the path defined above to set the desired waypoints for the controller

controller.Waypoints = path;

%% Set the path following controller parameters. The desired linear velocity is set to 0.6 meters/second for this example.

controller.DesiredLinearVelocity = 0.6;

%% The maximum angular velocity acts as a saturation limit for rotational velocity, which is set at 2 radians/second for this example.

controller.MaxAngularVelocity = 2;

%% As a general rule, the lookahead distance should be larger than the desired linear velocity for a smooth path. The robot might cut corners when the lookahead distance is large. In contrast, a small lookahead distance can result in an unstable path following behavior. A value of 0.3 m was chosen for this example.

controller.LookaheadDistance = 0.3;

%% The path following controller provides input control signals for the robot, which the robot uses to drive itself along the desired path.
% Define a goal radius, which is the desired distance threshold between the robot's final location and the goal location. Once the robot is within this distance from the goal, it will stop. Also, you compute the current distance between the robot location and the goal location. This distance is continuously checked against the goal radius and the robot stops when this distance is less than the goal radius.
% Note that too small value of the goal radius may cause the robot to miss the goal, which may result in an unexpected behavior near the goal.

goalRadius = 0.1;
distanceToGoal = norm(robotInitialLocation - robotGoal);

%% The controllerPurePursuit object computes control commands for the robot. Drive the robot using these control commands until it reaches within the goal radius. If you are using an external simulator or a physical robot, then the controller outputs should be applied to the robot and a localization system may be required to update the pose of the robot. The controller runs at 10 Hz.
% Initialize the simulation loop
sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);

% Initialize the figure
figure

% Determine vehicle frame size to most closely represent vehicle with plotTransforms
frameSize = robot.TrackWidth/0.8;

while( distanceToGoal > goalRadius )
    
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robotCurrentPose);
    
    % Get the robot's velocity using controller inputs
    vel = derivative(robot, robotCurrentPose, [v omega]);
    
    % Update the current pose
    robotCurrentPose = robotCurrentPose + vel*sampleTime; 
    
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    
    % Update the plot
    hold off
    
    % Plot path each instance so that it stays persistent while robot mesh
    % moves
    plot(path(:,1), path(:,2),"k--d")
    hold all
    
    % Plot the path of the robot as a set of transforms
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
    light;
    xlim([0 13])
    ylim([0 13])
    
    waitfor(vizRate);
end



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

%%
odomdata = receive(odom,3);
pose = odomdata.Pose.Pose;
x = pose.Position.X;
y = pose.Position.Y;
z = pose.Position.Z;



robotInitialLocation = [pose.Position.X, pose.Position.Y, pose.Position.Z];
robotGoal = [pose.Position.X+1, pose.Position.Y, pose.Position.Z];

initialOrientation = 0;

robotCurrentPose = [robotInitialLocation initialOrientation]';

disp("robotInitialLocation");
    disp(robotInitialLocation);
disp("robotGoal");
    disp(robotGoal);
disp("robotCurrentPose");
    disp(robotCurrentPose);

goalRadius = 0.1;
distanceToGoal = norm(robotInitialLocation - robotGoal);



%%

while( distanceToGoal > goalRadius )
    odomdata = receive(odom,3);
pose = odomdata.Pose.Pose;
x = pose.Position.X;
y = pose.Position.Y;
z = pose.Position.Z;
   

    % Update the current pose
    robotCurrentPose = [x,y,z];
    
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
   
    %disp(distanceToGoal);
    disp(y);
    
    
    velmsg = rosmessage(robot);
    velmsg.Angular.Z = 0.0;	% Angular velocity (rad/s)
    velmsg.Linear.X = 0.1; % Linear velocity (m/s), +forward,-reverse
    send(robot,velmsg); 
    

end
%%
clc

disp("robotCurrentPose");
    disp(robotCurrentPose);
    
disp("robotGoal");
    disp(robotGoal);
    
disp("distanceToGoal");    
    disp(distanceToGoal);
    
disp("goalRadius");    
    disp(goalRadius);
%%
s = 100;
for c = 1:s
   
    velmsg = rosmessage(robot);
    velmsg.Angular.Z = 0.0;	% Angular velocity (rad/s)
    velmsg.Linear.X = 0.5; % Linear velocity (m/s), +forward,-reverse
    send(robot,velmsg);
    
    
odomdata = receive(odom,3);
pose = odomdata.Pose.Pose;
x = pose.Position.X;
y = pose.Position.Y;
z = pose.Position.Z;


disp(x)
disp(y)
end
