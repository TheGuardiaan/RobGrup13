%%  rostopic list
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
image = imread('Shannon.jpg');


grayimage = rgb2gray(image);
bwimage = grayimage < 0.5;


map = binaryOccupancyMap(bwimage,20.8);
inflate(map, 0.2);

show(map)

robotPub = rospublisher('/mobile_base/commands/velocity');
odom = rossubscriber('/odom');

MonteCarlo_Localization_Algorithm(map, robotPub);




%%
function MonteCarlo_Localization_Algorithm(map, robotPub)
    odometryModel = odometryMotionModel;
    odometryModel.Noise = [0.2 0.2 0.2 0.2];

    rangeFinderModel = likelihoodFieldSensorModel;
    rangeFinderModel.SensorLimits = [0.45 8];
    rangeFinderModel.Map = map;

    % Query the Transformation Tree (tf tree) in ROS.  -- tftree.AvailableFrames
    tftree = rostf;
    waitForTransform(tftree,'/base_link', '/camera_link');
    sensorTransform = getTransform(tftree,'/base_link', '/camera_link');
    % Get the euler rotation angles.
    laserQuat = [sensorTransform.Transform.Rotation.W sensorTransform.Transform.Rotation.X ...
        sensorTransform.Transform.Rotation.Y sensorTransform.Transform.Rotation.Z];
    laserRotation = quat2eul(laserQuat, 'ZYX');

    % Setup the |SensorPose|, which includes the translation along base_link's
    % +X, +Y direction in meters and rotation angle along base_link's +Z axis
    % in radians.
    rangeFinderModel.SensorPose = ...
        [sensorTransform.Transform.Translation.X sensorTransform.Transform.Translation.Y laserRotation(1)];

    laserSub = rossubscriber('/scan');
    odomSub = rossubscriber('/odom');

    [velPub,velMsg] = ...
        rospublisher('/cmd_vel','geometry_msgs/Twist');

    % Initialize AMCL Object
    amcl = monteCarloLocalization;
    amcl.UseLidarScan = true;

    amcl.MotionModel = odometryModel;
    amcl.SensorModel = rangeFinderModel;

    amcl.UpdateThresholds = [0.2,0.2,0.2];
    amcl.ResamplingInterval = 1;


    amcl.GlobalLocalization = true;
    amcl.ParticleLimits = [500 50000];

    %amcl.ParticleLimits = [500 5000];
    %amcl.GlobalLocalization = false;

    amcl.InitialPose = ExampleHelperAMCLGazeboTruePose;
    amcl.InitialCovariance = eye(3)*0.5;


    visualizationHelper = ExampleHelperAMCLVisualization(map);
  
    wanderHelper = ...
        ExampleHelperAMCLWanderer(laserSub, sensorTransform, velPub, velMsg);
    
    numUpdates = 60;
    i = 0;
    while i < numUpdates
        % Receive laser scan and odometry message.
        scanMsg = receive(laserSub);
        odompose = odomSub.LatestMessage;

        % Create lidarScan object to pass to the AMCL object.
        scan = lidarScan(scanMsg);

        % For sensors that are mounted upside down, you need to reverse the
        % order of scan angle readings using 'flip' function.

        % Compute robot's pose [x,y,yaw] from odometry message.
        odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
            odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
        odomRotation = quat2eul(odomQuat);
        pose = [odompose.Pose.Pose.Position.X, odompose.Pose.Pose.Position.Y odomRotation(1)];

        % Update estimated robot's pose and covariance using new odometry and
        % sensor readings.
        [isUpdated,estimatedPose, estimatedCovariance] = amcl(pose, scan);

        % Drive robot to next pose.
        wander(wanderHelper);

        % Plot the robot's estimated pose, particles and laser scans on the map.
        if isUpdated
            i = i + 1;
            plotStep(visualizationHelper, amcl, estimatedPose, scan, i)
        end

       turn15Degre(robotPub);

    end

end
%% Final_projekt ----------
function turn15Degre(robotPub)
    for i = 1:5 % Turn the robot 15 degrees to the left
        sendVelmsgRob(0, -0.5, robotPub);
        pause(0.2);
    end 
end

function drive10Cm(robotPub)
    sendVelmsgRob(0.5, 0, robotPub);
    pause(0.5);
    %Stop Rob
    sendVelmsgRob(0, 0, robotPub);
end


function sendVelmsgRob(Linear, Angular, robotPub)
    %disp("Run sendVelmsgRob Func")
    % Send message to robot i Gazebo
    velmsg = rosmessage(robotPub);
    velmsg.Angular.Z = Angular;	% Angular velocity (rad/s)
    velmsg.Linear.X = Linear; % Linear velocity (m/s), +forward,-reverse
    send(robotPub,velmsg);   
end