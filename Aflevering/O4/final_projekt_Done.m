%% 
rosshutdown
clear
clc
VmIp = 'http://192.168.174.129:11311';
MyIp = '192.168.0.28';

setenv('ROS_MASTER_URI',VmIp);
setenv('ROS_IP',MyIp);
rosinit(VmIp,'NodeHost',MyIp);

%% MapLocalization
imageLocalization = imread('Shannon.jpg');
grayimageLocalization = rgb2gray(imageLocalization);
bwimageLocalization = grayimageLocalization < 0.03;
mapLocalization = binaryOccupancyMap(bwimageLocalization,20.8);
inflate(mapLocalization, 0.2); %inflate map to make walls thicker on map

image = imread('ShannonNew.jpg');
grayimage = rgb2gray(image);
bwimage = grayimage < 0.03;
map = binaryOccupancyMap(bwimage,20.8); %Resolution set to 20,8 to scale map to 3D map (ca. 54m)
inflate(map, 0.1);

robotPub = rospublisher('/mobile_base/commands/velocity');
odom = rossubscriber('/odom');

%% Reset robotpose to [0 0 0] in Gazebo
resetRobotPose = rospublisher('/mobile_base/commands/reset_odometry');
msg = rosmessage(resetRobotPose);
send(resetRobotPose, msg);

%%
%Set coordinates for A, B and C
A = [46.7 28.3];
B = [5 11];
C = [26 2.2];

nodes = 550; %Number of nodes to path finding

% %% Find robotpose with localization
LocalizationPose = MonteCarloLocalization(mapLocalization, robotPub);
LocalizationCoordinates = [LocalizationPose(1), LocalizationPose(2)];
disp("LocalizationPose:" + LocalizationPose);

%% Drive from startposition to A
path_Localization = calculatePath(LocalizationCoordinates, A, map, nodes);
controller = setController(path_Localization);
avoidObstaclesMode = false; %Obstacle detection not activated
drivePath(A, controller, robotPub, odom, LocalizationPose, avoidObstaclesMode);

%Turn robot 180 degrees
turn90DegreR(robotPub);
pause(1);
turn90DegreR(robotPub);

%% Main drive sequence

%Drive from A to B
path = calculatePath(A, B, map, nodes);
controller = setController(path);
avoidObstaclesMode = true; %Obstacle detection activated
drivePath(B, controller, robotPub, odom, LocalizationPose, avoidObstaclesMode);
findGreenDot(robotPub);

disp("---- Completed Path from A to B ---");
pause(1);

%Drive from B to C
path = calculatePath(B, C, map, nodes);
controller = setController(path);
avoidObstaclesMode = false; %Obstacle detection not activated
sendVelmsgRob(0.5, 0, robotPub);
pause(0.5);
drivePath(C, controller, robotPub, odom, LocalizationPose, avoidObstaclesMode);
findGreenDot(robotPub);

disp("---- Completed Path from B to C ---");
pause(1);

%% Functions
function drivePath(gazeboGoal, controller, robotPub, odom, localizationPose, avoidObstaclesMode) 
    GazeboCurrentPose = localizationPose;
    GazeboInitialLocation = localizationPose(1,2);
    
    odomPose = [0,0,0];   
    goalRadius = 0.7;
    distanceToGoal = norm(GazeboInitialLocation - gazeboGoal');
    
    sampleTime = 0.1;
    vizRate = rateControl(1/sampleTime);
    while( distanceToGoal > goalRadius )
    
        %Obstacle Detection 
        if ((distanceToGoal < 8) && (avoidObstaclesMode == true)) %Only enabled before concrete obstacle    
            %pause(1);
            [dist, a] =  scanWorld();
            disp("Dist to Wall: " + dist); 
                       
            while (dist < 1) 
                sendVelmsgRob(0, 0, robotPub);

                avoidObstacles(robotPub); 
                [dist, a] = scanWorld();
            end   
        end
    
        %Update the robotpose based on odometry
        oldPose = odomPose;        
        odomPose = getCurrentPose(odom);
        deltaPose = odomPose - oldPose; %Change in posistion since last iteration in the while-loop
        
        GazeboCurrentPose = GazeboCurrentPose + deltaPose;

        x = GazeboCurrentPose(1);
        y = GazeboCurrentPose(2);
        
        GazeboCurrentPoseXY = [x, y];
               
        %Plot the path of the robot
        hold on
        plot(x,y, 'g--X');
        xlim([0 62]);
        ylim([0 30]);

        %Get the linear- and angular velocity from the controller
        [v, omega] = controller(GazeboCurrentPose);    
        sendVelmsgRob(v,omega, robotPub);
        
        %Recalculate the distance to the goal
        distanceToGoal = norm(GazeboCurrentPoseXY - gazeboGoal(:)');      
        disp("DistanceToGoal: " + distanceToGoal);
        
        waitfor(vizRate);
    end
end

function avoidObstacles(robotPub)
    pause(1);
    scanWorld();
    pause(1);
    [dist, a] = scanWorld();
    disp("Dist to Wall - extra: " + dist); 
   
    
    if (dist < 0.6)
        sendVelmsgRob(-0.7, 0, robotPub);
        pause(1);
        sendVelmsgRob(0, 0, robotPub); %Stop robot
            pause(1);     
    end
    turn90DegreR(robotPub);       
    pause(1);

    sendVelmsgRob(0.8, 0, robotPub);    
    pause(1);

    turn90DegreL(robotPub);
    pause(1);  
end

function [dist, a] = scanWorld()

   if ismember('/scan',rostopic('list'))
        scansub = rossubscriber('/scan');  

        scan = receive(scansub); %Receive message             
        cart = readCartesian(scan); %Extract cartesian coordinates from scan
        
        if (length(cart) >= 200)  %Scan receive enough data to calculate distance
            x = cart(:,2); %X-posistion
            d = cart(:,1); %Depth

            xleft = x(100:200);
            dleft = d(100:200);

            mdl = fitlm(xleft,dleft); %Fit coordinates to line
            coef=mdl.Coefficients.Estimate; %Extract coefficients
            a = coef(2);
            b = coef(1);

            %Calculate distance of the closest obstacle/wall
            dist = abs(b)/(sqrt(a.^2+1));                               
        else       
            dist = 10;
            a = 0;
        end 
   end
end

%Get the current position from odometry
function odomPose = getCurrentPose(odom)
    odomdata = receive(odom,3); %Return error, if data is not received before 3 seconds
    pose = odomdata.Pose.Pose;
    x = pose.Position.X;
    y = pose.Position.Y;       

    eulerOrientation = euler(quaternion([pose.Orientation.X pose.Orientation.Y pose.Orientation.Z pose.Orientation.W]), 'ZYX', 'frame');
    theta = eulerOrientation(3);   

    odomPose = [x, y, theta];  
end

function estimatedPose = MonteCarloLocalization(map, robotPub)   
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

    numUpdates = 34; %Number of scans 
    i = 0;
    while i < numUpdates
        turn35DegreR(robotPub);
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
    end
end

%Create a roadmap and find path
function path = calculatePath(start_punkt, end_punkt, map, nodes)
    disp("Find path!!")
    prm = mobileRobotPRM(map, nodes); %probabilistic road map

    path = findpath(prm,start_punkt,end_punkt);
    figure(1);
    show(prm);    
end

%Create controller
function controllerReturn = setController(path)
    controller = controllerPurePursuit;
    controller.Waypoints = path;
    controller.DesiredLinearVelocity = 0.5;
    controller.MaxAngularVelocity = 0.7;
    controller.LookaheadDistance = 0.4;
    
    controllerReturn = controller;
end

function sendVelmsgRob(Linear, Angular, robotPub)
    %Send message to robot in Gazebo
    velmsg = rosmessage(robotPub);
    velmsg.Angular.Z = Angular;	% Angular velocity (rad/s)
    velmsg.Linear.X = Linear; % Linear velocity (m/s)
    send(robotPub,velmsg);   
end


function findGreenDot(robotPub)
    %disp("Run findGreenDot Func")
    greenDotNotFound = true;
        
    while (greenDotNotFound)           
        foundGreenDot = takePicture(robotPub);
        %takePicture(robotPub);
        for c = 1:0          
            sendVelmsgRob(0, 0.5, robotPub)
            pause(1)           
            %Stop Robot
            sendVelmsgRob(0, 0, robotPub)
        end
                 
        if (foundGreenDot) %Make BIB sound ----                
                for c = 1:3
                    disp("---- !! Bip !! !! Bip !! !! Bip !! ---")
                    pause(0.5)
                    return;
                end            
            greenDotNotFound = false;
        end
    end
end


function foundGreenDot = takePicture(robotPub)
    %takePictureDebug();
    % --- Debug ---
    disp("Take Picture");
    
    % Grap image from camera
    if ismember('/camera/rgb/image_color/compressed',rostopic('list'))
        imsub = rossubscriber('/camera/rgb/image_color/compressed');
    end

    if ismember('/camera/rgb/image_raw',rostopic('list'))
        imsub = rossubscriber('/camera/rgb/image_raw');
    end

    imgraw = receive(imsub); % a serialised image
    img = readImage(imgraw); % decode image    
        
    %figure(4);
    %imshow(img);
    pause(0.5); %Delay

    foundGreenDot = dotFound(img, robotPub);
end

function img = takePictureDebug()
     % --- Debug ---
    %disp("Take Picture For Debug");
     % Grap image from camera
    if ismember('/camera/rgb/image_color/compressed',rostopic('list'))
        imsub = rossubscriber('/camera/rgb/image_color/compressed');
    end

    if ismember('/camera/rgb/image_raw',rostopic('list'))
        imsub = rossubscriber('/camera/rgb/image_raw');
    end
    
    imgraw = receive(imsub); % a serialised image
    img = readImage(imgraw); % decode image  
end


function foundGreenDot = dotFound(RGB, robotPub)      

    figure(3);
    imshow(RGB);
    
    %disp("size(RGB);" +  size(RGB));
    
    %RGB = takePictureDebug();
    %pause(1);
    BW = createMask(RGB);
    figure(4);
    imshow(BW);
      
    stats = regionprops('table',BW,'Centroid');
    
    if(stats.Centroid)   
        
        [circleFound, centersBright] = findCircle(BW);
        if(circleFound) 
        
            disp("!! - Found Green DOT - !!");             
            pause(0.5);       
            foundGreenDot = true;        
            [a, dist] = findWall(robotPub, true); 
            disp("FindWall");
            pause(1);

            %----
            % Find green Dot and drive
            %----
            drivFountOnDot(robotPub, BW, centersBright); 

            %pause(20);
            %Rob køre 40 cm fra væg
            wallPositionClose(dist, robotPub);

            for i = 1:6 %Turn rob 180 degres
                turn90DegreR(robotPub);       
                pause(0.1);
            end
            disp("!!----STOP---!!!" );                            
            pause(2); 
         else
            disp("Not Found Green Dot");       
            %Rob Turn Round it self - Linear, Angular
            turn35DegreL(robotPub);
            foundGreenDot = false;   
        end
    else
        %disp("Not Found Green Dot");       
        %Rob Turn Round it self - Linear, Angular
        turn35DegreL(robotPub);
        foundGreenDot = false;     
    end       
end


function [circleFound, centersBright] = findCircle(A)
    Rmin = 10;
    Rmax = 100;

    [centersBright, radiiBright] = imfindcircles(A,[Rmin Rmax],'ObjectPolarity','bright');
    [centersDark, radiiDark] = imfindcircles(A,[Rmin Rmax],'ObjectPolarity','dark');
    viscircles(centersBright, radiiBright,'Color','b');

        if centersBright > 0
           circleFound = true; 

           disp(centersBright);
           return;
        end

    circleFound = false; 
    disp("circleFound: " + circleFound);
end

function  wallPosition(a, robotPub)

    %disp("AngToWall: " + a);
    angThresholdMin = -0.1;  
    angThresholdMax = 0.1;
        
    if((a > angThresholdMax) || (a < angThresholdMin))
        if (a > angThresholdMax)
            %disp("L - AngToWall: " + a);
            %Rob Turn Round it self - Linear, Angular
            sendVelmsgRob(0, -0.3, robotPub);
            %disp("----");
        elseif (a < angThresholdMin)
            %disp("R - AngToWall: " + a);
            %Rob Turn Round it self - Linear, Angular
            sendVelmsgRob(0, 0.3, robotPub);
            %disp("----");
        else
            %Stop Rob
            sendVelmsgRob(0, 0, robotPub);
        end
        findWall(robotPub, true);  
    end  
end

function  wallPositionClose(dist, robotPub)

    disp("dist To Wall: " + dist);
    distThresholdMin = 0.5;  
           
    if((dist > distThresholdMin))
        if (dist > distThresholdMin)
            %disp("1 - Dist ToWall: " + dist);
            %Rob Turn Round it self - Linear, Angular
            sendVelmsgRob(0.2, 0, robotPub);
            %disp("----")
        else
            %Stop Rob
            sendVelmsgRob(0, 0, robotPub);
        end
        mode = false;
        findWall(robotPub, mode);       
    end     
end


function drivFountOnDot(robotPub, BW, centersBright)

    RGB = takePictureDebug();
    BW = createMask(RGB);
    [circleFound, centersBright] = findCircle(BW);
    
    disp("centersBright--- " + centersBright);
    figure(3);
    imshow(BW);
    
    %pause(0.5);
    %BW_new = createMask(RGB);
    %imshow(BW_new);  
    
%     stats = regionprops('table',BW,'Centroid');  
%     if(stats.Centroid)         
%         allCentroids = [stats.Centroid];
%         xCentroids = allCentroids(1:2:end); 
%         disp("xCentroids: " + xCentroids);
%     else
    if centersBright > 0
           %disp(centersBright);
           xCentroids = centersBright(1:2:end); 
            disp("X - Dot: " + xCentroids);
        else
        disp("stats.Centroid Not found: ");        
        turn90DegreR(robotPub);
        pause(0.5);
        driveForward(robotPub, "f");
        pause(0.5);
        
        findGreenDot(robotPub);      
        return;
    end
  
    angThresholdMin = 300;  
    angThresholdMax = 360;   
    
    if((xCentroids > angThresholdMax) || (xCentroids < angThresholdMin))    
        %pause(2);
        if (xCentroids < angThresholdMin)
            disp("x < angThresholdMin");
            turn90DegreL(robotPub);
            disp("turn90DegreL");
            pause(1);
            if xCentroids < 50
                destinFromDot = 3;
            elseif xCentroids > 200
                destinFromDot = 2;
            else
                destinFromDot = 1;
            end
            
            for i = 1:destinFromDot
                sendVelmsgRob(0.5, 0, robotPub);
                pause(1);
            end
            
            turn90DegreR(robotPub)
            findWall(robotPub, true);
        end

        if (xCentroids > angThresholdMax)
            disp("x > angThresholdMax");
            turn90DegreR(robotPub)
            disp("turn90DegreR");
            pause(1);
            if xCentroids > 550
                destinFromDot = 3;
            elseif xCentroids > 450
                destinFromDot = 2;
            elseif xCentroids < 450
                destinFromDot = 1;
            else
                destinFromDot = 1;
            end
            
            for i = 1:destinFromDot
                sendVelmsgRob(0.5, 0, robotPub);
                pause(1);
            end
            turn90DegreL(robotPub);
            findWall(robotPub, true);
        end              
    else
        takePicture(robotPub);
    end

end

%Turn the robot 90 degrees to the right
function turn90DegreR(robotPub)
    sendVelmsgRob(0, -2.6, robotPub);
end

%Turn the robot 90 degrees to the left
function turn90DegreL(robotPub)
    sendVelmsgRob(0, 2.6, robotPub);
end

%Turn the robot 35 degrees to the right
function turn35DegreR(robotPub)
    for i = 1:3
        sendVelmsgRob(0, -0.5, robotPub);
        pause(0.2);
    end 
end

%Turn the robot 35 degrees to the left
function turn35DegreL(robotPub)
    for i = 1:3
        sendVelmsgRob(0, 0.5, robotPub);
        pause(0.2);
    end 
end

function driveForward(robotPub, mode)

switch mode
    case "f"
        speed = 0.8;  
        disp("Driving Forward");
    case "b"
        speed = -1;       
    otherwise
        disp("Driving Backward");
end

     for i = 1:1 % Turn the robot 5 degrees to the left
        sendVelmsgRob(speed, 0, robotPub);
        pause(0.3);
     end
end


function [a, dist] =  findWall(robotPub, mode)
    if ismember('/scan',rostopic('list'))

        scansub = rossubscriber('/scan');  
        scan = receive(scansub); %Receive message    
        %plot(scan);
        % get data from scan
        cart = readCartesian(scan);
        
        %Checker om scan indeholder er data
        while(length(cart) < 200)
            sendVelmsgRob(0, 0.5, robotPub);
            scan = receive(scansub); %Receive message
            cart = readCartesian(scan);

            disp("Calibraing - Angular");
        end                
        %figure(2);
        %plot(cart(:,2), cart(:,1), '.') % note - y before x..
        %title('Cart');

        x = cart(:,2); % x-pos
        d = cart(:,1); % depth

        % Fitting one line..

        % only left side
        xleft = x(100:200);
        dleft = d(100:200);
        %plot(xleft, dleft, '.');

        mdl = fitlm(xleft,dleft);
        coef=mdl.Coefficients.Estimate;

        %plot(x,d, '.'), hold on
        %plot(x, coef(1) + coef(2)*x, 'r'); 
                 
        a = coef(2);
        b = coef(1);
          
        % Compute distance of the closest obstacle
        dist = abs(b)/(sqrt(a.^2+1));        
                     
        %disp("Hældnigns coef - a: " + a) 
        %disp("Skæring punkt - b: " + b)  
        %disp("Længeden til væg: " + dist)
        %disp("----")
        
        if mode == true
            wallPosition(a, robotPub);
        elseif mode == false
            wallPositionClose(dist, robotPub);
        end
                   
    end
end

function [BW, maskedRGBImage] = createMask(RGB)
%createMask  Threshold RGB image using auto-generated code from colorThresholder app.
%  [BW,MASKEDRGBIMAGE] = createMask(RGB) thresholds image RGB using
%  auto-generated code from the colorThresholder app. The colorspace and
%  range for each channel of the colorspace were set within the app. The
%  segmentation mask is returned in BW, and a composite of the mask and
%  original RGB images is returned in maskedRGBImage.

% Auto-generated by colorThresholder app on 12-May-2021
%------------------------------------------------------


% Convert RGB image to chosen color space
I = RGB;

% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.000;
channel1Max = 0.000;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.000;
channel2Max = 255.000; %255.000

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.000;
channel3Max = 0.000;

% Create mask based on chosen histogram thresholds
sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BW = sliderBW;

% Initialize output masked image based on input image.
maskedRGBImage = RGB;

% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BW,[1 1 3])) = 0;
end




