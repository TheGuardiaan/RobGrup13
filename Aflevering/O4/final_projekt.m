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

robotPub = rospublisher('/mobile_base/commands/velocity');
sendVelmsgRob(0,0, robotPub); %Reset Rob AngularVel to 0



%% ----Reset pose to [0 0] in gazebo - Need to get fix!!!
%resetRobotPose = rospublisher('/mobile_base/commands/reset_odometry');
%msg = rosmessage(resetRobotPose);
%send(resetRobotPose, msg)

%% -----------------
%Set new startpose - Virker ikke - Prøv med - "Localize TurtleBot Using Monte Carlo Localization"
            %robotInitialLocation = punkt_A;
            %initialOrientation = 0;

            %GazeboInitialLocation = robotInitialLocation;
            %GazeboCurrentPose = [robotInitialLocation initialOrientation];
% ---------------


%% 
%Calculate a simple path:
punkt_A = [0 0]; %Testpunkt
%punkt_B = [2 2]; 
punkt_C = [2 3]; 

%punkt_A = [46.5 29];
punkt_B = [4 10];
punkt_C = [27.5 2.5];

%%
nodes = 350;


findGreenDot(robotPub)
pause(10)


% -- Path one drive
path = findpathFunc(punkt_A, punkt_B, map, nodes, 1);
controller = setController(path);
drivePath(punkt_B, controller, robotPub); %(Goal,controller) 
findGreenDot(robotPub)

for i = 1:1 %Use just for testing
    disp("---- Path 1 ---")
    pause(2)
end

% -- Path two drive
path2 = findpathFunc(punkt_B, punkt_C, map, nodes, 2);
controller2 = setController(path2);
drivePath(punkt_C, controller2, robotPub);
findGreenDot(robotPub);

sendVelmsgRob(0,0, robotPub); %Stop Rob


for i = 1:1 %Use just for testing
    disp("----I'am Done MOM!!!!---")
    pause(2)
end



%%

%Drive To punkt func
function drivePath(GazeboGoal, controller, robotPub)

    %---------------
    % -- Get Rob pose [x y theta] and set GazeboInitialLocation
    odom = rossubscriber('/odom');
    odomdata = receive(odom,3); %modtag indenfor 3 sekunder -> ellers fejl
    pose = odomdata.Pose.Pose;

    x = pose.Position.X;
    y = pose.Position.Y;

    eulerOrientation = euler(quaternion([pose.Orientation.X pose.Orientation.Y pose.Orientation.Z pose.Orientation.W]), 'ZYX', 'frame');
    theta = eulerOrientation(3);

    GazeboInitialLocation = [x, y]
    GazeboCurrentPose = [x, y, theta]  

    goalRadius = 0.5;
    distanceToGoal = norm(GazeboInitialLocation - GazeboGoal');
    %---------------
    
    
  	scanWorld()
    
    
    

    %%% Initialize the simulation loop
    sampleTime = 0.1;
    vizRate = rateControl(1/sampleTime);
    while( distanceToGoal > goalRadius )

        % Compute the controller outputs, i.e., the inputs to the robot
        [v, omega] = controller(GazeboCurrentPose);    
           
        % Send message to robot i Gazebo
        sendVelmsgRob(v,omega, robotPub);
       
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
end

%Create a simple roadmap with 50 nodes
function path = findpathFunc(start_punkt, end_punkt, map, nodes, figNum)
    disp("Find path!!")
    %nodes = 1000;
    prmSimple = mobileRobotPRM(map,nodes);

    path = findpath(prmSimple,start_punkt,end_punkt);
    figure(figNum)
    show(prmSimple)
        
end

function scanWorld()
% Read scan continously
    disp("Run scanWorld Func")
    if ismember('/scan',rostopic('list'))
        scansub = rossubscriber('/scan');  
        linescan = receive(scansub); %Receive message
        ranges = linescan.Ranges; %Extract scan
        angles = linescan.AngleMin:linescan.AngleIncrement:linescan.AngleMax;

        angleInDegrees = rad2deg(angles);

        plot(angleInDegrees, ranges)
        xlabel('Angle [Degrees]')
        ylabel('Distance [m]')
        saveas(gcf,'linescan.png')

        disp("Ranges");
            disp(ranges);
    end
end

function controllerReturn = setController(path)
    disp("Run controllerReturn Func")
    %Simulering i Gazebo
    controller = controllerPurePursuit;
    controller.Waypoints = path;
    controller.DesiredLinearVelocity = 0.5;
    controller.MaxAngularVelocity = 1;
    controller.LookaheadDistance = 0.4;
    
    controllerReturn = controller;
end

function sendVelmsgRob(Linear, Angular, robotPub)
    %disp("Run sendVelmsgRob Func")
    % Send message to robot i Gazebo
    velmsg = rosmessage(robotPub);
    velmsg.Angular.Z = Angular;	% Angular velocity (rad/s)
    velmsg.Linear.X = Linear; % Linear velocity (m/s), +forward,-reverse
    send(robotPub,velmsg);   
end


function findGreenDot(robotPub)
    disp("Run findGreenDot Func")
    greenDotNotFound = true;
        
    while (greenDotNotFound)           
        foundGreenDot = takePicture();
        
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
                end            
            greenDotNotFound = false;
        end
    end
end


function foundGreenDot = takePicture()
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
    
    
    figure(3)
    imshow(img);

    pause(1) %Delay

     
      
    
      foundGreenDot = dotFound(img);  
end


function foundGreenDot = dotFound(RGB)      

    imshow(RGB);
    
    BW = createMask(RGB);
    
    imshow(BW);
    
    stats = regionprops('table',BW,'Centroid');
    
    if(stats.Centroid )
        disp("green dot found");
        foundGreenDot = true; 
        
        centers = stats.Centroid;    
        disp("center" + centers)
    else
        disp("dot not found");  
        foundGreenDot = false;     
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
channel2Max = 0.000; %255.000

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
