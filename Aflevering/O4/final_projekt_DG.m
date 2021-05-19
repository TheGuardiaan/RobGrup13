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
image = imread('ShannonX.jpg');
grayimage = rgb2gray(image);
bwimage = grayimage < 0.5;

map = binaryOccupancyMap(bwimage,20.8);
%Sæt vædien ned = Map mindre
%Sæt vædien op = Map større
%Reslusion til 21 for at scalere 2D map til 3D map som cirka er 54m
show(map)

robotPub = rospublisher('/mobile_base/commands/velocity');
%sendVelmsgRob(0,0, robotPub); %Reset Rob AngularVel to 0
odom = rossubscriber('/odom');

%% ----Reset pose to [0 0] in gazebo - Need to get fix!!!
resetRobotPose = rospublisher('/mobile_base/commands/reset_odometry');
msg = rosmessage(resetRobotPose);
send(resetRobotPose, msg)

%% -----------------
%Set new startpose - Virker ikke - Prøv med - "Localize TurtleBot Using Monte Carlo Localization"
            %robotInitialLocation = punkt_A;
            %initialOrientation = 0;

            %GazeboInitialLocation = robotInitialLocation;
            %GazeboCurrentPose = [robotInitialLocation initialOrientation];
% ---------------


%% Nulstil rob place
%nodes = 550;

% turn90DegreR(robotPub);
% turn90DegreR(robotPub);

% punkt_Zero = [1.5 1.5];
% punkt_A = [3 17];
% 
% path = findpathFunc(punkt_Zero, punkt_A, map, nodes, 1);
% controller = setController(path);
% drivePath(punkt_A, controller, robotPub, map, nodes, odom); %(Goal,controller) 


%%
%Calculate a simple path:
punkt_A = [1.5 1.5]; %Testpunkt
punkt_BC = [2.5 6.7]; 
%punkt_C = [2 3]; 


%%
%Calculate a simple path:
%punkt_A = [46.7 28.3];
punkt_B = [3.5 10.3];
punkt_C = [26 2.2];


%%
% test avoid obstacles
nodes = 350;

% -- Path one drive
path = findpathFunc(punkt_A,punkt_B, map, nodes, 1);
controller = setController(path);
drivePath(punkt_B, controller, robotPub, map, nodes, odom); %(Goal,controller) 
findGreenDot(robotPub);
% sendVelmsgRob(1,0, robotPub);
% while 1 
%     avoidObstacles(robotPub);
% end

disp("---- Path 1 ---")
pause(2)

%%
nodes = 350;

% -- Path one drive
path = findpathFunc(punkt_A, punkt_B, map, nodes, 1);
controller = setController(path);
drivePath(punkt_B, controller, robotPub, map, nodes, odom); %(Goal,controller) 
findGreenDot(robotPub);

disp("---- Path 1 ---")
pause(2)

% -- Path one drive
path = findpathFunc(punkt_A, punkt_B, map, nodes, 1);
controller = setController(path);
drivePath(punkt_B, controller, robotPub, map, nodes, odom); %(Goal,controller) 
findGreenDot(robotPub);

disp("---- Path 1 ---")
pause(2)

% -- Path BC drive
path3 = findpathFunc(punkt_B, punkt_BC, map, nodes, 1);
controller3 = setController(path3);
drivePath(punkt_BC, controller3, robotPub, map, nodes, odom);


% -- Path two drive
path2 = findpathFunc(punkt_BC, punkt_C, map, nodes, 1);
controller2 = setController(path2);
drivePath(punkt_C, controller2, robotPub, map, nodes, odom);
findGreenDot(robotPub);

sendVelmsgRob(0,0, robotPub); %Stop Rob

disp("----I'am Done MOM!!!!---")
pause(2)



%%
%Drive To punkt func
function drivePath(GazeboGoal, controller, robotPub, map, nodes, odom)

    %---------------
    % -- Get Rob pose [x y theta] and set GazeboInitialLocation
    %odom = rossubscriber('/odom');
    odomdata = receive(odom,3); %modtag indenfor 3 sekunder -> ellers fejl
    pose = odomdata.Pose.Pose;

    x = pose.Position.X;
    y = pose.Position.Y;

    eulerOrientation = euler(quaternion([pose.Orientation.X pose.Orientation.Y pose.Orientation.Z pose.Orientation.W]), 'ZYX', 'frame');
    theta = eulerOrientation(3);

    GazeboInitialLocation = [x, y];
    GazeboCurrentPose = [x, y, theta];  

    goalRadius = 0.5;
    distanceToGoal = norm(GazeboInitialLocation - GazeboGoal');
    %---------------
    
    %%% Initialize the simulation loop
    sampleTime = 0.1;
    vizRate = rateControl(1/sampleTime);
    while( distanceToGoal > goalRadius )

		%Obstacle Detection
        avoidObstacles(robotPub);  
        disp("Back in DrivePath while loop")
            
        % Compute the controller outputs, i.e., the inputs to the robot
        [v, omega] = controller(GazeboCurrentPose);    
           
        % Send message to robot i Gazebo
        sendVelmsgRob(v,omega, robotPub);
       
        % Get the robot's velocity using controller inputs
        %vel = derivative(robot, robotCurrentPose, [v omega]);

        % Update the current pose
        %odom = rossubscriber('/odom');
        odomdata = receive(odom,3); %modtag indenfor 3 sekunder -> ellers fejl
        pose = odomdata.Pose.Pose;
        x = pose.Position.X;
        y = pose.Position.Y;       

        eulerOrientation = euler(quaternion([pose.Orientation.X pose.Orientation.Y pose.Orientation.Z pose.Orientation.W]), 'ZYX', 'frame');
        theta = eulerOrientation(3);

        GazeboCurrentPose = [x, y, theta];         
        %disp("x - y - theta")

        % Update the plot
        hold on

        % Plot the path of the robot as a set of transforms
        plot(x, y, 'g--X');

        xlim([0 62]);
        ylim([0 30]);

        % Re-compute the distance to the goal
        distanceToGoal = norm(GazeboCurrentPose(1:2) - GazeboGoal(:)');      

        disp("DistanceToGoal: " + distanceToGoal);
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
                end            
            greenDotNotFound = false;
        end
    end
end


function foundGreenDot = takePicture(robotPub)
    takePictureDebug();
    pause(1);
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
        
    figure(4);
    imshow(img);
    pause(1); %Delay

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

    imshow(RGB);
    %pause(2);
    BW = createMask(RGB);

    imshow(BW);  
    
    stats = regionprops('table',BW,'Centroid');
    
    if(stats.Centroid )
        disp("!! - Found Green DOT - !!");             
        pause(1);       
        foundGreenDot = true;        
        [a, dist] = findWall(robotPub, true); 
        %----
        % Find green Dot and drive
        %----
        drivFountOnDot(robotPub); 
                     
        %pause(20);
        %Rob køre 40 cm fra væg
        wallPositionClose(dist, robotPub);
                
        for i = 1:3 %Turn rob 180 degres
            turn90DegreL(robotPub);
        end
        
        disp("!!----STOP---!!!" );                            
        pause(2); 
        
    else
        %disp("Not Found Green Dot");       
        %Rob Turn Round it self - Linear, Angular
        turn15Degre(robotPub);
        %sendVelmsgRob(0, 0.5, robotPub);
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

function  wallPosition(a, robotPub)

    %disp("AngToWall: " + a);
    angThresholdMin = -0.1;  
    angThresholdMax = 0.1;
        
    if((a > angThresholdMax) || (a < angThresholdMin))
        if (a > angThresholdMax)
            disp("L - AngToWall: " + a);
            %Rob Turn Round it self - Linear, Angular
            sendVelmsgRob(0, -0.3, robotPub);
            disp("----");
        elseif (a < angThresholdMin)
            disp("R - AngToWall: " + a);
            %Rob Turn Round it self - Linear, Angular
            sendVelmsgRob(0, 0.3, robotPub);
            disp("----");
        else
            %Stop Rob
            sendVelmsgRob(0, 0, robotPub);
        end
        
        findWall(robotPub, true);  
    end  
end

function  wallPositionClose(dist, robotPub)

    disp("dist To Wall: " + dist);
    distThresholdMin = 0.4;  
           
    if((dist > distThresholdMin))
        if (dist > distThresholdMin)
            %disp("1 - Dist ToWall: " + dist);
            %Rob Turn Round it self - Linear, Angular
            sendVelmsgRob(0.2, 0, robotPub);
            disp("----")
        else
            %Stop Rob
            sendVelmsgRob(0, 0, robotPub);
        end
        mode = false;
        findWall(robotPub, mode);       
    end     
end


function drivFountOnDot(robotPub)

    pause(1);
    RGB = takePictureDebug();
    figure(4);
    imshow(RGB);
    
    pause(1);
    BW_new = createMask(RGB);

    %imshow(BW_new);  
    
    stats = regionprops('table',BW_new,'Centroid');
    
    if(stats.Centroid)         
        allCentroids = [stats.Centroid];
        xCentroids = allCentroids(1:2:end); 
        disp("xCentroids: " + xCentroids);
    else
        disp("stats.Centroid Not found: ");       
        findGreenDot(robotPub);       
        %sendVelmsgRob(0.5, 0, robotPub);
        %pause(0.5);
        return;
    end
  
    angThresholdMin = 300;  
    angThresholdMax = 360;
               
    if((xCentroids > angThresholdMax) || (xCentroids < angThresholdMin))    
        %pause(2);
        if (xCentroids < angThresholdMin)
            disp("x < angThresholdMin");
            turn90DegreR(robotPub)

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
            
            turn90DegreL(robotPub)
            findWall(robotPub, true);
        end

        if (xCentroids > angThresholdMax)
            disp("x > angThresholdMax");
            turn90DegreL(robotPub)

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
            turn90DegreR(robotPub)
            findWall(robotPub, true);
        end              
    else
        takePicture(robotPub);
    end

end


function turn90DegreL(robotPub)
    for i = 1:5 % Turn the robot 90 degrees to the left
        sendVelmsgRob(0, -1, robotPub);
        pause(0.3);
    end 
end

function turn90DegreR(robotPub)
    for i = 1:5 % Turn the robot 90 degrees to the Right
        sendVelmsgRob(0, 1, robotPub);
        pause(0.3);
    end 
end

function turn15Degre(robotPub)
    for i = 1:5 % Turn the robot 15 degrees to the left
        sendVelmsgRob(0, -0.5, robotPub);
        pause(0.2);
    end 
end

function driveForward(robotPub)
    sendVelmsgRob(0.5, 0, robotPub);
    pause(0.2);
end


function avoidObstacles(robotPub)

   disp("In obstacleDetected function")
   [dist, enoughDataForCart] = scanWorld(robotPub); 
   if(enoughDataForCart == false)
       disp("enoughDataForCart = " + enoughDataForCart)
       return;
   end

   disp("Dist: " + dist)
   %checks if too close to an obstacle

   minDist = 1;
   distanceThresholdMin = 0.7;
   distanceThresholdMax = 1;

   if (dist < minDist)
       disp("dist < 1 meter")
       sendVelmsgRob(0, 0, robotPub); %Stop robot
       [dist] = scanWorld(robotPub); %Scan again to check if moving obstacle
       
       findWall(robotPub, true);
       
       turn90DegreR(robotPub);
       while(dist < minDist)
            disp(" in while loop, dist < 1 meter")

            if ((dist > distanceThresholdMin) && (dist < distanceThresholdMax))
                sendVelmsgRob(0.5, 0, robotPub);
                disp("Distance til væg: " + dist)

            elseif(dist < distanceThresholdMin) % Distance to Small
                sendVelmsgRob(0.2, 0.1, robotPub); 
                disp("Distance til væg: " + dist)
            end
            [dist, enoughDataForCart] = scanWorld(robotPub); 

            if(enoughDataForCart == false)
                disp("enoughDataForCart = false" + enoughDataForCart)
                driveForward(robotPub)
                turn90DegreL(robotPub);
                return;
            end

       end
   end
end

function [dist, enoughDataForCart] = scanWorld(robotPub)
   if ismember('/scan',rostopic('list'))
        scansub = rossubscriber('/scan');  

        scan = receive(scansub); %Receive message             
                    
        cart = readCartesian(scan); %extract cartesian coordinates from scan
        
        if (length(cart) >= 200)  % Scan receive enough data to calculate distance (den ser noget foran sig)
            %scan = receive(scansub); %Receive message
            %cart = readCartesian(scan);
            x = cart(:,2); % x-pos
            d = cart(:,1); % depth

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
            enoughDataForCart = true;
        else
            dist = 10;
            enoughDataForCart = false;
        end
          
   end
end
