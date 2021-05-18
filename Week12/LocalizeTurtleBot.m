%%
%stop(wanderHelper);
rosshutdown
clc
 
clear
clc
VmIp = 'http://192.168.174.129:11311';
MyIp = '192.168.0.28';


setenv('ROS_MASTER_URI',VmIp);
% assuming your own ip is 192.168.1.100
setenv('ROS_IP',MyIp);
rosinit(VmIp,'NodeHost',MyIp);


%%

%load('offlineSlamData.mat');


%%
%image = imread('Shannon00.jpg');
image = imread('Shannon.jpg');
grayimage = rgb2gray(image);
bwimage = grayimage < 0.5;

map = binaryOccupancyMap(bwimage,20.8);
%Sæt vædien ned = Map mindre
%Sæt vædien op = Map større
%Reslusion til 21 for at scalere 2D map til 3D map som cirka er 54m
%show(map)

%%
maxLidarRange = 8;
mapResolution = 20;
slamAlg = lidarSLAM(mapResolution, maxLidarRange);

%%
slamAlg.LoopClosureThreshold = 210;  
slamAlg.LoopClosureSearchRadius = 8;

%%
for i=1:10
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scans{i});
    if isScanAccepted
        fprintf('Added scan %d \n', i);
    end
end

%%
figure;
show(slamAlg);
title({'Map of the Environment','Pose Graph for Initial 10 Scans'});

%%
firstTimeLCDetected = false;

figure;
for i=10:length(scans)
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scans{i});
    if ~isScanAccepted
        continue;
    end
    % visualize the first detected loop closure, if you want to see the
    % complete map building process, remove the if condition below
    if optimizationInfo.IsPerformed && ~firstTimeLCDetected
        show(slamAlg, 'Poses', 'off');
        hold on;
        show(slamAlg.PoseGraph); 
        hold off;
        firstTimeLCDetected = true;
        drawnow
    end
end
title('First loop closure');

%%
figure
show(slamAlg);
title({'Final Built Map of the Environment', 'Trajectory of the Robot'});
%%
[scans, optimizedPoses]  = scansAndPoses(slamAlg);
map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);

%%

wheelsub = rossubscriber('/joint_states');
lastWheelState = [0 0];
distBetweenScans = 10;

stillScanning = 0;

while(stillScanning == 1)
    wheelstate = receive(wheelsub);
    if sum(abs(wheelstate.Position - lastWheelState))>distBetweenScans
        lastWheelState = wheelstate.Position;
        % Read scan
        scansub = rossubscriber('/scan');
        
    end
end