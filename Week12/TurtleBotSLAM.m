%%
%stop(wanderHelper);
rosshutdown
clear
clc

VmIp = 'http://192.168.174.129:11311';
MyIp = '192.168.0.28';


setenv('ROS_MASTER_URI',VmIp);
% assuming your own ip is 192.168.1.100
setenv('ROS_IP',MyIp);
rosinit(VmIp,'NodeHost',MyIp);


% Create a lidarSLAM object and set the map resolution and the max lidar range. This example uses a Jackal™ robot from Clearpath Robotics™. The robot is equipped with a SICK™ TiM-511 laser scanner with a max range of 10 meters. Set the max lidar range slightly smaller than the max scan range (8m), as the laser readings are less accurate near max range. Set the grid map resolution to 20 cells per meter, which gives a 5cm precision.
maxLidarRange = 8;
mapResolution = 20;
slamAlg = lidarSLAM(mapResolution, maxLidarRange);

%The following loop closure parameters are set empirically. Using higher loop closure threshold helps reject false positives in loop closure identification process. However, keep in mind that a high-score match may still be a bad match. For example, scans collected in an environment that has similar or repeated features are more likely to produce false positives. Using a higher loop closure search radius allows the algorithm to search a wider range of the map around current pose estimate for loop closures.
slamAlg.LoopClosureThreshold = 210;
slamAlg.LoopClosureSearchRadius = 8;
 
wheelsub = rossubscriber('/joint_states');
lastWheelState = [0 0];
distBetweenScans = 10;

%%
scancount=0;
totalscans=100;

while(scancount<totalscans)
    wheelstate = receive(wheelsub);
        
    if sum(abs(wheelstate.Position - lastWheelState))>distBetweenScans
        scancount = scancount + 1;
        lastWheelState = wheelstate.Position;
        
        if ismember('/scan',rostopic('list'))
            % Read scan
            scansub = rossubscriber('/scan');
            disp('Subscribed to topic')
            
            linescan = receive(scansub); %Receive message
            disp('Read scan topic')
            ranges = linescan.Ranges; % Extract scan
            angles = linescan.AngleMin:linescan.AngleIncrement:linescan.AngleMax;

            scan = lidarScan(double(ranges),double(angles));

            % Continue to add scans in a loop. Loop closures should be automatically detected as the robot moves. Pose graph optimization is performed whenever a loop closure is identified. The output optimizationInfo has a field, IsPerformed, that indicates when pose graph optimization occurs..
            %Plot the scans and poses whenever a loop closure is identified and verify the results visually. This plot shows overlaid scans and an optimized pose graph for the first loop closure. A loop closure edge is added as a red link.
            
            firstTimeLCDetected = false;            
            figure(1)
            
            
            [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scan);
            if ~isScanAccepted
                continue;
            end
            % visualize the first detected loop closure, if you want to see the
            % complete map building process, remove the if condition below
            if optimizationInfo.IsPerformed
                disp('Loop closure!!!')
            else
                disp('No loop closure this round :-(')
            end
            
            show(slamAlg, 'Poses', 'off');
            hold on;
            show(slamAlg.PoseGraph);
            hold off;
            
            drawnow   
            

        end
        
    end
    
    %stillScanning = stillScanning + 1;
end


% Plot the final built map after all scans are added to the slamAlg object. The previous for loop should have added all the scans despite only plotting the initial loop closure.
figure
show(slamAlg);
title({'Final Built Map of the Environment', 'Trajectory of the Robot'});

% Build Occupancy Grid Map
% The optimized scans and poses can be used to generate a occupancyMap, which represents the environment as a probabilistic occupancy grid.

[scans, optimizedPoses]  = scansAndPoses(slamAlg);
map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);

% Visualize the occupancy grid map populated with the laser scans and the optimized pose graph.
figure;
show(map);
hold on
show(slamAlg.PoseGraph, 'IDs', 'off');
hold off
title('Occupancy Grid Map Built Using Lidar SLAM');


% Draw occupancy grid
figure
imshow(map.checkOccupancy);