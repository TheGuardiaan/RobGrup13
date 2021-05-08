%Using the Path Following Controller Along with PRM
%If the desired set of waypoints are computed by a path planner, the path following controller can be used in the same fashion. First, visualize the map

load exampleMaps
map = binaryOccupancyMap(simpleMap);
figure
show(map)

%%
%You can compute the path using the PRM path planning algorithm. See  Path Planning in Environments of Different Complexity for details.
%mapInflated = copy(map);
%inflate(mapInflated, robot.TrackWidth/2);
prm = robotics.PRM(mapInflated);
prm.NumNodes = 100;
prm.ConnectionDistance = 10;


%%
%Find a path between the start and end location. Note that the path will be different due to the probabilistic nature of the PRM algorithm.
startLocation = [4.0 2.0];
endLocation = [24.0 20.0];
path = findpath(prm, startLocation, endLocation)

%Display the inflated map, the road maps, and the final path.
show(prm);

%You defined a path following controller above which you can re-use for computing the control commands of a robot on this map. To re-use the controller and redefine the waypoints while keeping the other information the same, use the release function.
%release(controller);
%controller.Waypoints = path;

%Set initial location and the goal of the robot as defined by the path
robotInitialLocation = path(1,:);
robotGoal = path(end,:);

%Assume an initial robot orientation
initialOrientation = 0;

%Define the current pose for robot motion [x y theta]
robotCurrentPose = [robotInitialLocation initialOrientation]';

%Compute distance to the goal location
distanceToGoal = norm(robotInitialLocation - robotGoal);

%Define a goal radius
goalRadius = 0.1;

%%
%Drive the robot using the controller output on the given map until it reaches the goal. The controller runs at 10 Hz.
reset(vizRate);

% Initialize the figure
figure

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
    show(map);
    hold all

    % Plot path each instance so that it stays persistent while robot mesh
    % moves
    plot(path(:,1), path(:,2),"k--d")
    
    % Plot the path of the robot as a set of transforms
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, 'MeshFilePath', 'groundvehicle.stl', 'Parent', gca, "View","2D", "FrameSize", frameSize);
    light;
    xlim([0 27])
    ylim([0 26])
    
    waitfor(vizRate);
end