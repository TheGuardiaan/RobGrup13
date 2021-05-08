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



%% oprettet commandore til robotten - Sætter en vinklen og hastighed
velmsg.Angular.Z = 0.0;	% Angular velocity (rad/s)
velmsg.Linear.X = 0.1; % Linear velocity (m/s), +forward,-reverse

%%
send(robot,velmsg);
%% Rostopic list    
odom = rossubscriber('/odom'); %Scriber til topic for at for roboten's koordinater

%%


%% Test odom for at se det virker
    s = 1000;
for c = 1:s
odomdata = receive(odom,3);
pose = odomdata.Pose.Pose;
x = pose.Position.X;
y = pose.Position.Y;
z = pose.Position.Z; % Angular
  
     
disp("Position: x,y,z");
    disp(x);    
    disp(y);
    disp(z);
    



send(robot,velmsg);
    
    
end