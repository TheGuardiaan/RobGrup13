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

% Read scan continously
if ismember('/scan',rostopic('list'))
    scansub = rossubscriber('/scan');
    
    
    
    linescan = receive(scansub); %Receive message
    ranges = linescan.Ranges; % Extract scan
    angles = linescan.AngleMin:linescan.AngleIncrement:linescan.AngleMax;
    plot(angles, ranges)
    xlabel('Angle [rad]')
    ylabel('Distance [m]')
    saveas(gcf,'linescan.png')
        
    
end

%%
detectNode = ros2node("/detection");
pause(2)
laserSub = ros2subscriber(detectNode,"/scan");
pause(2)