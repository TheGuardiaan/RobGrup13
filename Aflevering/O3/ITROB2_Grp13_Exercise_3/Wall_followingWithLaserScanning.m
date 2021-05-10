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
% Read scan continously
a_flag = true;
if ismember('/scan',rostopic('list'))
    scansub = rossubscriber('/scan');  
    while(a_flag)
        linescan = receive(scansub); %Receive message
        ranges = linescan.Ranges; %Extract scan
        angles = linescan.AngleMin:linescan.AngleIncrement:linescan.AngleMax;
        
        angleInDegrees = rad2deg(angles);
                
        plot(angleInDegrees, ranges)
        xlabel('Angle [Degrees]')
        ylabel('Distance [m]')
        %saveas(gcf,'linescan.png')
                
        disp("Ranges");
            disp(ranges);
                        
       a_flag = false;
    end
end

%% "Laser scan" line fit example
clear
clc

a_flag = 1;
a_flag_stop = 200;

spinVelocityRight = -0.1; 
spinVelocityLeft = 0.1; % Angular velocity (rad/s)

forwardVelocity = 0.1;    % Linear velocity (m/s)
forwardVelocityMaxSpeed = 0.4;    % Linear velocity (m/s)
SpeedUp = forwardVelocity;

backwardVelocity = -0.1; % Linear velocity (reverse) (m/s)

%distanceThreshold = 1; % Distance threshold (m) for turning
distanceThresholdMin = 0.5;  
distanceThresholdMax = 0.7;

if ismember('/scan',rostopic('list')) %Check if conecctet to robot
    
    scansub = rossubscriber('/scan');   
    scan = receive(scansub); %Receive message     
    cart = readCartesian(scan);
     
    robot = rospublisher('/mobile_base/commands/velocity');
    velmsg = rosmessage(robot);
    
    
    %Checker om scan er indeholder data
    while(length(cart) < 200)
    
        velmsg.Angular.Z = spinVelocityRight;   % Angular velocity (rad/s)
        velmsg.Linear.X = 0;    % Linear velocity (m/s), +forward,-reverse
        send(robot,velmsg);

        scan = receive(scansub); %Receive message
        cart = readCartesian(scan);

        disp("Searching for more data-points")
    end
   
    
    while(a_flag < a_flag_stop)
        scan = receive(scansub); %Receive message             
        %plot(scan);
        % get data from scan
        cart = readCartesian(scan);
        
        %Checker om scan indeholder er data
        while(length(cart) < 200)

            velmsg.Angular.Z = spinVelocityRight;   % Angular velocity (rad/s)
            velmsg.Linear.X = 0;    % Linear velocity (m/s), +forward,-reverse

            send(robot,velmsg);

            scan = receive(scansub); %Receive message
            cart = readCartesian(scan);

            disp("Calibraing - Angular")
        end 
                     
        figure(2)
        plot(cart(:,2), cart(:,1), '.') % note - y before x..
        title('Cart')

        x = cart(:,2); % x-pos
        d = cart(:,1); % depth

        % Fitting one line..

        % only left side
        xleft = x(100:200);
        dleft = d(100:200);
        plot(xleft, dleft, '.')

        mdl = fitlm(xleft,dleft);
        coef=mdl.Coefficients.Estimate;

        plot(x,d, '.'), hold on
        plot(x, coef(1) + coef(2)*x, 'r') 
                 
        a = coef(2);
        b = coef(1);
          
        % Compute distance of the closest obstacle
        dist = abs(b)/(sqrt(a.^2+1));        
                     
        disp("Hældnigns coef - a: " + a) 
        disp("Skæring punkt - b: " + b)  
        disp("Længeden til væg: " + dist)
        disp("----")
                
        % Command robot action
        if ((dist > distanceThresholdMin) && (dist < distanceThresholdMax)) 
            % Continue on forward path
            
            
            velmsg.Linear.X = SpeedUp;
            velmsg.Angular.Z = 0;                  
            disp("Drive forward")
            
            SpeedUp = SpeedUp + 0.01;
            disp("Speed forward" + SpeedUp);
            
            
           
        elseif(dist < distanceThresholdMin) % Destance to Small
            disp("Drive Left - Destance to Small: If")
                        
            velmsg.Linear.X = forwardVelocity;
            velmsg.Angular.Z = spinVelocityLeft;
            disp("Længeden til væg: " + dist)
              
            count = 0;
            countMax = 1000;
            
            while count < countMax
                velmsg.Linear.X = forwardVelocity;
                velmsg.Angular.Z = spinVelocityLeft;
                send(robot,velmsg);
                
                disp("Count: " + count)
                count = count + 1;
            end
            
        elseif(dist > distanceThresholdMax)  % Destance to big
            disp("Drive Right - Destance to Big: If")
            
            velmsg.Angular.Z = spinVelocityRight;
            velmsg.Linear.X = forwardVelocity;
           
            disp("Længeden til væg: " + dist)
                                      
        end   
               
        send(robot,velmsg);               
       a_flag = a_flag + 1;
    end
    
end



