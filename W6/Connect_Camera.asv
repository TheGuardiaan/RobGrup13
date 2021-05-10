%%
rosshutdown

%%  
clear
clc
VmIp = 'http://192.168.174.129:11311';
MyIp = '192.168.0.28';


setenv('ROS_MASTER_URI',VmIp);
% assuming your own ip is 192.168.1.100
setenv('ROS_IP',MyIp);
rosinit(VmIp,'NodeHost',MyIp);



%%
clear
clc
%Date from google x-box kenct
%https://www.ncbi.nlm.nih.gov/pmc/articles/PMC3304120/
%Focal length	f = 4.884 ± 0.006 [mm]

%                 h = 4.5 [mm]
% Pixel size      px = 9.3 [μm]
%                 py = 9.3 [μm]

%f = 4884;
%S_x = 4.5;

%k1 = f/S_x;
%Z1 = (k1 * deltaX)/deltaR;

%deltaR found from:
%https://www.dfki.de/fileadmin/user_upload/import/8767_wasenmuller2016comparison.pdf

deltaR = 640; %height in pixels
deltaX = 1000; % Real height of object
Z = 2000; %Distance to object

k = deltaR / (deltaX/Z);
k = 1280; %Cam value
Z1 = (k * deltaX)/deltaR;


% Initialize the simulation loop
sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);
%

num_punk = 0.25;
maxZ = 5;
Z = 2;


xlabel("deltaR: Height in pixels");
ylabel("Z: Distance to object");

while (Z < (maxZ+num_punk))
    
    deltaR = k * deltaX/Z;
    % Update the plot
    hold on
    
    plot(deltaR, Z, 'X');
    disp("Display: Z");
        disp(Z);
        
    Z = num_punk + Z;
    
    
    waitfor(vizRate);
end
%%
%convert RGB to HSV
%hsv = rgb2hsv(Img)
%%
%rostopic info /camera/rgb/image_raw

% Grap image from camera
if ismember('/camera/rgb/image_color/compressed',rostopic('list'))
    imsub = rossubscriber('/camera/rgb/image_color/compressed');
end

if ismember('/camera/rgb/image_raw',rostopic('list'))
    imsub = rossubscriber('/camera/rgb/image_raw');
end

imgraw = receive(imsub); % a serialised image
img = readImage(imgraw); % decode image
figure
imshow(img);

imtool(img);