%%
close all
clear all
clc

camList = webcamlist;
%%

cam = webcam;

while true
    
im = cam.snapshot;
    
im = rgb2gray(im);
BW = edge(im,'canny');

figure(1), imshow(BW)


[H,T,R] = hough(BW,'RhoResolution',0.5,'Theta',-90:0.5:89);
P  = houghpeaks(H,50,'threshold',ceil(0.5*max(H(:))));
lines = houghlines(BW,T,R,P,'FillGap',20,'MinLength',20);


figure(2), imshow(im), hold on
max_len = 0;
for k = 1:length(lines)
    if abs(abs(lines(k).theta)-90) > 5 % my ugly fix to remove unwanted lines
        xy = [lines(k).point1; lines(k).point2];
        plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
        
        
        % Plot beginnings and ends of lines
        plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
        plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');
        
        % Determine the endpoints of the longest line segment
        len = norm(lines(k).point1 - lines(k).point2);
        if ( len > max_len)
            max_len = len;
            xy_long = xy;
        end
    end
end
hold off
end