%% Gray scale object detection example
clear

m1 = imread('IMG1.JPG');
imshow(m1)

%%
m1g = rgb2gray(m1);
%imshow(m1)
imagesc(m1g), colorbar

%%
m1b = m1g < 130; % Image Segmenter APP
imagesc(m1b)

%%
m1bd = imclose(m1b, strel('disk', 20)); % fill holes in disc
imagesc(m1bd)

%%
m1be = bwpropfilt(m1bd, 'Area', [1000 200000]); % remove small blobs..
imagesc(m1be)

%% visualize / analyze..
m1lab = bwlabel(m1be);
m1prop = regionprops(m1lab, 'Area', 'Eccentricity') % find features..

%% remove high Eccentricity..
m1bf = bwpropfilt(m1be, 'Eccentricity', 1, 'smallest');
imagesc(m1bf)

%% Make a decision..
m1prop = regionprops(m1bf, 'Area', 'Eccentricity'); % find features..
if m1prop.Eccentricity < 0.6, 
    disp('GOAL!'), 
end

