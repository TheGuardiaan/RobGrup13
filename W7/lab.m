clear all
clc


image = imread("E:\Software_IKT\IHA\ITROB2_Robotter\W7\IMG_2323.jpg");


imageSuits  = imread("Suits.png");
%imshow(imageSuits);

gray_image = rgb2gray(imageSuits);
%imshow(gray_image)

bw = gray_image < 100;
%imshow(bw)


CC = bwconncomp(bw);
S = regionprops(CC,'Centroid');

stats = regionprops('table',bw,'Centroid',...
    'MajorAxisLength','MinorAxisLength')

centers = stats.Centroid;
diameters = mean([stats.MajorAxisLength stats.MinorAxisLength],2);
radii = diameters/2;

hold on
viscircles(centers,radii);
hold off