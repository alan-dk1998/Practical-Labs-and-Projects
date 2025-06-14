rgb = imread('Image2.jpg');
rgb1 = rgb+50;
rgb_imadjust = imadjust(rgb1,[0.1 0.7],[]);
hsv = rgb2hsv(rgb_imadjust); 
%h = round(hsv(:,:,1)*360);
h = hsv(:,:,1);
s = hsv(:,:,2);
v = hsv(:,:,3);

red = h > 0.95 ;
%imshow (red);
red_stats = regionprops(red,'Area','Centroid','Orientation')
red_stats([red_stats.Area]<1000)=[];
First_red = [red_stats(1).Centroid(1) red_stats(1).Centroid(2) red_stats(1).Orientation]

green = h > 0.35 & h < 0.55   ;
%imshow (green);
green_stats = regionprops(green,'Area','Centroid','Orientation')
green_stats([green_stats.Area]<1000)=[];
First_green = [green_stats(1).Centroid(1) green_stats(1).Centroid(2) green_stats(1).Orientation]

blue = h > 0.6 & h < 0.65 & v > 0.7  ;
imshow (blue);
blue_stats = regionprops(blue,'Area','Centroid','Orientation')
blue_stats([blue_stats.Area]<1000)=[];
First_blue = [blue_stats(1).Centroid(1) blue_stats(1).Centroid(2) blue_stats(1).Orientation]

yellow = h > 0.1 & h < 0.2  ;
%imshow (yellow);
yellow_stats = regionprops(yellow,'Area','Centroid','Orientation')
yellow_stats([yellow_stats.Area]<1000)=[];
First_yellow = [yellow_stats(1).Centroid(1) yellow_stats(1).Centroid(2) yellow_stats(1).Orientation]

black = h > 0.5 & v > 0.15 & v < 0.55 ;
%imshow (black);
black_stats = regionprops(black,'Area','Centroid','Orientation')
black_stats([black_stats.Area]<1000)=[];
First_black = [black_stats(1).Centroid(1) black_stats(1).Centroid(2) black_stats(1).Orientation]

