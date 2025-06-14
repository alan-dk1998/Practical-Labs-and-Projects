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
%imshow (blue);
blue_stats = regionprops(blue,'Area','Centroid','Orientation')
blue_stats([blue_stats.Area]<1000)=[];
First_blue = [blue_stats(1).Centroid(1) blue_stats(1).Centroid(2) blue_stats(1).Orientation]

yellow = h > 0.1 & h < 0.2  ;
%imshow (yellow);
yellow_stats = regionprops(yellow,'Area','Centroid','Orientation')
yellow_stats([yellow_stats.Area]<1000)=[];
First_yellow = [yellow_stats(1).Centroid(1) yellow_stats(1).Centroid(2) yellow_stats(1).Orientation]

%Obtain the x and y centroid coordinates for each of the coloured bars
x_centroid_red = red_stats(1).Centroid(2)
y_centroid_red = red_stats(1).Centroid(1)
x_centroid_green = green_stats(1).Centroid(2)
y_centroid_green = green_stats(1).Centroid(1)
x_centroid_blue = blue_stats(1).Centroid(2)
y_centroid_blue = blue_stats(1).Centroid(1)
x_centroid_yellow = yellow_stats(1).Centroid(2)
y_centroid_yellow = yellow_stats(1).Centroid(1)

%Display all the locations of the 4 different coloured bar
fprintf('x-coordinates for red bar: %f \n',x_centroid_red)
fprintf('y-coordinates for red bar: %f \n',y_centroid_red)
fprintf('x-coordinates for green bar: %f \n',x_centroid_green)
fprintf('y-coordinates for green bar: %f \n',y_centroid_green)
fprintf('x-coordinates for blue bar: %f \n',x_centroid_blue)
fprintf('y-coordinates for blue bar: %f \n',y_centroid_blue)
fprintf('x-coordinates for yellow bar: %f \n',x_centroid_yellow)
fprintf('y-coordinates for yellow bar: %f \n',y_centroid_yellow)

L=0.05;                 % cube size (length of an edge)
%--------------------------------------------------------------------------
%Drawing Red bars
xc_red = (x_centroid_red/1000)+0.2; 
yc_red = (y_centroid_red/1000)-0.32; 
zc_red = -0.2;    % coordinated of the center
alpha=0.8;           % transparency (max=1=opaque)

X_red = [0 0 0 0 0 1; 1 0 1 1 1 1; 1 0 1 1 1 1; 0 0 0 0 0 1];
Y_red = [0 0 0 0 1 0; 0 1 0 0 1 1; 0 1 1 1 1 1; 0 0 1 1 1 0];
Z_red = [0 0 1 0 0 0; 0 0 1 0 0 0; 1 1 1 0 1 1; 1 1 1 0 1 1];

C_red='red';                  % red

X_red = L*(X_red-1) + xc_red;
Y_red = L*(Y_red) + yc_red;
Z_red = L*(Z_red-0.5) + zc_red;
%--------------------------------------------------------------------------


%--------------------------------------------------------------------------
%Drawing Green bars
xc_green = x_centroid_green/1000+0.2; 
yc_green = y_centroid_green/1000-0.32; 
zc_green = -0.2;    % coordinated of the center
alpha=0.8;           % transparency (max=1=opaque)

X_green = [0 0 0 0 0 1; 1 0 1 1 1 1; 1 0 1 1 1 1; 0 0 0 0 0 1];
Y_green = [0 0 0 0 1 0; 0 1 0 0 1 1; 0 1 1 1 1 1; 0 0 1 1 1 0];
Z_green = [0 0 1 0 0 0; 0 0 1 0 0 0; 1 1 1 0 1 1; 1 1 1 0 1 1];

C_green='green';                  % green

X_green = L*(X_green-1) + xc_green;
Y_green = L*(Y_green) + yc_green;
Z_green = L*(Z_green-0.5) + zc_green;
%--------------------------------------------------------------------------


%--------------------------------------------------------------------------
%Drawing Blue bars
xc_blue = x_centroid_blue/1000+0.2; 
yc_blue = y_centroid_blue/1000-0.32; 
zc_blue = -0.2;    % coordinated of the center
alpha=0.8;           % transparency (max=1=opaque)

X_blue = [0 0 0 0 0 1; 1 0 1 1 1 1; 1 0 1 1 1 1; 0 0 0 0 0 1];
Y_blue = [0 0 0 0 1 0; 0 1 0 0 1 1; 0 1 1 1 1 1; 0 0 1 1 1 0];
Z_blue = [0 0 1 0 0 0; 0 0 1 0 0 0; 1 1 1 0 1 1; 1 1 1 0 1 1];

C_blue='blue';                  % blue

X_blue = L*(X_blue-1) + xc_blue;
Y_blue = L*(Y_blue) + yc_blue;
Z_blue = L*(Z_blue-0.5) + zc_blue;
%--------------------------------------------------------------------------


%--------------------------------------------------------------------------
%Drawing Yellow bars
xc_yellow = x_centroid_yellow/1000+0.2; 
yc_yellow = y_centroid_yellow/1000-0.32; 
zc_yellow = -0.2;    % coordinated of the center
alpha=0.8;           % transparency (max=1=opaque)

X_yellow = [0 0 0 0 0 1; 1 0 1 1 1 1; 1 0 1 1 1 1; 0 0 0 0 0 1];
Y_yellow = [0 0 0 0 1 0; 0 1 0 0 1 1; 0 1 1 1 1 1; 0 0 1 1 1 0];
Z_yellow = [0 0 1 0 0 0; 0 0 1 0 0 0; 1 1 1 0 1 1; 1 1 1 0 1 1];

C_yellow='yellow';                  % yellow

X_yellow = L*(X_yellow-1) + xc_yellow;
Y_yellow = L*(Y_yellow) + yc_yellow;
Z_yellow = L*(Z_yellow-0.5) + zc_yellow;
%--------------------------------------------------------------------------

L1=Link([0 0.1916 0.4 0]);
L2=Link([0 0 0.25 pi]);
L3=Link([0 0.18 0 0 1]);
L4=Link([0 0.05 0 0]);

L3.qlim = [0 0.18]
zlim([0 1])
robot=SerialLink([L1 L2 L3 L4])

%Plot the bar object on the same plot as the robot
fill3(X_red,Y_red,Z_red,C_red,'FaceAlpha',alpha);
hold on;
fill3(X_green,Y_green,Z_green,C_green,'FaceAlpha',alpha)
hold on;
fill3(X_blue,Y_blue,Z_blue,C_blue,'FaceAlpha',alpha)
hold on;
fill3(X_yellow,Y_yellow,Z_yellow,C_yellow,'FaceAlpha',alpha);

%Inverse Kinematics Solution (for each coloured bar)
%For red bar
theta2_red = acos(((x_centroid_red/1000+0.2)^2+(y_centroid_red/1000-0.32)^2-0.285)/0.2);
a_red = ((y_centroid_red/1000-0.32)*(0.4+(0.25*cos(theta2_red))))-(0.25*(x_centroid_red/1000+0.2)*sin(theta2_red));
b_red = ((x_centroid_red/1000+0.2)*(0.4+(0.25*cos(theta2_red))))+(0.25*(y_centroid_red/1000-0.32)*sin(theta2_red));

theta1_red = atan(a_red/b_red)

%For green bar
theta2_green = acos(((x_centroid_green/1000+0.2)^2+(y_centroid_green/1000-0.32)^2-0.285)/0.2);
a_green = ((y_centroid_green/1000-0.32)*(0.4+(0.25*cos(theta2_green))))-(0.25*(x_centroid_green/1000+0.2)*sin(theta2_green));
b_green = ((x_centroid_green/1000+0.2)*(0.4+(0.25*cos(theta2_green))))+(0.25*(y_centroid_green/1000-0.32)*sin(theta2_green));

theta1_green = atan(a_green/b_green)

%For blue bar
theta2_blue = acos(((x_centroid_blue/1000+0.2)^2+(y_centroid_blue/1000-0.32)^2-0.285)/0.2);
a_blue = ((y_centroid_blue/1000-0.32)*(0.4+(0.25*cos(theta2_blue))))-(0.25*(x_centroid_blue/1000+0.2)*sin(theta2_blue));
b_blue = ((x_centroid_blue/1000+0.2)*(0.4+(0.25*cos(theta2_blue))))+(0.25*(y_centroid_blue/1000-0.32)*sin(theta2_blue));

theta1_blue = atan(a_blue/b_blue)

%For yellow bar
theta2_yellow = acos(((x_centroid_yellow/1000+0.2)^2+(y_centroid_yellow/1000-0.32)^2-0.285)/0.2);
a_yellow = ((y_centroid_yellow/1000-0.32)*(0.4+(0.25*cos(theta2_yellow))))-(0.25*(x_centroid_yellow/1000+0.2)*sin(theta2_yellow));
b_yellow = ((x_centroid_yellow/1000+0.2)*(0.4+(0.25*cos(theta2_yellow))))+(0.25*(y_centroid_yellow/1000-0.32)*sin(theta2_yellow));

theta1_yellow = atan(a_yellow/b_yellow)
%--------------------------------------------------------------------------

%1st iteration - Moving to red bar
for a=0:-0.05:theta1_red
    xlim([-0.75 0.75])
    ylim([-0.75 0.75])
    zlim([-0.2 1])
    robot.plot([a 0 0 0.05])
end

for b=0:0.1:theta2_red
    xlim([-0.75 0.75])
    ylim([-0.75 0.75])
    zlim([-0.2 1])
    robot.plot([theta1_red b 0 0.05])
end

for c=0:0.02:0.18
    xlim([-0.75 0.75])
    ylim([-0.75 0.75])
    zlim([-0.2 1])
    robot.plot([theta1_red theta2_red c 0.05])
end

pause(2)
%--------------------------------------------------------------------------
%2nd iteration - Moving to green bar
for a=0:-0.05:theta1_green
    xlim([-0.75 0.75])
    ylim([-0.75 0.75])
    zlim([-0.2 1])
    robot.plot([a 0 0 0.05])
end

for b=0:0.1:theta2_green
    xlim([-0.75 0.75])
    ylim([-0.75 0.75])
    zlim([-0.2 1])
    robot.plot([theta1_green b 0 0.05])
end

for c=0:0.02:0.18
    xlim([-0.75 0.75])
    ylim([-0.75 0.75])
    zlim([-0.2 1])
    robot.plot([theta1_green theta2_green c 0.05])
end
pause(2)
%--------------------------------------------------------------------------
%3rd iteration - Moving to blue bar
for a=0:-0.05:theta1_blue
    xlim([-0.75 0.75])
    ylim([-0.75 0.75])
    zlim([-0.2 1])
    robot.plot([a 0 0 0.05])
end

for b=0:0.1:theta2_blue
    xlim([-0.75 0.75])
    ylim([-0.75 0.75])
    zlim([-0.2 1])
    robot.plot([theta1_blue b 0 0.05])
end

for c=0:0.02:0.18
    xlim([-0.75 0.75])
    ylim([-0.75 0.75])
    zlim([-0.2 1])
    robot.plot([theta1_blue theta2_blue c 0.05])
end
pause(2)
%--------------------------------------------------------------------------
%4th iteration - Moving to yellow bar
for a=0:-0.05:theta1_yellow
    xlim([-0.75 0.75])
    ylim([-0.75 0.75])
    zlim([-0.2 1])
    robot.plot([a 0 0 0.05])
end

for b=0:0.1:theta2_yellow
    xlim([-0.75 0.75])
    ylim([-0.75 0.75])
    zlim([-0.2 1])
    robot.plot([theta1_yellow b 0 0.05])
end

for c=0:0.02:0.18
    xlim([-0.75 0.75])
    ylim([-0.75 0.75])
    zlim([-0.2 1])
    robot.plot([theta1_yellow theta2_yellow c 0.05])
end
%--------------------------------------------------------------------------