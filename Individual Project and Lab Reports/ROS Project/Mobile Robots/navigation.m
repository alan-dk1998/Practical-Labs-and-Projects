% launch gazebo
% roslaunch turtlebot3_gazebo turtlebot3_rst_lab.launch
% roslaunch turtlebot3_navigation turtlebot3_navigation_rst_lab.launch

%% 1) Initiate Matlab to ROS communication
clear all;
close all;
rosshutdown;
% setenv("ROS_IP", "192.168.153.128")
rosinit("192.168.153.128")
disp("Connected to ROS");


%% Configuration
rx_timeout = 15;
ss_min_turning_radius = 0.3;
sv_validation_distance = 0.005;
rrt_max_connection_distance = 1.6;

prm_max_connection_distance = 3;
prm_max_num_nodes = 750;

teb_max_timeout = 15000;
teb_run = true;
teb_frequency = 5;

%% 2) Initiate a velocity publisher and a subscriber for the current pose ("/odom") 
velPub = rospublisher('/cmd_vel');
odomSub = rossubscriber('/odom');
disp("Initiated subscribers");

%% 3) Initiate a goal subscriber to the "/move_base_simple/goal" topic from
% RVIZ with a subscriber callback that fills the values of the goal handle.
%----------
goalHandle = PoseHandle();

% initiate goal with something so we don't have to switch to the virtual
% every time
goalSub = rossubscriber('/move_base_simple/goal', {@goalCallback, goalHandle});

disp("Select target pose");
goalSub.receive();
%----------
% goalHandle = PoseHandle();
% 
% % initiate goal with something so we don't have to switch to the virtual
% % every time
% goalHandle.x = 6;
% goalHandle.y = 3;
% goalHandle.theta = 0;
% 
% goalSub = rossubscriber('/move_base_simple/goal',{@goalCallback, goalHandle});

disp("Installed goal callback");

%% 4) Load the map and display it. Also add the goal pose.

map_img = imread('rst_lab_cropped.pgm');

[msg,~,~] = odomSub.receive(rx_timeout);
robot_pose = msg2pose(msg);

fprintf("Received robot inital pose: {%f, %f, %f}\n", robot_pose(1), robot_pose(2), robot_pose(3));

% Required steps to align the map correctly (do not change!)
bwimage = map_img < 100;
bwimage = imrotate(bwimage, 90);
map = binaryOccupancyMap(bwimage,1/0.05);
map.GridLocationInWorld = [-4, -3];

figure(Name="Map");
map.show;

hold on;
plot(robot_pose(1),robot_pose(2),'go',MarkerSize=8);    % robot pose
plot(goalHandle.x,goalHandle.y,'rx',MarkerSize=8);      % goal pose
legend("Robot", "Goal");

%% 5) Implement PRM and RRT as global path planners for the lab environment regarding the current robot pose.

% disp("Select target pose");
% goalSub.receive();


% get current pose
[msg,~,~] = odomSub.receive(rx_timeout);
start_pose = msg2pose(msg);
goal_pose = handle2pose(goalHandle);


planners = ["PRM"; "RRT"];

options = optimizePathOptions();
options.MinTurningRadius = 0.08;
options.MaxPathStates = 3500;
options.ObstacleSafetyMargin = 0.5;
options.WeightVelocity = 1000;
options.NumIteration = 4;
options.MaxSolverIteration = 15;

for k=1:size(planners)
    figure(Name=planners(k));
    map.show;
    xlim(map.XWorldLimits);
    ylim(map.YWorldLimits);

    hold on
    plot(start_pose(1), start_pose(2), "*", "Color", "g", "LineWidth",3);
    plot(goal_pose(1), goal_pose(2), "*", "Color", "r", "LineWidth", 3);
    legend("Start", "Goal");
    
    if planners(k)=="PRM"
        title("PRM");
        disp("Executing PRM");

        stateSpace = stateSpaceSE2();
        stateSpace.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
        stateValidator = validatorOccupancyMap(stateSpace, 'Map', map);
        stateValidator.ValidationDistance = sv_validation_distance;
        
    
        planner = plannerPRM(stateSpace, stateValidator, ...
        'MaxConnectionDistance', prm_max_connection_distance, ...
        'MaxNumNodes', prm_max_num_nodes);
    
        graph = graphData(planner);
        nodes = table2array(graph.Nodes);
        edges = table2array(graph.Edges);
    
        % visualize roadmap
        plot(nodes(:,1),nodes(:,2),"*","Color","b","LineWidth",2,'HandleVisibility','off')
    
        for i = 1:size(edges,1)
            % Samples states at distance 0.02 meters.
            x = [nodes(edges(i,1),1), nodes(edges(i,2),1)];
            y = [nodes(edges(i,1),2), nodes(edges(i,2),2)];
    
            % states = interpolate(...);
            % draw edges
            plot(x, y,"Color","b",'HandleVisibility','off')
        end
    
        % plan path from start to goal
        [path,~] = planner.plan(start_pose, goal_pose);
        plannedPath = path.States;
    
    elseif planners(k)=="RRT"
        title("RRT");
        disp("Executing RRT");

        stateSpace = stateSpaceDubins();
        stateSpace.MinTurningRadius = ss_min_turning_radius;
        stateSpace.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
        
        stateValidator = validatorOccupancyMap(stateSpace, 'Map', map);
        stateValidator.ValidationDistance = sv_validation_distance;
    
        planner = plannerRRT(stateSpace, stateValidator , ...
        'MaxConnectionDistance', rrt_max_connection_distance);
    
        [path,solnInfo] = planner.plan(start_pose, goal_pose);   
        
        % Plot Tree expansion
        plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-','HandleVisibility','off');
        plannedPath=path.States;

        disp("Executing RRT optimization");
    else
        error("Unknown global planner.")
    end



    % Optimize the path generated by the planner.
    [optimizedPath,kineticInfo,solutionInfo] = optimizePath(plannedPath, map, options);
    plot(optimizedPath(:,1),optimizedPath(:,2),'g-','LineWidth',2,'HandleVisibility','off');

    plot(plannedPath(:,1),plannedPath(:,2),'r-','LineWidth',2,'HandleVisibility','off');

    if planners(k)=="PRM"
        plannedPath_prm = plannedPath;
        optimizedPath_prm = optimizedPath;
    else
        plannedPath_rrt = plannedPath;
        optimizedPath_rrt = optimizedPath;
    end
    drawnow;
    hold off
end



%% 6) Optimize the path and implement the teb planner for path following and obstacle avoidance

use_prm = false;
figure;

if teb_run
    if(use_prm)
        refpath = optimizedPath_prm;
    else
        refpath = optimizedPath_rrt;
    end


    % Initiate TEB with some parameters
    teb=controllerTEB(refpath, map);
    teb.RobotInformation.Shape="Rectangle";
    teb.RobotInformation.Dimension = [0.2 0.2];
    teb.LookAheadTime=5.0;
    teb.MaxVelocity=[0.5 1.5];
    teb.MaxAcceleration=[0.5 0.5];
    teb.ReferenceDeltaTime=0.3;
    teb.ObstacleSafetyMargin = 0.1;
    % teb.GoalTolerance= [0.1 0.1 0.1];

    % Init values for variables
    localmap = binaryOccupancyMap(15,15,map.Resolution);

    % Current velocity and pose
    [msg,~,~] = odomSub.receive(rx_timeout);
    curpose = msg2pose(msg);

    v = msg.Twist.Twist;
    curvel = [v.Linear.X, v.Angular.Z];

    goalReached = false;
    runTime=[];
    cmd_vel = rosmessage(velPub);

    rateObj=rateControl(teb_frequency);
    % Stop if goal reached or maximum sim time 

    while ~goalReached && rateObj.TotalElapsedTime < teb_max_timeout
        % Update map to keep robot in the center of the map. Also update the
        % map with new information from the global map or sensor measurements.
        moveMapBy = curpose(1:2) - localmap.XLocalLimits(end)/2;
        localmap.move(moveMapBy,FillValue=0.5);
        syncWith(localmap,map);

        % Get pose
        [msg,~,~] = odomSub.receive(rx_timeout);
        curpose = msg2pose(msg);

        % Take last velocity
        v = msg.Twist.Twist;
        curvel = [v.Linear.X, v.Angular.Z];

        % Generate new vel commands with teb
        tic;
        [velcmds,tstamps,curpath,info] = step(teb,curpose,curvel);
        toc;

        % Check if goal is reached
        goalReached = info.HasReachedGoal;
        feasibleDriveDuration = tstamps(info.LastFeasibleIdx);

        if info.LastFeasibleIdx ~= height(tstamps) && ...
                feasibleDriveDuration < (teb.LookAheadTime/3)
            route = planner.plan(curpose, goal_pose);
            refpath = route.States;
            headingToNextPose = headingFromXY(refpath(:,1:2));
            refpath(2:end-1,3) = headingToNextPose(2:end-1);
            teb.ReferencePath = refpath;
        end
        
        %for k = 1:size(velcmds,1)-1
            cmd_vel.Linear.X = velcmds(1,1);
            cmd_vel.Angular.Z = velcmds(1,2);
            velPub.send(cmd_vel);
            %pause(tstamps(k+1)-tstamps(k));
        %end
        waitfor(rateObj);
    end
end
