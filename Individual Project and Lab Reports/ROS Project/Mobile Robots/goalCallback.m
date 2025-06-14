function [] = goalCallback(~,msg,goalHandle)
    pose = msg2pose(msg);
    goalHandle.x = pose(1);
    goalHandle.y = pose(2);
    goalHandle.theta = pose(3);
    
    fprintf("Received new navigation goal {%f, %f, %f}\n", pose(1), pose(2), pose(3));
end