function [pose] = msg2pose(msg)

    pose_msg = msg.Pose;

    if(strcmp(pose_msg.MessageType, "geometry_msgs/PoseWithCovariance"))
        pose_msg = pose_msg.Pose;
    end

    o = pose_msg.Orientation;
    p = pose_msg.Position;

    q = quaternion(o.W,o.X, o.Y, o.Z);
    w = quat2eul(q);

    pose = [p.X, p.Y, w(1)];
end

