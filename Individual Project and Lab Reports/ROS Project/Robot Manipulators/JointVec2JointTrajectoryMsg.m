function [ jointTrajectoryMsg ] = JointVec2JointTrajectoryMsg(robot, q, t, qvel, qacc)
%  converts sequence of joint configuration q_1,...,q_n to  joint Trajectory message
% q : n x 1 , vector of gripper states , n #waypoints
% t : n x 1 vector of time stamps of motion in seconds
% qvel : n x 1 , vector of gripper velocities, n #waypoints
% qacc : n x 1 , vector of gripper accelerations,  n #waypoints
% assume Gripper Joint Name 

if (nargin < 5)
    qacc=0*q;
end

if (nargin < 4)
    qvel=0*q;
end


jointConf=homeConfiguration(robot);
jointTrajectoryMsg=rosmessage('trajectory_msgs/JointTrajectory');
% assert(isequal(length(jointConf),size(q,2)),'mismatch of length of robot joint variables and joint vector');
for i=1:length(t)-1
    assert(t(i+1)>t(i),'time stamps not in ascending order');
end
assert(isequal(size(q,1),length(t)),'#waypoints and #time stamps does not match');
for i=1:size(q,2) % dimension of joint vector
  jointTrajectoryMsg.JointNames(i)=cellstr(jointConf(i).JointName);  % assume first m JointNames correspond to q_1,..,_q_m
end


for i=1:size(q,1)
    jointTrajectoryMsg.Points(i)=rosmessage('trajectory_msgs/JointTrajectoryPoint');
    jointTrajectoryMsg.Points(i).Positions=q(i,:);
    jointTrajectoryMsg.Points(i).Velocities=qvel(i,:);
    jointTrajectoryMsg.Points(i).Accelerations=qacc(i,:);
    jointTrajectoryMsg.Points(i).TimeFromStart.Sec=floor(t(i));
    jointTrajectoryMsg.Points(i).TimeFromStart.Nsec=int32(10^9*(t(i)-floor(t(i))));
end

jointTrajectoryMsg.Header.Stamp=rostime('now');

% jointTrajectoryMsg.Header.Stamp.Nsec=jointTrajectoryMsg.Header.Stamp.Nsec + 10^7; % time stamp 10 ms into the future
end

