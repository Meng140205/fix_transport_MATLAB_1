function t=stampSeconds(stamp)
%STAMPSECONDS Convert a ROS 2 builtin_interfaces/Time message to seconds.
t=double(stamp.sec)+double(stamp.nanosec)*1e-9;
end
