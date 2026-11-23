function stamp=makeStamp(secondsValue)
%MAKESTAMP Convert simulation seconds to a ROS 2 Time field struct.
if ~isfinite(secondsValue) || secondsValue<0,error('Invalid ROS time');end
nanoseconds=int64(round(secondsValue*1e9));
stamp=struct('sec',int32(idivide(nanoseconds,int64(1e9),'floor')),...
    'nanosec',uint32(mod(nanoseconds,int64(1e9))));
end
