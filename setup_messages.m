function setup_messages()
%SETUP_MESSAGES Build MATLAB types for the three deployed ROS 2 messages.
% Requires a supported C++ compiler selected with mex -setup cpp.
root=fileparts(mfilename('fullpath'));
source=fullfile(root,'msg_source');
ros2genmsg(source,BuildConfiguration='fasterruns');
types={'dualarm_mpc_msgs/TorquePlan',...
    'dualarm_mpc_msgs/PlanCommit',...
    'dualarm_mpc_msgs/ArmStatus'};
for i=1:numel(types)
    msg=ros2message(types{i});
    fprintf('MESSAGE_READY %s fields=%d\n',types{i},numel(fieldnames(msg)));
end
end
