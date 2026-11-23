function path=captureRobotURDF(domainId,folder)
%CAPTUREROBOTURDF Read the active robot_description ROS 2 parameter.
% This avoids copying a stale URDF from Ubuntu to the Windows host.
if nargin<2 || isempty(folder),folder=fullfile(pwd,'matlab_mpc_runtime');end
if ~exist(folder,'dir'),mkdir(folder);end
client=ros2param('/robot_state_publisher',DomainID=domainId);
xml=get(client,'robot_description');
xml=char(xml);
if ~contains(xml,'<robot') || ~contains(xml,'left_joint1') || ...
        ~contains(xml,'right_joint6')
    error('Active robot_description is not the expected dual-arm URDF');
end
path=fullfile(folder,'dualarm_effort_active.urdf');
file=fopen(path,'w');
if file<0,error('Cannot write active robot description');end
cleanup=onCleanup(@()fclose(file)); %#ok<NASGU>
fwrite(file,xml,'char');
end
