function report=lift_30cm(mode,urdfFile,sdfFile,domainId,...
    supportRemoved,clearanceChecked)
%LIFT_30CM MATLAB 30 cm/60 s cooperative MPC. Preview is read-only.
% Execution requires explicit scene and clearance confirmations.
if nargin<4 || isempty(domainId),domainId=0;end
if nargin<5,supportRemoved=false;end
if nargin<6,clearanceChecked=false;end
if ~any(strcmp(mode,{'preview','execute'}))
    error('mode must be preview or execute');
end
execute=strcmp(mode,'execute');
if execute && ~(supportRemoved && clearanceChecked)
    error('Execution requires supportRemoved=true and clearanceChecked=true');
end
addpath(fileparts(mfilename('fullpath')));
if nargin<2 || isempty(urdfFile)
    urdfFile=dualarm.captureRobotURDF(domainId,...
        fullfile(fileparts(mfilename('fullpath')),'runtime'));
end
if nargin<3 || isempty(sdfFile)
    sdfFile=fullfile(fileparts(mfilename('fullpath')),...
        'model_data','rod_with_pose.sdf');
end
runner=dualarm.Lift30Runner(urdfFile,sdfFile,domainId,execute);
if execute
    report=runner.execute(urdfFile,sdfFile);
else
    report=runner.preview(urdfFile,sdfFile);
end
end
