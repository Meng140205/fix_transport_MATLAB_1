addpath(fileparts(mfilename('fullpath')));
root=fileparts(fileparts(mfilename('fullpath')));
urdf=fullfile(root,'lift_failure_review','actual_robot.urdf');
sdf=fullfile(root,'mpc_readonly_review_2026-11-16','rod_with_pose.sdf');
q0=[1.1519853;-2.1270778;2.0490423;-1.4927645;1.5707890;-0.4188037];
arms={dualarm.loadArm(urdf,'left',[.01 .01]),...
      dualarm.loadArm(urdf,'right',[.01 .01])};
[~,left]=dualarm.armEvaluate(arms{1},q0,zeros(6,1));
[~,right]=dualarm.armEvaluate(arms{2},q0,zeros(6,1));
rod=eye(4);rod(1:3,4)=[0;.29966;1.2717];
transport=MockExecuteTransport([q0;q0],rod,[left;right]);
runner=dualarm.Lift30Runner(urdf,sdf,0,true,transport);
transport.runner=runner;
folder=fullfile(fileparts(mfilename('fullpath')),'.validation');
if ~exist(folder,'dir'),mkdir(folder);end
cd(folder);
report=runner.execute(urdf,sdf);
assert(strcmp(report.result,'MATLAB_LIFT_30CM_PASS'));
assert(report.test_mode && transport.stopped && transport.sent>500);
fprintf('RUNNER_EXECUTE_MOCK_PASS plans=%d\n',transport.sent);
