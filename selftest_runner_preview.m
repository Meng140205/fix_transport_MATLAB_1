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
transport=MockLiftTransport([q0;q0],rod,[left;right]);
runner=dualarm.Lift30Runner(urdf,sdf,0,false,transport);
report=runner.preview(urdf,sdf);
assert(report.actuator_commands_sent==0);
assert(report.nominal_qp_samples==1201);
fprintf('RUNNER_PREVIEW_PASS\n');
