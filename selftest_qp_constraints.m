addpath(fileparts(mfilename('fullpath')));
root=fileparts(fileparts(mfilename('fullpath')));
urdf=fullfile(root,'lift_failure_review','actual_robot.urdf');
q0=[1.1519853;-2.1270778;2.0490423;-1.4927645;1.5707890;-0.4188037];
arms={dualarm.loadArm(urdf,'left',[.01 .01]),...
      dualarm.loadArm(urdf,'right',[.01 .01])};
q=[q0;q0];dq=zeros(12,1);
model=dualarm.localModel(arms,q,dq,[],[],[],[],zeros(12,1));
[~,hL]=dualarm.armEvaluate(arms{1},q0,zeros(6,1));
[~,hR]=dualarm.armEvaluate(arms{2},q0,zeros(6,1));
previous=[hL;hR];
lower=[arms{1}.lower;arms{2}.lower];
upper=[arms{1}.upper;arms{2}.upper];
limits=[arms{1}.effort;arms{2}.effort];
target=zeros(12,1);target([4 10])=.05;
result=dualarm.solveMPC(model,q,target,previous,lower,upper,limits,10,.05);
assert(result.residual<=2e-4);
assert(max(abs(result.torque-previous))<=.900001);
fprintf('CONSTRAINED_QP_PASS method=%s residual=%.3g first_step_delta=%.3f Nm\n',...
    result.method,result.residual,max(abs(result.torque-previous)));
