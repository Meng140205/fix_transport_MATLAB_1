addpath(fileparts(mfilename('fullpath')));
root=fileparts(fileparts(mfilename('fullpath')));
urdf=fullfile(root,'lift_failure_review','actual_robot.urdf');
q0=[1.1519853;-2.1270778;2.0490423;-1.4927645;1.5707890;-0.4188037];
arms={dualarm.loadArm(urdf,'left',[.01 .01]),...
      dualarm.loadArm(urdf,'right',[.01 .01])};
rod=eye(4);rod(1:3,4)=[0;.29966;1.2717];
started=tic;
path=dualarm.liftReference(arms,[q0;q0],rod,.30,60);
fprintf('IK_30CM_PASS seconds=%.2f samples=%d delta=%.6f speed=%.6f sigma=%.6f\n',...
    toc(started),numel(path.grid),path.maxJointDisplacement,...
    path.maxNominalSpeed,path.minSigma);
