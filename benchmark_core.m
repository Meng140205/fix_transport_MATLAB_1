addpath(fileparts(mfilename('fullpath')));
root=fileparts(fileparts(mfilename('fullpath')));
urdf=fullfile(root,'lift_failure_review','actual_robot.urdf');
sdf=fullfile(root,'mpc_readonly_review_2026-11-16','rod_with_pose.sdf');
q0=[1.1519853;-2.1270778;2.0490423;-1.4927645;1.5707890;-0.4188037];
q=[q0;q0];dq=zeros(12,1);
arms={dualarm.loadArm(urdf,'left',[.01 .01]),...
      dualarm.loadArm(urdf,'right',[.01 .01])};
rod=eye(4);rod(1:3,4)=[0;.29966;1.2717];
grasp=cell(1,2);previous=zeros(12,1);
for i=1:2
    sel=(6*i-5):(6*i);
    [~,previous(sel),~,tip]=dualarm.armEvaluate(arms{i},q(sel),dq(sel));
    grasp{i}=rod\tip;
end
payload=dualarm.rodInertia(sdf);
lo=[arms{1}.lower;arms{2}.lower];
hi=[arms{1}.upper;arms{2}.upper];
limits=[arms{1}.effort;arms{2}.effort];
modelTimes=zeros(8,1);qpTimes=zeros(8,1);
for k=1:8
    started=tic;
    model=dualarm.localModel(arms,q,dq,rod,zeros(6,1),grasp,payload,zeros(12,1));
    modelTimes(k)=toc(started)*1000;
    result=dualarm.solveMPC(model,q,zeros(11,36),previous,lo,hi,limits,10,.05);
    qpTimes(k)=result.elapsed*1000;
end
fprintf('WARM_TIMING model_ms_median=%.2f qp_ms_median=%.2f total_ms_median=%.2f\n',...
    median(modelTimes(2:end)),median(qpTimes(2:end)),...
    median(modelTimes(2:end)+qpTimes(2:end)));
dqMove=repmat([0;.001;-.01;.01;0;0],2,1);
movingTimes=zeros(6,1);
for k=1:6
    started=tic;
    dualarm.localModel(arms,q,dqMove,rod,[0;0;.002;0;0;0],...
        grasp,payload,zeros(12,1));
    movingTimes(k)=toc(started)*1000;
end
fprintf('MOVING_MODEL_MS median=%.2f\n',median(movingTimes(2:end)));
