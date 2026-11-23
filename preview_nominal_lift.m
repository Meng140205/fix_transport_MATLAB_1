function report=preview_nominal_lift(urdfFile,sdfFile,q0,rod0,height,duration,previous,fingers)
%PREVIEW_NOMINAL_LIFT Read-only MATLAB IK and full-duration QP precheck.
% No ROS nodes or actuator publishers are created.
addpath(fileparts(mfilename('fullpath')));
q0=q0(:);
if numel(q0)~=12,error('Expected 12 arm positions');end
if nargin<5 || isempty(height),height=.30;end
if nargin<6 || isempty(duration),duration=60;end
if nargin<8 || isempty(fingers),fingers=ones(4,1)*.01;end
fingers=fingers(:);
if numel(fingers)~=4,error('Expected four finger positions');end
arms={dualarm.loadArm(urdfFile,'left',fingers(1:2).'),...
      dualarm.loadArm(urdfFile,'right',fingers(3:4).')};
payload=dualarm.rodInertia(sdfFile);
path=dualarm.liftReference(arms,q0,rod0,height,duration);
grasp=cell(1,2);
useMeasured=nargin>=7 && ~isempty(previous);
if ~useMeasured,previous=zeros(12,1);end
previous=previous(:);
if numel(previous)~=12,error('Expected 12 previous torques');end
for i=1:2
    idx=(6*i-5):(6*i);
    [~,gravity,~,tip]=dualarm.armEvaluate(arms{i},q0(idx),zeros(6,1));
    if ~useMeasured,previous(idx)=gravity;end
    grasp{i}=rod0\tip;
end
lower=[arms{1}.lower;arms{2}.lower];
upper=[arms{1}.upper;arms{2}.upper];
limits=[arms{1}.effort;arms{2}.effort];
mu=[arms{1}.friction;arms{2}.friction];
maximumTorque=zeros(12,1);maximumResidual=0;
sampleCount=round(duration/.05)+1;
methods=cell(sampleCount,1);modelTime=zeros(sampleCount,1);qpTime=zeros(sampleCount,1);
for sample=1:sampleCount
    elapsed=(sample-1)*.05;
    [q,dq,rod,vo]=dualarm.evalLiftReference(path,elapsed);
    friction=dualarm.frictionFeedforward(mu,dq,zeros(12,1));
    started=tic;
    model=dualarm.localModel(arms,q,dq,rod,vo,grasp,payload,friction);
    modelTime(sample)=toc(started);
    refs=zeros(11,36);
    for k=0:10
        [qr,dqr,pr,vr]=dualarm.evalLiftReference(path,elapsed+k*.05);
        poseError=[pr(1:3,4)-rod(1:3,4);...
            dualarm.rotLog(pr(1:3,1:3)*rod(1:3,1:3).')];
        refs(k+1,:)=[qr-q;poseError;dqr;vr].';
    end
    solved=dualarm.solveMPC(model,q,refs,previous,lower,upper,limits,10,.05);
    maximumTorque=max(maximumTorque,abs(solved.torque));
    maximumResidual=max(maximumResidual,solved.residual);
    previous=solved.torque;
    methods{sample}=solved.method;
    qpTime(sample)=solved.elapsed;
end
report=struct('result','MATLAB_NOMINAL_LIFT_PRECHECK_PASS',...
    'height_m',height,'duration_s',duration,'ik_samples',numel(path.grid),...
    'nominal_qp_samples',sampleCount,'actuator_commands_sent',0,...
    'max_joint_displacement_rad',path.maxJointDisplacement,...
    'max_nominal_joint_speed_rad_s',path.maxNominalSpeed,...
    'min_scaled_jacobian_singular_value',path.minSigma,...
    'max_abs_torque_Nm',maximumTorque.',...
    'max_constraint_residual',maximumResidual,...
    'quadprog_calls',sum(strcmp(methods,'quadprog')),...
    'median_model_ms',median(modelTime)*1000,...
    'median_qp_ms',median(qpTime)*1000);
disp(jsonencode(report));
end
