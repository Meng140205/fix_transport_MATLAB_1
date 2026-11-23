function path=liftReference(arms,q0,rod0,height,duration)
%LIFTREFERENCE Two-arm IK and quintic vertical rod reference.
if nargin<4 || isempty(height),height=.02;end
if nargin<5 || isempty(duration),duration=8;end
q0=q0(:);
if numel(q0)~=12 || ~isequal(size(rod0),[4 4])
    error('Expected 12 arm joints and a 4x4 rod pose');
end
if height<=.02
    samples=41;
else
    samples=61;  % Same long-path sampling as the 30 cm Python IK preview.
end
grid=linspace(0,1,samples);
jointPath=zeros(12,numel(grid));
q=q0;
tips=cell(1,2);
for i=1:2
    sel=(6*i-5):(6*i);
    [tips{i},~]=dualarm.armGeometry(arms{i},q(sel));
end
maxPositionError=0;maxRotationError=0;minimumSigma=Inf;
options=optimoptions('lsqnonlin','Display','off',...
    'MaxFunctionEvaluations',80,'FunctionTolerance',1e-11,...
    'StepTolerance',1e-11,'OptimalityTolerance',1e-11);
for sample=1:numel(grid)
    for i=1:2
        sel=(6*i-5):(6*i);
        target=tips{i};
        target(3,4)=target(3,4)+height*grid(sample);
        fun=@(qi) poseResidual(arms{i},qi,target);
        [answer,~,~,flag]=lsqnonlin(fun,q(sel),...
            arms{i}.lower+.01,arms{i}.upper-.01,options);
        residual=fun(answer);
        ep=norm(residual(1:3));
        er=norm(residual(4:6))/.2;
        if flag<=0 || ep>1e-5 || er>1e-4
            error('IK path failed at fraction %.3f, arm %d',grid(sample),i);
        end
        q(sel)=answer;
        [~,J]=dualarm.armGeometry(arms{i},answer);
        J(4:6,:)=.2*J(4:6,:);
        minimumSigma=min(minimumSigma,min(svd(J)));
        maxPositionError=max(maxPositionError,ep);
        maxRotationError=max(maxRotationError,er);
    end
    jointPath(:,sample)=q;
end
maxJointDisplacement=max(abs(jointPath(:)-repmat(q0,numel(grid),1)));
if (height<=.02 && maxJointDisplacement>.12) || minimumSigma<.005
    error('IK path exceeds local movement/singularity bounds');
end
pp=pchip(grid,jointPath);
path=struct('grid',grid,'joints',jointPath,'spline',pp,...
    'q0',q0,'rod0',rod0,'height',height,'duration',duration,...
    'minSigma',minimumSigma,'maxPositionError',maxPositionError,...
    'maxRotationError',maxRotationError,...
    'maxJointDisplacement',maxJointDisplacement);
speed=0;
for t=linspace(0,duration,101)
    [~,dq]=dualarm.evalLiftReference(path,t);
    speed=max(speed,max(abs(dq)));
end
path.maxNominalSpeed=speed;
if speed>.08,error('Nominal joint path too fast');end
end

function residual=poseResidual(arm,q,target)
[actual,~]=dualarm.armGeometry(arm,q);
residual=[actual(1:3,4)-target(1:3,4);...
    .2*dualarm.rotLog(actual(1:3,1:3)*target(1:3,1:3).')];
end
