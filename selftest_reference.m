addpath(fileparts(mfilename('fullpath')));
root=fileparts(fileparts(mfilename('fullpath')));
urdf=fullfile(root,'lift_failure_review','actual_robot.urdf');
q0=[1.1519853;-2.1270778;2.0490423;-1.4927645;1.5707890;-0.4188037];
arms={dualarm.loadArm(urdf,'left',[.01 .01]),...
      dualarm.loadArm(urdf,'right',[.01 .01])};
rod=eye(4);rod(1:3,4)=[0;.29966;1.2717];
started=tic;
path=dualarm.liftReference(arms,[q0;q0],rod,.02,8);
fprintf('IK_PASS seconds=%.3f displacement=%.9f speed=%.9f sigma=%.9f\n',...
    toc(started),path.maxJointDisplacement,path.maxNominalSpeed,path.minSigma);
sample=zeros(5,12);
for i=1:5
    [q,dq,pr,v]=dualarm.evalLiftReference(path,(i-1)*2);
    sample(i,:)=q.';
    assert(all(isfinite([q;dq;pr(:);v])));
end
settled=rod;settled(3,4)=settled(3,4)-.0012;
[anchored,grasp,reanchor]=dualarm.reanchorLift(path,arms,...
    [q0;q0],zeros(12,1),ones(4,1)*.01,settled,zeros(6,1),...
    23,23,23,ones(4,1)*.01);
assert(abs(reanchor.shift_m-.0012)<1e-12);
assert(abs(anchored.rod0(3,4)-settled(3,4))<1e-12);
assert(numel(grasp)==2);
folder=fullfile(fileparts(mfilename('fullpath')),'.validation');
if ~exist(folder,'dir'),mkdir(folder);end
out=struct('q_samples',sample,'max_displacement',path.maxJointDisplacement,...
    'max_speed',path.maxNominalSpeed,'min_sigma',path.minSigma,...
    'max_position_error',path.maxPositionError,...
    'max_rotation_error',path.maxRotationError);
file=fopen(fullfile(folder,'reference.json'),'w');
fprintf(file,'%s',jsonencode(out));fclose(file);
