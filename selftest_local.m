addpath(fileparts(mfilename('fullpath')));
root=fileparts(fileparts(mfilename('fullpath')));
urdf=fullfile(root,'lift_failure_review','actual_robot.urdf');
sdf=fullfile(root,'mpc_readonly_review_2026-11-16','rod_with_pose.sdf');
q0=[1.1519853;-2.1270778;2.0490423;-1.4927645;1.5707890;-0.4188037];
q=[q0;q0];dq=zeros(12,1);
arms={dualarm.loadArm(urdf,'left',[.01 .01]),...
      dualarm.loadArm(urdf,'right',[.01 .01])};
rod=eye(4);rod(1:3,4)=[0;.29966;1.2717];
grasp=cell(1,2);
for i=1:2
    sel=(6*i-5):(6*i);
    [~,~,~,tip]=dualarm.armEvaluate(arms{i},q(sel),dq(sel));
    grasp{i}=rod\tip;
end
payload=dualarm.rodInertia(sdf);
modelTimer=tic;
model=dualarm.localModel(arms,q,dq,rod,zeros(6,1),grasp,payload,zeros(12,1));
modelElapsed=toc(modelTimer);
assert(isequal(size(model.F),[18 12]));
assert(norm(model.error)<1e-10);
assert(all(isfinite([model.F(:);model.g;model.L(:);model.l])));
limits=[arms{1}.effort;arms{2}.effort];
[~,hL]=dualarm.armEvaluate(arms{1},q0,zeros(6,1));
[~,hR]=dualarm.armEvaluate(arms{2},q0,zeros(6,1));
previous=[hL;hR];
solution=dualarm.solveMPC(model,q,zeros(11,36),previous,...
    [arms{1}.lower;arms{2}.lower],...
    [arms{1}.upper;arms{2}.upper],limits,10,.05);
assert(solution.residual<=2e-4);
out=struct('F',model.F,'g',model.g,'A',model.Aclosure,...
    'error',model.error,'L',model.L,'l',model.l,...
    'rod_mass',payload.mass,'rod_inertia',payload.inertia,...
    'qp_torque',solution.torque,'qp_method',solution.method);
folder=fullfile(fileparts(mfilename('fullpath')),'.validation');
if ~exist(folder,'dir'),mkdir(folder);end
file=fopen(fullfile(folder,'local.json'),'w');
fprintf(file,'%s',jsonencode(out));fclose(file);
fprintf('LOCAL_MODEL_PASS mass=%.9f max_error=%.3g\n',payload.mass,max(abs(model.error)));
fprintf('TIMING model_ms=%.2f qp_ms=%.2f method=%s\n',...
    modelElapsed*1000,solution.elapsed*1000,solution.method);
