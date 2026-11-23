addpath(fileparts(mfilename('fullpath')));
workspace=fileparts(fileparts(mfilename('fullpath')));
urdf=fullfile(workspace,'lift_failure_review','actual_robot.urdf');
q=[1.1519853;-2.1270778;2.0490423;-1.4927645;1.5707890;-0.4188037];
results=cell(1,2);index=0;
for side={'left','right'}
    index=index+1;
    arm=dualarm.loadArm(urdf,side{1},[0.01 0.01]);
    [M,h,J,tip]=dualarm.armEvaluate(arm,q,zeros(6,1));
    assert(all(size(M)==[6 6]));
    assert(min(eig(M))>0);
    assert(all(isfinite([M(:);h;J(:);tip(:)])));
    result=struct('side',side{1},'mass',M,'gravity',h,...
        'jacobian',J,'tip',tip);
    results{index}=result;
    fprintf('MODEL_JSON %s\n',jsonencode(result));
end
folder=fullfile(fileparts(mfilename('fullpath')),'.validation');
if ~exist(folder,'dir'),mkdir(folder);end
file=fopen(fullfile(folder,'model.json'),'w');
fprintf(file,'%s',jsonencode(results));fclose(file);
