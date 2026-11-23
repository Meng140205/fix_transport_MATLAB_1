addpath(fileparts(mfilename('fullpath')));
desired=zeros(12,1);desired([2 4 8 10])=.01;
[friction,extra]=dualarm.frictionFeedforward(ones(12,1)*3.5,...
    desired,zeros(12,1));
assert(max(abs(extra([2 4 8 10])-.8))<1e-12);
assert(max(abs(friction([2 4 8 10])-4.3))<1e-12);
state=[];
[state,~]=dualarm.trackingBiasStep(state,ones(12,1)*.02,0,true);
[state,value]=dualarm.trackingBiasStep(state,ones(12,1)*.02,.05,true);
assert(max(abs(value-.008))<1e-12);
[state,value]=dualarm.trackingBiasStep(state,ones(12,1)*.02,.05,true);
assert(max(abs(value-.008))<1e-12);
fprintf('BIAS_PASS breakaway=%.3f residual=%.3f\n',extra(4),value(4));
