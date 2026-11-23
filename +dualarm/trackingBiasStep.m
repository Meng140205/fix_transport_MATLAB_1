function [state,value] = trackingBiasStep(state,trackingError,stamp,active)
%TRACKINGBIASSTEP Bounded simulation-time residual estimate.
trackingError=trackingError(:);
if numel(trackingError)~=12 || any(~isfinite(trackingError)) || ~isfinite(stamp)
    error('Invalid tracking-bias input');
end
if isempty(state)
    state=struct('value',zeros(12,1),'lastStamp',stamp,...
        'gain',8,'limit',.5);
    value=state.value;
    return
end
dt=stamp-state.lastStamp;
if dt<0,error('Residual-estimator clock reset');end
state.lastStamp=stamp;
if dt>0 && active
    state.value=min(state.limit,max(-state.limit,...
        state.value+state.gain*trackingError*min(dt,.1)));
end
value=state.value;
end
