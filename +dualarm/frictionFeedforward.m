function [friction,breakaway] = frictionFeedforward(mu,desired,actual)
%FRICTIONFEEDFORWARD Stage 4.3 prediction bias; absolute joint torques.
mu=mu(:);desired=desired(:);actual=actual(:);
if numel(mu)~=12 || numel(desired)~=12 || numel(actual)~=12 || ...
        any(~isfinite([mu;desired;actual]))
    error('Expected finite 12-joint friction inputs');
end
margin=repmat([0;.8;0;.8;0;0],2,1);
direction=tanh(desired/.0002);
breakaway=margin.*direction.*exp(-(actual/.005).^2);
friction=mu.*direction+breakaway;
end
