function [command,baseline,change] = composeTorque(alpha,mpcTorque,...
    anchorTau,currentGravity,anchorGravity,kp,kd,...
    anchorQ,q,anchorDq,dq,actual,limits)
%COMPOSETORQUE Match the Python HOLD-to-MPC blend and plan guards.
values={mpcTorque,anchorTau,currentGravity,anchorGravity,kp,kd,...
    anchorQ,q,anchorDq,dq,actual,limits};
for i=1:numel(values)
    values{i}=values{i}(:);
    if numel(values{i})~=12 || any(~isfinite(values{i}))
        error('Torque blend requires finite 12-vectors');
    end
end
[mpcTorque,anchorTau,currentGravity,anchorGravity,kp,kd,...
    anchorQ,q,anchorDq,dq,actual,limits]=deal(values{:});
if ~isscalar(alpha) || ~isfinite(alpha) || alpha<0 || alpha>1
    error('Invalid blend fraction');
end
baseline=anchorTau+currentGravity-anchorGravity+...
    kp.*(anchorQ-q)-kd.*(dq-anchorDq);
command=(1-alpha)*baseline+alpha*mpcTorque;
change=max(abs(command-actual));
if any(abs(command)>limits) || change>1
    error('Proposed absolute torque exceeds controller limits');
end
end
