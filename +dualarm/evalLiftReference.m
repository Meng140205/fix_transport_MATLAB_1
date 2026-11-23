function [q,dq,rod,velocity]=evalLiftReference(path,elapsed)
%EVALLIFTREFERENCE Interpolate the 41-point IK path at simulation time.
s=min(1,max(0,elapsed/path.duration));
fraction=10*s^3-15*s^4+6*s^5;
if elapsed>0 && elapsed<path.duration
    dfdt=(30*s^2-60*s^3+30*s^4)/path.duration;
else
    dfdt=0;
end
q=ppval(path.spline,fraction);q=q(:);
epsilon=1e-5;
left=max(0,fraction-epsilon);right=min(1,fraction+epsilon);
if right>left
    dqdf=(ppval(path.spline,right)-ppval(path.spline,left))/(right-left);
else
    dqdf=zeros(12,1);
end
dq=dqdf(:)*dfdt;
rod=path.rod0;
rod(3,4)=rod(3,4)+path.height*fraction;
velocity=[0;0;path.height*dfdt;0;0;0];
end
